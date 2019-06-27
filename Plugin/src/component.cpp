#include "xpcf/module/ModuleFactory.h"
#include "PipelineNaturalImageMarker.h"
#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::PipelineNaturalImageMarker)

#define TRACKING

namespace SolAR {
using namespace datastructure;
using namespace api::pipeline;
namespace PIPELINES {

PipelineNaturalImageMarker::PipelineNaturalImageMarker():ConfigurableBase(xpcf::toUUID<PipelineNaturalImageMarker>())
{
    declareInterface<api::pipeline::IPipeline>(this);
    declareInjectable<input::devices::ICamera>(m_camera);
    declareInjectable<features::IKeypointDetector>(m_kpDetector);
    declareInjectable<input::files::IMarker2DNaturalImage>(m_naturalImagemarker);
    declareInjectable<features::IMatchesFilter>(m_geomMatchesFilter);
    declareInjectable<features::IDescriptorMatcher>(m_matcher);
    declareInjectable<features::IDescriptorsExtractor>(m_descriptorExtractor);
    declareInjectable<features::IMatchesFilter>(m_basicMatchesFilter);
    declareInjectable<geom::IImage2WorldMapper>(m_img_mapper);
    declareInjectable<features::IKeypointsReIndexer>(m_keypointsReindexer);
    declareInjectable<sink::ISinkPoseImage>(m_sink);
    declareInjectable<source::ISourceImage>(m_source);
    declareInjectable<image::IImageConvertor>(m_imageConvertorUnity,"imageConvertorUnity");
    declareInjectable<features::IKeypointDetectorRegion>(m_kpDetectorRegion);
    declareInjectable<geom::IProject>(m_projection);
    declareInjectable<geom::IUnproject>(m_unprojection);
    declareInjectable<solver::pose::I3DTransformSACFinderFrom2D3D>(m_poseEstimationPlanar);
    declareInjectable<tracking::IOpticalFlowEstimator>(m_opticalFlowEstimator);

    declareProperty("updateTrackedPointThreshold", m_updateTrackedPointThreshold);
    declareProperty("detectionMatchesNumberThreshold", m_detectionMatchesNumberThreshold);

    m_initOK = false;
    m_startedOK = false;
    m_stopFlag = false;
    m_haveToBeFlip = false;
    m_taskGetCameraImages = nullptr;
    m_taskDetection = nullptr;
    m_taskTracking = nullptr;
    LOG_DEBUG(" Pipeline constructor");
}

PipelineNaturalImageMarker::~PipelineNaturalImageMarker()
{
    if(m_taskDetection != nullptr)
        delete m_taskDetection;
    LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode PipelineNaturalImageMarker::init(SRef<xpcf::IComponentManager> xpcfComponentManager)
{
    // load marker
    LOG_INFO("LOAD MARKER IMAGE ");
    m_naturalImagemarker->loadMarker();
    m_naturalImagemarker->getImage(m_refImage);
    m_naturalImagemarker->getWorldCorners(m_markerWorldCorners);


    // detect keypoints in reference image
    LOG_INFO("DETECT MARKER KEYPOINTS ");
    m_kpDetector->detect(m_refImage, m_refKeypoints);

    // extract descriptors in reference image
    LOG_INFO("EXTRACT MARKER DESCRIPTORS ");
    m_descriptorExtractor->extract(m_refImage, m_refKeypoints, m_refDescriptors);
    LOG_INFO("EXTRACT MARKER DESCRIPTORS COMPUTED");

    // initialize image mapper with the reference image size and marker size

    LOG_INFO(" worldWidth : {} worldHeight : {} \n",m_naturalImagemarker->getSize().width,m_naturalImagemarker->getSize().height)

    m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(m_refImage->getSize().width);
    m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(m_refImage->getSize().height);
    m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(m_naturalImagemarker->getSize().width);
    m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(m_naturalImagemarker->getSize().height);


    Point2Df corner0(0, 0);
    Point2Df corner1((float)m_refImage->getWidth(), 0);
    Point2Df corner2((float)m_refImage->getWidth(), (float)m_refImage->getHeight());
    Point2Df corner3(0, (float)m_refImage->getHeight());
    m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner0));
    m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner1));
    m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner2));
    m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner3));

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            m_pose(i,j)=0.f;

    // initialize pose estimation based on planar points with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
    m_poseEstimationPlanar->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());

    // initialize projection component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
    m_projection->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());

    // initialize unprojection component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
    m_unprojection->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());

    m_isTrack = false;
    m_needNewTrackedPoints = false;

    m_initOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

CameraParameters PipelineNaturalImageMarker::getCameraParameters()
{
    CameraParameters camParam;
    if (m_camera)
    {
        camParam = m_camera->getParameters();
    }
    return camParam;
}


// get images from camera

void PipelineNaturalImageMarker::getCameraImages(){

    SRef<Image> view;
    if (m_stopFlag || !m_initOK || !m_startedOK)
        return;

    if(m_haveToBeFlip)
    {
        if(m_source->getNextImage(view)!=SolAR::SourceReturnCode::_NEW_IMAGE){
            m_stopFlag = true;
            return;
        }
        m_imageConvertorUnity->convert(view,view,Image::ImageLayout::LAYOUT_RGB);
    }
    else if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
        m_stopFlag = true;
        return;
    }

    if(m_CameraImagesForDetection.empty())
         m_CameraImagesForDetection.push(view);

    if(m_CameraImagesForTracking.empty())
         m_CameraImagesForTracking.push(view);

    return;
};

bool PipelineNaturalImageMarker::processDetection()
{
    if (m_stopFlag || !m_initOK || !m_startedOK)
        return false;

    bool valid_pose = false;

    SRef<Image> camImage;
    std::vector<SRef<Point2Df>> imagePoints_inliers;
    std::vector<SRef<Point3Df>> worldPoints_inliers;
    Transform3Df pose;
    std::vector< SRef<Keypoint> >  camKeypoints;  // where to store detected keypoints in ref image and camera image
    std::vector<DescriptorMatch>  matches;
    SRef<DescriptorBuffer>  camDescriptors;

    if (!m_CameraImagesForDetection.tryPop(camImage))
        return false ;

    // detect keypoints in camera image
    m_kpDetector->detect(camImage, camKeypoints);

    /* extract descriptors in camera image*/
    m_descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);

    /*compute matches between reference image and camera image*/
    m_matcher->match(m_refDescriptors, camDescriptors, matches);

    /* filter matches to remove redundancy and check geometric validity */
    m_basicMatchesFilter->filter(matches, matches, m_refKeypoints, camKeypoints);
    m_geomMatchesFilter->filter(matches, matches, m_refKeypoints, camKeypoints);

    std::vector <SRef<Point2Df>> ref2Dpoints;
    std::vector <SRef<Point2Df>> cam2Dpoints;
    std::vector <SRef<Point3Df>> ref3Dpoints;

    std::vector <SRef<Point2Df>> markerCornersinCamImage;
    std::vector <SRef<Point3Df>> markerCornersinWorld;


    std::vector<SRef<Point2Df>> refMatched2Dpoints, camMatched2Dpoints;

    if (matches.size()> m_detectionMatchesNumberThreshold) {
        // reindex the keypoints with established correspondence after the matching
        m_keypointsReindexer->reindex(m_refKeypoints, camKeypoints, matches, ref2Dpoints, cam2Dpoints);

        // mapping to 3D points
        m_img_mapper->map(ref2Dpoints, ref3Dpoints);

        // Estimate the pose from the 2D-3D planar correspondence
        if (m_poseEstimationPlanar->estimate(cam2Dpoints, ref3Dpoints, imagePoints_inliers,worldPoints_inliers, pose) != FrameworkReturnCode::_SUCCESS)
        {
            valid_pose = false;
            LOG_DEBUG("No pose from Detection for this frame");
        }
        else
        {
#ifdef TRACKING
            m_isTrack = true;
            LOG_INFO("Start tracking");
#else
            LOG_INFO("Valid pose");
#endif
            valid_pose = true;
        }
     }
#ifdef TRACKING
    if(valid_pose){
        m_outBufferDetection.push(std::make_tuple(camImage, pose, valid_pose));
    }
#else
    if(valid_pose){
        m_sink->set(pose, camImage);
    }
    else
        m_sink->set(camImage);
#endif
    return true;

}

bool PipelineNaturalImageMarker::processTracking()
{
    if (m_stopFlag || !m_initOK || !m_startedOK)
        return false;

    bool valid_pose = false;

    m_needNewTrackedPoints=false;
    SRef<Image> camImage;

    if (!m_CameraImagesForTracking.tryPop(camImage))
        return false ;

    std::tuple< SRef<Image>, Transform3Df, bool> getCameraPoseDetection;

    if (!m_isTrack) {
#ifdef TRACKING
        m_sink->set(camImage);
#endif
        return true;
    }

    if (m_outBufferDetection.tryPop(getCameraPoseDetection)) {
        SRef<Image> detectedImage;
        Transform3Df detectedPose;
        bool isDetectedPose;
        std::tie(detectedImage, detectedPose, isDetectedPose) = getCameraPoseDetection;
        if (isDetectedPose) {
            m_previousCamImage = detectedImage->copy();
            m_pose = detectedPose;
            m_needNewTrackedPoints = true;
        }
    }


    if (m_needNewTrackedPoints) {
        m_imagePoints_track.clear();
        m_worldPoints_track.clear();
        std::vector<SRef<Point2Df>> projectedMarkerCorners;
        std::vector<SRef<Keypoint>> newKeypoints;
        // Get the projection of the corner of the marker in the current image
        m_projection->project(m_markerWorldCorners, projectedMarkerCorners, m_pose);

        // Detect the keypoints within the contours of the marker defined by the projected corners
        m_kpDetectorRegion->detect(m_previousCamImage, projectedMarkerCorners, newKeypoints);

        if (newKeypoints.size() > m_updateTrackedPointThreshold) {
            for (auto keypoint : newKeypoints)
                m_imagePoints_track.push_back(xpcf::utils::make_shared<Point2Df>(keypoint->getX(), keypoint->getY()));

            // get back the 3D positions of the detected keypoints in world space
            m_unprojection->unproject(m_imagePoints_track, m_worldPoints_track, m_pose);
            LOG_DEBUG("Reinitialize points to track");
        }
        else {
            m_isTrack = false;
            LOG_DEBUG("Cannot reinitialize points to track");
        }
        m_needNewTrackedPoints = false;
    }

    if (!m_isTrack) {
        LOG_INFO("Tracking lost");
        m_sink->set(camImage);
        return true;
    }

    std::vector<SRef<Point2Df>> trackedPoints, pts2D;
    std::vector<SRef<Point3Df>> pts3D;
    std::vector<unsigned char> status;
    std::vector<float> err;

    // tracking 2D-2D
    m_opticalFlowEstimator->estimate(m_previousCamImage, camImage, m_imagePoints_track, trackedPoints, status, err);

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            pts2D.push_back(trackedPoints[i]);
            pts3D.push_back(m_worldPoints_track[i]);
        }
    }

    // Estimate the pose from the 2D-3D planar correspondence
    if (m_poseEstimationPlanar->estimate(pts2D, pts3D, m_imagePoints_track, m_worldPoints_track, m_pose) != FrameworkReturnCode::_SUCCESS)
    {
        m_isTrack = false;
        valid_pose = false;
        m_needNewTrackedPoints = false;
        LOG_INFO("Tracking lost");
    }
    else
    {
        valid_pose = true;
        m_previousCamImage = camImage->copy();
        if (m_worldPoints_track.size() < m_updateTrackedPointThreshold) {
            m_needNewTrackedPoints = true;
            LOG_DEBUG("Need new point to track");
        }
    }

    if(valid_pose){
        m_sink->set(m_pose, camImage);
//        std::cout << m_pose.matrix() <<"\n";
    }
    else
        m_sink->set(camImage);

    return true;
}
//////////////////////////////// ADD
SourceReturnCode PipelineNaturalImageMarker::loadSourceImage(void* sourceTextureHandle, int width, int height)
{
   m_haveToBeFlip = true;
   return m_source->setInputTexture((unsigned char *)sourceTextureHandle, width, height);
}
////////////////////////////////////


FrameworkReturnCode PipelineNaturalImageMarker::start(void* imageDataBuffer)
{
    if (m_initOK==false)
    {
        LOG_WARNING("Try to start the Fiducial marker pipeline without initializing it");
        return FrameworkReturnCode::_ERROR_;
    }
    m_stopFlag=false;
    m_sink->setImageBuffer((unsigned char*)imageDataBuffer);
    if (!m_haveToBeFlip && (m_camera->start() != FrameworkReturnCode::_SUCCESS))
    {
        LOG_ERROR("Camera cannot start");
        return FrameworkReturnCode::_ERROR_;
    }

    // create and start threads to process the images

    auto getCameraImagesThread = [this](){;getCameraImages();};
    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskGetCameraImages->start();

    auto processDetectionThread = [this](){;processDetection();};
    m_taskDetection = new xpcf::DelegateTask(processDetectionThread);
    m_taskDetection->start();

#ifdef TRACKING
    // create and start a thread to process the images
    auto processTrackingThread = [this](){;processTracking();};
    m_taskTracking = new xpcf::DelegateTask(processTrackingThread);
    m_taskTracking->start();
#endif


    m_startedOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineNaturalImageMarker::stop()
{
    m_stopFlag=true;
    if( !m_haveToBeFlip && m_taskGetCameraImages != nullptr )
            m_taskGetCameraImages->stop();


    if (m_taskDetection != nullptr)
        m_taskDetection->stop();
#ifdef TRACKING
    if (m_taskTracking != nullptr)
        m_taskTracking->stop();
#endif
    if(!m_initOK)
    {
        LOG_WARNING("Try to stop a pipeline that has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_startedOK)
    {
        LOG_WARNING("Try to stop a pipeline that has not been started");
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_INFO("Pipeline has stopped: \n");

    return FrameworkReturnCode::_SUCCESS;
}

SinkReturnCode PipelineNaturalImageMarker::update(Transform3Df& pose)
{
    return m_sink->tryGet(pose);
}

}
}
