/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include "xpcf/module/ModuleFactory.h"
#include "PipelineNaturalImageMarker.h"

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"

#include "core/Log.h"

#define TRACKING

namespace xpcf=org::bcom::xpcf;

// Declaration of the module embedding the fiducial marker pipeline
XPCF_DECLARE_MODULE("5bbc1452-8d7f-4512-83f1-7bf48453809f", "FiducialMarkerModule", "The module embedding a pipeline to estimate the pose based on a squared fiducial marker")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::PipelineNaturalImageMarker>(componentUUID,interfaceRef);

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::PipelineNaturalImageMarker)
XPCF_END_COMPONENTS_DECLARATION

// The pipeline component for the fiducial marker

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::PipelineNaturalImageMarker)

namespace SolAR {
using namespace datastructure;
using namespace api::pipeline;
namespace PIPELINES {

PipelineNaturalImageMarker::PipelineNaturalImageMarker():ConfigurableBase(xpcf::toUUID<PipelineNaturalImageMarker>())
{
   addInterface<api::pipeline::IPipeline>(this);
   SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
   params->wrapInteger("updateTrackedPointThreshold", m_updateTrackedPointThreshold);
   params->wrapInteger("detectionMatchesNumberThreshold", m_detectionMatchesNumberThreshold);

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
    LOG_DEBUG("Start init")
    try {
        m_camera = xpcfComponentManager->create<MODULES::OPENCV::SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
        if (m_camera)
            LOG_INFO("Camera component loaded");
        m_kpDetector = xpcfComponentManager->create<MODULES::OPENCV::SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
        if (m_kpDetector)
            LOG_INFO("Keypoint detector component loaded");
        m_naturalImagemarker = xpcfComponentManager->create<MODULES::OPENCV::SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
        if (m_naturalImagemarker)
            LOG_INFO("Natural Image Marker component loaded");
        m_geomMatchesFilter = xpcfComponentManager->create<MODULES::OPENCV::SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
        if (m_geomMatchesFilter)
            LOG_INFO("geometric matcher component loaded");
        m_matcher = xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
        if (m_matcher)
            LOG_INFO("descriptor matcher component loaded");
        m_descriptorExtractor =  xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
        if (m_descriptorExtractor)
            LOG_INFO("Descriptor extractor component loaded");
        m_basicMatchesFilter = xpcfComponentManager->create<MODULES::TOOLS::SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
        if (m_basicMatchesFilter)
            LOG_INFO("Basic matcher component loaded");
        m_img_mapper = xpcfComponentManager->create<MODULES::TOOLS::SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
        if (m_img_mapper)
            LOG_INFO("Image Mapper component loaded");
        m_keypointsReindexer = xpcfComponentManager->create<MODULES::TOOLS::SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
        if (m_keypointsReindexer)
            LOG_INFO("Keypoint Reindexer component loaded");
        m_sink = xpcfComponentManager->create<MODULES::TOOLS::SolARBasicSink>()->bindTo<sink::ISinkPoseImage>();
        if (m_sink)
            LOG_INFO("Pose Texture Buffer Sink component loaded");
        m_source = xpcfComponentManager->create<MODULES::TOOLS::SolARBasicSource>()->bindTo<source::ISourceImage>();
        if (m_source)
            LOG_INFO("Source image component loaded");
        m_imageConvertorUnity =xpcfComponentManager->create<MODULES::OPENCV::SolARImageConvertorUnity>()->bindTo<image::IImageConvertor>();
        if (m_imageConvertorUnity)
            LOG_INFO("Image Convertor Unity component loaded");
        m_kpDetectorRegion = xpcfComponentManager->create<MODULES::OPENCV::SolARKeypointDetectorRegionOpencv>()->bindTo<features::IKeypointDetectorRegion>();
        if (m_kpDetectorRegion)
            LOG_INFO(" keypoint detection area component loaded");
        m_projection = xpcfComponentManager->create<MODULES::OPENCV::SolARProjectOpencv>()->bindTo<geom::IProject>();
        if (m_projection)
            LOG_INFO("projection component loaded");
        m_unprojection = xpcfComponentManager->create<MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv>()->bindTo<geom::IUnproject>();
        if (m_unprojection)
            LOG_INFO("inverse projection component loaded");
        m_poseEstimationPlanar = xpcfComponentManager->create<MODULES::OPENCV::SolARPoseEstimationPlanarPointsOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
        if (m_poseEstimationPlanar)
            LOG_INFO("Palnar pose estimation component loaded");
        m_opticalFlowEstimator = xpcfComponentManager->create<MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv>()->bindTo<tracking::IOpticalFlowEstimator>();
        if (m_opticalFlowEstimator)
            LOG_INFO("Optical flow component loaded");


    }
    catch (xpcf::Exception e)
    {
       LOG_WARNING("One or more components cannot be created: {}", e.what());
       return FrameworkReturnCode::_ERROR_;
    }
    LOG_INFO("All components have been created");

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

CamCalibration PipelineNaturalImageMarker::getCameraParameters()
{
    CamCalibration camParam;
    if (m_camera)
    {
        //Sizei resolution = m_camera->getResolution();
        //CamCalibration calib = m_camera->getIntrinsicsParameters();
        camParam = m_camera->getIntrinsicsParameters();
        //camParam.width = resolution.width;
        //camParam.height = resolution.height;
        //camParam.focalX = calib(0,0);
        //camParam.focalY = calib(1,1);
        //camParam.centerX = calib(0,2);
        //camParam.centerY = calib(1,2);
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
    //LOG_DEBUG("Fiducial marker pipeline has started");

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
