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
#include "SolARModuleOpengl_traits.h"

#include "core/Log.h"

int updateTrackedPointThreshold = 300;

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
   m_initOK = false;
   m_startedOK = false;
   m_stopFlag = false;
   m_haveToBeFlip = false;
   m_taskProcess = nullptr;
   LOG_DEBUG(" Pipeline constructor");
}

PipelineNaturalImageMarker::~PipelineNaturalImageMarker()
{
    if(m_taskProcess != nullptr)
        delete m_taskProcess;
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
        m_transform2D = xpcfComponentManager->create<MODULES::TOOLS::SolAR2DTransform>()->bindTo<geom::I2DTransform>();
        if (m_transform2D)
            LOG_INFO("Transform 2D component loaded");
        m_homographyValidation = xpcfComponentManager->create<MODULES::TOOLS::SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
        if (m_homographyValidation)
            LOG_INFO("Homography validation component loaded");
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
        m_projection = xpcfComponentManager->create<MODULES::OPENCV::SolARProjectOpencv>()->bindTo<geom::IProject>();
        m_unprojection = xpcfComponentManager->create<MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv>()->bindTo<geom::IUnproject>();
        m_poseEstimationPlanar = xpcfComponentManager->create<MODULES::OPENCV::SolARPoseEstimationPlanarPointsOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
        m_opticalFlowEstimator = xpcfComponentManager->create<MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv>()->bindTo<tracking::IOpticalFlowEstimator>();


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

    int refImageWidth,refImageheight;
    float worldWidth,worldHeight;
    refImageWidth=m_refImage->getSize().width;
    refImageheight=m_refImage->getSize().height;

    LOG_INFO(" worldWidth : {} worldHeight : {} \n",worldWidth,worldHeight)

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

    m_valid_pose = false;
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
        Sizei resolution = m_camera->getResolution();
        CamCalibration calib = m_camera->getIntrinsicsParameters();
        camParam.width = resolution.width;
        camParam.height = resolution.height;
        camParam.focalX = calib(0,0);
        camParam.focalY = calib(1,1);
    }
    return camParam;
}

bool PipelineNaturalImageMarker::processCamImage()
{
    if (m_stopFlag || !m_initOK || !m_startedOK)
        return false;

    bool poseComputed = false;

    if(m_haveToBeFlip)
    {
        m_source->getNextImage(m_camImage);
        m_imageConvertorUnity->convert(m_camImage,m_camImage,Image::ImageLayout::LAYOUT_RGB);
    }
    else if (m_camera->getNextImage(m_camImage) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE)
    {
        LOG_WARNING("The camera cannot load any image");
        m_stopFlag = true;
        return false;
    }

    m_valid_pose = false;


    if(!m_isTrack){

        // detect keypoints in camera image
        m_kpDetector->detect(m_camImage, m_camKeypoints);

        /* extract descriptors in camera image*/
        m_descriptorExtractor->extract(m_camImage, m_camKeypoints, m_camDescriptors);

        /*compute matches between reference image and camera image*/
        m_matcher->match(m_refDescriptors, m_camDescriptors, m_matches);

        /* filter matches to remove redundancy and check geometric validity */
        m_basicMatchesFilter->filter(m_matches, m_matches, m_refKeypoints, m_camKeypoints);
        m_geomMatchesFilter->filter(m_matches, m_matches, m_refKeypoints, m_camKeypoints);

        std::vector <SRef<Point2Df>> ref2Dpoints;
        std::vector <SRef<Point2Df>> cam2Dpoints;
        std::vector <SRef<Point3Df>> ref3Dpoints;
        Transform2Df Hm;
        std::vector <SRef<Point2Df>> markerCornersinCamImage;
        std::vector <SRef<Point3Df>> markerCornersinWorld;


        std::vector<SRef<Point2Df>> refMatched2Dpoints, camMatched2Dpoints;



        if (m_matches.size()> 10) {
            // reindex the keypoints with established correspondence after the matching
            m_keypointsReindexer->reindex(m_refKeypoints, m_camKeypoints, m_matches, ref2Dpoints, cam2Dpoints);

            // mapping to 3D points
            m_img_mapper->map(ref2Dpoints, ref3Dpoints);

            // Estimate the pose from the 2D-3D planar correspondence
            if (m_poseEstimationPlanar->estimate(cam2Dpoints, ref3Dpoints, m_imagePoints_inliers,m_worldPoints_inliers, m_pose) != FrameworkReturnCode::_SUCCESS)
            {
                m_valid_pose = false;
                LOG_INFO("Wrong homography for this frame");
            }
            else
            {
#ifdef TRACKING
                m_isTrack = true;
                m_needNewTrackedPoints = true;
#endif
                m_valid_pose = true;
                poseComputed = true;
                m_previousCamImage= m_camImage->copy();
                LOG_INFO("Start tracking", m_pose.matrix());
            }
         }
    }
    else // We track planar keypoints and we estimate the pose based on a homography
    {
         std::vector<SRef<Point2Df>> trackedPoints, pts2D;
         std::vector<SRef<Point3Df>> pts3D;
         std::vector<unsigned char> status;
         std::vector<float> err;

         // tracking 2D-2D
         m_opticalFlowEstimator->estimate(m_previousCamImage, m_camImage, m_imagePoints_inliers, trackedPoints, status, err);

         for (int i = 0; i < status.size(); i++)
         {
             if (status[i])
             {
                 pts2D.push_back(trackedPoints[i]);
                 pts3D.push_back(m_worldPoints_inliers[i]);
             }
         }

         // calculate camera pose
         // Estimate the pose from the 2D-3D planar correspondence
         if (m_poseEstimationPlanar->estimate(pts2D, pts3D, m_imagePoints_inliers, m_worldPoints_inliers, m_pose) != FrameworkReturnCode::_SUCCESS)
         {
             m_isTrack = false;
             m_valid_pose = false;
             m_needNewTrackedPoints = false;
             LOG_INFO("Tracking lost");
         }
         else
         {
             m_valid_pose = true;
             m_previousCamImage = m_camImage->copy();
             if (m_worldPoints_inliers.size() < updateTrackedPointThreshold)
                 m_needNewTrackedPoints = true;
         }
     }
#ifdef TRACKING
    if (m_needNewTrackedPoints)
    {
        m_imagePoints_inliers.clear();
        m_worldPoints_inliers.clear();
        std::vector<SRef<Keypoint>> newKeypoints;
        // Get the projection of the corner of the marker in the current image
        m_projection->project(m_markerWorldCorners, m_projectedMarkerCorners, m_pose);
        // Detect the keypoints within the contours of the marker defined by the projected corners
        m_kpDetectorRegion->detect(m_camImage, m_projectedMarkerCorners, newKeypoints);

        for (auto keypoint : newKeypoints)
            m_imagePoints_inliers.push_back(xpcf::utils::make_shared<Point2Df>(keypoint->getX(), keypoint->getY()));

        // get back the 3D positions of the detected keypoints in world space
        m_unprojection->unproject(m_imagePoints_inliers, m_worldPoints_inliers, m_pose);
        m_needNewTrackedPoints = false;
        LOG_INFO("Reinitialize points to track");
    }
#endif

    if(m_valid_pose){
        m_sink->set(m_pose, m_camImage);
        std::cout << m_pose.matrix() <<"\n";
    }
    else
        m_sink->set(m_camImage);

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
    // create and start a thread to process the images
    auto processCamImageThread = [this](){;processCamImage();};

    m_taskProcess = new xpcf::DelegateTask(processCamImageThread);
    m_taskProcess->start();
    //LOG_DEBUG("Fiducial marker pipeline has started");

    m_startedOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineNaturalImageMarker::stop()
{
    m_stopFlag=true;
    if( !m_haveToBeFlip)
        m_camera->stop();

    if (m_taskProcess != nullptr)
        m_taskProcess->stop();

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
