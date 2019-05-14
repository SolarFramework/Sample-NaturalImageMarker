/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PIPELINENATURALIMAGEMARKER_H
#define PIPELINENATURALIMAGEMARKER_H

#if _WIN32
#ifdef PipelineNaturalImageMarker_API_DLLEXPORT
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API __declspec(dllexport)
#else //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API __declspec(dllimport)
#endif //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPipeline.h"

// Add the headers to datastructures and component interfaces used by the pipeline
#include "api/input/devices/ICamera.h"

#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/ISBPatternReIndexer.h"

#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"
#include "api/sink/ISinkPoseImage.h"
#include "api/source/ISourceImage.h"
#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include "api/geom/IProject.h"
#include "api/geom/IUnproject.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/tracking/IOpticalFlowEstimator.h"

#include "api/features/IKeypointDetectorRegion.h"


namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::sink;
using namespace api::source;
using namespace api::pipeline;
namespace PIPELINES {

/**
 * @class FiducialMarker
 * @brief A pipeline to estimate the pose based on a squared fiducial marker.
 */

class SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API PipelineNaturalImageMarker : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IPipeline
{
public:
    PipelineNaturalImageMarker();
    ~PipelineNaturalImageMarker();

    //// @brief Initialization of the pipeline
    /// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
    /// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
    FrameworkReturnCode init(SRef<xpcf::IComponentManager> xpcfComponentManager) override;

    /// @brief Provide the camera parameters
    /// @return the camera parameters (its resolution and its focal)
    CamCalibration getCameraParameters() override;

    /// @brief Starts the pipeline and provides a texture buffer which will be updated when required.
    /// @param[in] textureHandle a pointer to the texture buffer which will be updated at each call of the update method.
    FrameworkReturnCode start(void* imageDataBuffer) override;

    /// @brief Stop the pipeline.
    FrameworkReturnCode stop() override;

    /// @brief update the pipeline
    /// Get the new pose and update the texture buffer with the image that has to be displayed
    SinkReturnCode update(Transform3Df& pose) override;

    /// @brief load the source image
    SourceReturnCode loadSourceImage(void* sourceTextureHandle, int width, int height) override;

    void unloadComponent () override final;

private:
    // Decalaration of data structures shared between initialization and process thread
    SRef<DescriptorBuffer> m_markerPatternDescriptor;

    // Declaration of the components used by the pipeline
    SRef<input::devices::ICamera> m_camera;
    SRef<input::files::IMarker2DSquaredBinary> m_binaryMarker;

    SRef<input::files::IMarker2DNaturalImage> m_naturalImagemarker;
    SRef<features::IKeypointDetector> m_kpDetector;
    SRef<features::IDescriptorMatcher> m_matcher;
    SRef<features::IMatchesFilter> m_basicMatchesFilter;
    SRef<features::IMatchesFilter> m_geomMatchesFilter;
    SRef<features::IKeypointsReIndexer> m_keypointsReindexer;
    SRef<features::IDescriptorsExtractor> m_descriptorExtractor;
    SRef<geom::IImage2WorldMapper> m_img_mapper;

    SRef<image::IImageConvertor> m_imageConvertor;
    SRef<sink::ISinkPoseImage> m_sink;
    SRef<source::ISourceImage> m_source;
    SRef<image::IImageConvertor> m_imageConvertorUnity;
    SRef<features::IKeypointDetectorRegion> m_kpDetectorRegion;
    SRef<solver::pose::I3DTransformSACFinderFrom2D3D> m_poseEstimationPlanar;
    SRef<tracking::IOpticalFlowEstimator> m_opticalFlowEstimator;
    SRef<geom::IProject> m_projection;
    SRef<geom::IUnproject> m_unprojection;


    // State flag of the pipeline
    bool m_stopFlag, m_initOK, m_startedOK,m_haveToBeFlip;

    // Threads
    xpcf::DelegateTask* m_taskGetCameraImages;
    void getCameraImages();

    xpcf::DropBuffer< SRef<Image> >  m_CameraImagesForDetection;
    bool processDetection();
    xpcf::DelegateTask* m_taskDetection;

    std::vector<SRef<Point2Df>> m_imagePoints_inliers;
    std::vector<SRef<Point3Df>> m_worldPoints_inliers;

    xpcf::DropBuffer<std::tuple< SRef<Image>, Transform3Df, bool>>  m_outBufferDetection;
    xpcf::DropBuffer< SRef<Image> >  m_CameraImagesForTracking;
    bool processTracking();
    xpcf::DelegateTask* m_taskTracking;



private :
    int m_updateTrackedPointThreshold = 300;
    int m_detectionMatchesNumberThreshold = 10;

    SRef<DescriptorBuffer>  m_refDescriptors;

    Transform2Df  m_Hm;
    std::vector< SRef<Keypoint> >  m_refKeypoints;  // where to store detected keypoints in ref image and camera image

    std::vector<SRef <Point2Df>> m_refImgCorners;

    SRef<Image> m_refImage;
    SRef<Image> camImage;
    SRef<Image> m_previousCamImage;

    Transform3Df m_pose;

    std::vector<SRef<Point3Df>> m_markerWorldCorners;
    std::vector<SRef<Point2Df>> m_projectedMarkerCorners;
    std::vector<SRef<Point2Df>> m_imagePoints_track;
    std::vector<SRef<Point3Df>> m_worldPoints_track;


    bool m_isTrack;
    bool m_needNewTrackedPoints;

};

}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::PipelineNaturalImageMarker,
                             "6f6c7ae8-764c-49b3-a2fc-e9a4f539b9b1",
                             "PipelineNaturalImageMarker",
                             "A pipeline to estimate the pose based on a natural Image marker");

#endif // PIPELINENATURALIMAGEMARKER_H
