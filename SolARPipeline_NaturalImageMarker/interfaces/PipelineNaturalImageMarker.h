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
#ifdef SolARPipelineNaturalImageMarker_API_DLLEXPORT
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API __declspec(dllexport)
#else //SolARPipelineNaturalImageMarker_API_DLLEXPORT
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API __declspec(dllimport)
#endif //SolARPipelineNaturalImageMarker_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPoseEstimationPipeline.h"

// Add the headers to datastructures and component interfaces used by the pipeline
#include "api/input/devices/ICamera.h"

#include "api/input/files/ITrackableLoader.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"

#include "api/input/files/ITrackableLoader.h"
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
namespace PIPELINES {

/**
 * @class PipelineNaturalImageMarker
 * @brief A pipeline to estimate the pose based on a squared fiducial marker.
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::input::devices::ICamera}
 * @SolARComponentInjectable{SolAR::api::input::files::ITrackableLoader}
 * @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
 * @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
 * @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
 * @SolARComponentInjectable{SolAR::api::features::IKeypointsReIndexer}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
 * @SolARComponentInjectable{SolAR::api::geom::IImage2WorldMapper}
 * @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
 * @SolARComponentInjectable{SolAR::api::sink::ISinkPoseImage}
 * @SolARComponentInjectable{SolAR::api::source::ISourceImage}
 * @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
 * @SolARComponentInjectable{SolAR::api::features::IKeypointDetectorRegion}
 * @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformSACFinderFrom2D3D}
 * @SolARComponentInjectable{SolAR::api::tracking::IOpticalFlowEstimator}
 * @SolARComponentInjectable{SolAR::api::geom::IProject}
 * @SolARComponentInjectable{SolAR::api::geom::IUnproject}
 * @SolARComponentInjectablesEnd
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ updateTrackedPointThreshold,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 300 }}
 * @SolARComponentProperty{ detectionMatchesNumberThreshold,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 10 }}
 * @SolARComponentPropertiesEnd
 *
 */

class SOLARPIPELINENATURALIMAGEMARKER_EXPORT_API PipelineNaturalImageMarker : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IPoseEstimationPipeline
{
public:
    PipelineNaturalImageMarker();
    ~PipelineNaturalImageMarker();

    //// @brief Initialization of the pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode init() override;

    /// @brief Provide the camera parameters
    /// @return the camera parameters (its resolution and its focal)
    datastructure::CameraParameters getCameraParameters() const override;

    /// @brief Start the pipeline
    /// @return FrameworkReturnCode::_ERROR_ by default as the pipeline needs to be construct with an imageDataBuffer as parameter
    FrameworkReturnCode start() override { return FrameworkReturnCode::_ERROR_; }

    /// @brief Starts the pipeline and provides a texture buffer which will be updated when required.
    /// @param[in] textureHandle a pointer to the texture buffer which will be updated at each call of the update method.
    FrameworkReturnCode start(void* imageDataBuffer) override;

    /// @brief Stop the pipeline.
    FrameworkReturnCode stop() override;

    /// @brief update the pipeline
    /// Get the new pose and update the texture buffer with the image that has to be displayed
    api::sink::SinkReturnCode update(datastructure::Transform3Df& pose) override;

    /// @brief load the source image
    api::source::SourceReturnCode loadSourceImage(void* sourceTextureHandle, int width, int height) override;

    xpcf::XPCFErrorCode onConfigured() override;

    void unloadComponent () override final;

    void onInjected() override;

private:
    // Decalaration of data structures shared between initialization and process thread
    SRef<datastructure::DescriptorBuffer> m_markerPatternDescriptor;

    // Declaration of the components used by the pipeline
    SRef<api::input::devices::ICamera> m_camera;

    SRef<api::input::files::ITrackableLoader> m_trackableLoader;
    SRef<api::features::IKeypointDetector> m_kpDetector;
    SRef<api::features::IDescriptorMatcher> m_matcher;
    SRef<api::features::IMatchesFilter> m_basicMatchesFilter;
    SRef<api::features::IMatchesFilter> m_geomMatchesFilter;
    SRef<api::features::IKeypointsReIndexer> m_keypointsReindexer;
    SRef<api::features::IDescriptorsExtractor> m_descriptorExtractor;
    SRef<api::geom::IImage2WorldMapper> m_img_mapper;

    SRef<api::image::IImageConvertor> m_imageConvertor;
    SRef<api::sink::ISinkPoseImage> m_sink;
    SRef<api::source::ISourceImage> m_source;
    SRef<api::image::IImageConvertor> m_imageConvertorUnity;
    SRef<api::features::IKeypointDetectorRegion> m_kpDetectorRegion;
    SRef<api::solver::pose::I3DTransformSACFinderFrom2D3D> m_poseEstimationPlanar;
    SRef<api::tracking::IOpticalFlowEstimator> m_opticalFlowEstimator;
    SRef<api::geom::IProject> m_projection;
    SRef<api::geom::IUnproject> m_unprojection;


    // State flag of the pipeline
    bool m_stopFlag, m_initOK, m_startedOK,m_haveToBeFlip;

    // Threads
    xpcf::DelegateTask* m_taskGetCameraImages;
    void getCameraImages();

    xpcf::DropBuffer< SRef<datastructure::Image> >  m_CameraImagesForDetection;
    bool processDetection();
    xpcf::DelegateTask* m_taskDetection;

    std::vector<datastructure::Point2Df> m_imagePoints_inliers;
    std::vector<datastructure::Point3Df> m_worldPoints_inliers;

    xpcf::DropBuffer<std::tuple< SRef<datastructure::Image>, datastructure::Transform3Df, bool>>  m_outBufferDetection;
    xpcf::DropBuffer< SRef<datastructure::Image> >  m_CameraImagesForTracking;
    bool processTracking();
    xpcf::DelegateTask* m_taskTracking;



private :
    int m_updateTrackedPointThreshold = 300;
    int m_detectionMatchesNumberThreshold = 10;

    SRef<datastructure::DescriptorBuffer>  m_refDescriptors;

    datastructure::Transform2Df  m_Hm;
    std::vector<datastructure::Keypoint>  m_refKeypoints;  // where to store detected keypoints in ref image and camera image

    std::vector<datastructure::Point2Df> m_refImgCorners;

    SRef<datastructure::Image> m_refImage;
    SRef<datastructure::Image> camImage;
    SRef<datastructure::Image> m_previousCamImage;

    datastructure::Transform3Df m_pose;

    std::vector<datastructure::Point3Df> m_markerWorldCorners;
    std::vector<datastructure::Point2Df> m_projectedMarkerCorners;
    std::vector<datastructure::Point2Df> m_imagePoints_track;
    std::vector<datastructure::Point3Df> m_worldPoints_track;


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
