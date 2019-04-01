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

#include <iostream>
#include <vector>
#include <utility>
#include <tuple>
#include <numeric>
#include <string>
#include <functional>
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include <boost/log/core.hpp>

#include "core/Log.h"

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"

// ADD COMPONENT INTERFACES HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IKeypointDetectorRegion.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/IProject.h"
#include "api/geom/IUnproject.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/tracking/IOpticalFlowEstimator.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"


using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf = org::bcom::xpcf;
using namespace std;

//#define TRACKING

int updateTrackedPointThreshold = 500;

int main(int argc, char *argv[])
{

#ifdef NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    //    SolARLog::init();
    LOG_ADD_LOG_TO_CONSOLE();
    LOG_INFO("program is running");

    try {
        /* instantiate component manager*/
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load("conf_NaturalImageMarker.xml") != org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file conf_NaturalImageMarker.xml", argv[1])
            return -1;
        }
        // declare and create components
        LOG_INFO("Start creating components");

        auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
        auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
        auto kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
        auto kpDetectorRegion = xpcfComponentManager->create<SolARKeypointDetectorRegionOpencv>()->bindTo<features::IKeypointDetectorRegion>();
        auto descriptorExtractor = xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
        auto matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
        auto basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
        auto geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
        auto keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
        auto poseEstimationPlanar = xpcfComponentManager->create<SolARPoseEstimationPlanarPointsOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
        auto img_mapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
        auto opticalFlowEstimator = xpcfComponentManager->create<SolAROpticalFlowPyrLKOpencv>()->bindTo<tracking::IOpticalFlowEstimator>();
        auto projection = xpcfComponentManager->create<SolARProjectOpencv>()->bindTo<geom::IProject>();
        auto unprojection = xpcfComponentManager->create<SolARUnprojectPlanarPointsOpencv>()->bindTo<geom::IUnproject>();
        auto overlay2DComponent = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
        auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
        auto imageViewerKeypoints = xpcfComponentManager->create<SolARImageViewerOpencv>("keypoints")->bindTo<display::IImageViewer>();
        auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();

        // Declare data structures used to exchange information between components for initialization
        SRef<Image> refImage;
        std::vector<SRef<Keypoint>> refKeypoints;
        SRef<DescriptorBuffer> refDescriptors;
        std::vector<SRef<Point3Df>> markerWorldCorners;

        // Declare Buffers used to exchange data between tasks
        xpcf::DropBuffer<SRef<Image>>   m_dropBufferCamImageForDetection;
        xpcf::DropBuffer<SRef<Image>>   m_dropBufferCamImageForTracking;
        xpcf::DropBuffer<std::pair<SRef<Image>, Transform3Df>>  m_dropBufferPoseForTracking;
        xpcf::DropBuffer<std::tuple<SRef<Image>, Transform3Df, bool>>  m_dropBufferPoseForDisplay;

        bool stop = false;

        // load marker
        LOG_INFO("LOAD MARKER IMAGE ");
        marker->loadMarker();
        marker->getWorldCorners(markerWorldCorners);

        marker->getImage(refImage);

        // detect keypoints in reference image
        kpDetector->detect(refImage, refKeypoints);

        // extract descriptors in reference image
        descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);

#ifndef NDEBUG
        // display keypoints in reference image
        // copy reference image
        SRef<Image> kpImage = refImage->copy();
        // draw circles on keypoints

        overlay2DComponent->drawCircles(refKeypoints, kpImage);
        // displays the image with circles in an imageviewer
        imageViewerKeypoints->display(kpImage);
#endif

        if (camera->start() != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("Camera cannot start");
            return -1;
        }

        // initialize overlay 3D component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
        overlay3DComponent->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

        // initialize pose estimation based on planar points with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
        poseEstimationPlanar->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

        // initialize projection component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
        projection->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

        // initialize unprojection component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
        unprojection->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

        // initialize image mapper with the reference image size and marker size
        img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(refImage->getSize().width);
        img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(refImage->getSize().height);
        img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(marker->getSize().width);
        img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(marker->getSize().height);

        // to count the average number of processed frames per seconds
        clock_t start, end;
        int count = 0;
        start = clock();

        // Get images from camera and fill drop buffers for Detection and Tracking tasks
        std::function<void(void)> camImageCapture = [&stop,camera,&m_dropBufferCamImageForDetection, &m_dropBufferCamImageForTracking](){

            SRef<Image> camImage;
            if (camera->getNextImage(camImage) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
                stop = true;
                return;
            }
            m_dropBufferCamImageForDetection.push(camImage);
            m_dropBufferCamImageForTracking.push(camImage);
        };

        // Marker Detection
        std::function<void(void)> detection = [&refKeypoints, refDescriptors,
                                               &m_dropBufferCamImageForDetection, &m_dropBufferPoseForTracking,
                                               kpDetector, descriptorExtractor, matcher, basicMatchesFilter, geomMatchesFilter, keypointsReindexer, img_mapper, poseEstimationPlanar](){
            SRef<Image> camImage;
            std::vector<SRef<Keypoint>> camKeypoints;
            SRef<DescriptorBuffer> camDescriptors;
            std::vector<DescriptorMatch> matches;
            std::vector<SRef<Point2Df>> refMatched2Dpoints, camMatched2Dpoints;
            std::vector<SRef<Point3Df>> ref3Dpoints;
            Transform3Df pose;
            std::vector<SRef<Point2Df>> imagePoints_inliers;
            std::vector<SRef<Point3Df>> worldPoints_inliers;

            if (!m_dropBufferCamImageForDetection.tryPop(camImage))
                return;

            // detect keypoints in camera image
            kpDetector->detect(camImage, camKeypoints);

            /* extract descriptors in camera image*/
            descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);

            /*compute matches between reference image and camera image*/
            matcher->match(refDescriptors, camDescriptors, matches);

            /* filter matches to remove redundancy and check geometric validity */
            basicMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
            geomMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);

            /*we consider that, if we have less than 10 matches (arbitrarily), we can't compute homography for the current frame */
            if (matches.size() > 10)
            {
                // reindex the keypoints with established correspondence after the matching
                keypointsReindexer->reindex(refKeypoints, camKeypoints, matches, refMatched2Dpoints, camMatched2Dpoints);

                // mapping to 3D points
                img_mapper->map(refMatched2Dpoints, ref3Dpoints);

                if (poseEstimationPlanar->estimate(camMatched2Dpoints, ref3Dpoints, imagePoints_inliers, worldPoints_inliers, pose) != FrameworkReturnCode::_SUCCESS)
                {
                    m_dropBufferPoseForTracking.push(std::make_pair(camImage, pose));
                    return;
                }
            }
            return;
        };

        // Marker Tracking

        // Variable shared for tracking
        SRef<Image> previousCamImage;
        Transform3Df previousPose;
        std::vector<SRef<Point2Df>> imagePoints_inliers;
        std::vector<SRef<Point3Df>> worldPoints_inliers;

        std::function<void(void)> tracking = [&markerWorldCorners, &previousCamImage, &previousPose, &imagePoints_inliers, &worldPoints_inliers,
                                              &m_dropBufferCamImageForTracking, &m_dropBufferPoseForTracking, &m_dropBufferPoseForDisplay,
                                              projection, kpDetectorRegion, unprojection, opticalFlowEstimator, poseEstimationPlanar]()
        {
            SRef<Image> camImage;
            std::pair<SRef<Image>, Transform3Df> poseImageFromDetection;
            std::vector<SRef<Keypoint>> newKeypoints;
            std::vector<SRef<Point2Df>> projectedMarkerCorners, trackedPoints, pts2D;
            std::vector<SRef<Point3Df>> pts3D;
            std::vector<unsigned char> status;
            std::vector<float> err;
            Transform3Df pose;
            bool detectionPerformed = false;
            bool needNewTrackedPoints = false;

            if (!m_dropBufferCamImageForTracking.tryPop(camImage))
                return;

            // The detection task has just estimated a pose
            if (m_dropBufferPoseForTracking.tryPop(poseImageFromDetection))
            {
                previousCamImage = poseImageFromDetection.first;
                previousPose = poseImageFromDetection.second;
                detectionPerformed = true;
            }
            // if tracking and detection have failed, just pass the current image to the display task
            if (imagePoints_inliers.empty() && !detectionPerformed)
            {
                m_dropBufferPoseForDisplay.push(std::make_tuple(camImage, Transform3Df::Identity(), false));
                return;
            }
            if (imagePoints_inliers.size() < updateTrackedPointThreshold)
                needNewTrackedPoints = true;

            // Add new keypoints to track
            if (needNewTrackedPoints)
            {
                imagePoints_inliers.clear();
                worldPoints_inliers.clear();
                std::vector<SRef<Keypoint>> newKeypoints;
                // Get the projection of the corner of the marker in the current image
                projection->project(markerWorldCorners, projectedMarkerCorners, previousPose);
/*
#ifndef NDEBUG
                overlay2DComponent->drawContour(projectedMarkerCorners, kpImageCam);
#endif
*/
                // Detect the keypoints within the contours of the marker defined by the projected corners
                kpDetectorRegion->detect(camImage, projectedMarkerCorners, newKeypoints);

                for (auto keypoint : newKeypoints)
                    imagePoints_inliers.push_back(xpcf::utils::make_shared<Point2Df>(keypoint->getX(), keypoint->getY()));

                // Get back the 3D positions of the detected keypoints in world space
                unprojection->unproject(imagePoints_inliers, worldPoints_inliers, previousPose);
            }

            // tracking 2D-2D
            opticalFlowEstimator->estimate(previousCamImage, camImage, imagePoints_inliers, trackedPoints, status, err);

            for (int i = 0; i < status.size(); i++)
            {
                if (status[i])
                {
                    pts2D.push_back(trackedPoints[i]);
                    pts3D.push_back(worldPoints_inliers[i]);
                }
            }

            // Estimate the pose from the 2D-3D planar correspondence
            if (poseEstimationPlanar->estimate(pts2D, pts3D, imagePoints_inliers, worldPoints_inliers, pose) != FrameworkReturnCode::_SUCCESS)
            {
                m_dropBufferPoseForDisplay.push(std::make_tuple(camImage, Transform3Df::Identity(), false));
                previousPose = pose;
                previousCamImage = camImage;
            }
            else
            {
                m_dropBufferPoseForDisplay.push(std::make_tuple(camImage, Transform3Df::Identity(), false));
                imagePoints_inliers.clear();
            }
            return;
        };


        std::function<void(void)> display = [&stop, &count,
                                             &m_dropBufferPoseForDisplay,
                                             overlay3DComponent, imageViewerResult](){

            std::tuple<SRef<Image>, Transform3Df, bool> resultFromTracking;

            if (!m_dropBufferPoseForDisplay.tryPop(resultFromTracking))
                return;

            // If the pose is valid
            if (std::get<2>(resultFromTracking))
                overlay3DComponent->draw(std::get<1>(resultFromTracking), std::get<0>(resultFromTracking));

            if (imageViewerResult->display(std::get<0>(resultFromTracking)) == FrameworkReturnCode::_STOP){
                    stop=true;
             };
            count++;
            return;
        };

        // instantiate and start tasks
        xpcf::DelegateTask taskCamImageCapture(camImageCapture);
        xpcf::DelegateTask taskDetection(detection);
        xpcf::DelegateTask taskTracking(tracking);
        xpcf::DelegateTask taskDisplay(display);

        taskCamImageCapture.start();
        taskDetection.start();
        taskTracking.start();
        taskDisplay.start();

        // The main loop
        while (!stop)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Measure time per frame
        end = clock();
        double duration = double(end - start) / CLOCKS_PER_SEC;
        printf("\n\nElasped time is %.2lf seconds.\n", duration);
        printf("Number of processed frame per second : %8.2f\n", count / duration);
        std::cout << "this is the end..." << '\n';

        // Stop tasks
        taskCamImageCapture.stop();
        taskDetection.stop();
        taskTracking.stop();
        taskDisplay.stop();

        std::cout << "end of processes \n";
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR("The following exception has been catch {}", e.what());
    }

    return 0;
}
