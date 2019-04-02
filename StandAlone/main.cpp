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
#include <numeric>
#include <string>
#include <functional>

#include <boost/log/core.hpp>

using namespace std;
#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"

#include "xpcf/xpcf.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

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


#include "core/Log.h"
#include "SharedBuffer.hpp"

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf = org::bcom::xpcf;

#include <string>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#define TRACKING

int updateTrackedPointThreshold		= 300;
int cameraPoseDetectionThreshold	= 10;

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

		// information of reference image
		SRef<Image> refImage;
		std::vector<SRef<Keypoint>> refKeypoints;
		SRef<DescriptorBuffer> refDescriptors;
		std::vector<SRef<Point3Df>> markerWorldCorners;

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

		if (camera->start() != FrameworkReturnCode::_SUCCESS) // videoFile
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

		// global variables
		bool isTrack = false;
		xpcf::DropBuffer<std::tuple<SRef<Image>, Transform3Df, bool>> bufferImagePoseDisplay;


		// Thread to capture images
		xpcf::DropBuffer< SRef<Image> >  bufferCamImagesToDetect, bufferCamImagesToTrack;
		std::function<void(void)> getCameraImages = [&camera, &bufferCamImagesToDetect, &bufferCamImagesToTrack]() {
			SRef<Image> imageDetect, imageTrack;
			if (camera->getNextImage(imageDetect) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
				return;
			}
			bufferCamImagesToDetect.push(imageDetect);
			imageTrack = imageDetect->copy();
			bufferCamImagesToTrack.push(imageTrack);
		};

		// Thread to detect camera pose				
		xpcf::DropBuffer<std::tuple< SRef<Image>, Transform3Df, bool>> bufferCameraPoseDetection;
		std::function<void(void)> cameraPoseDetection = [&kpDetector, &descriptorExtractor, &matcher, &basicMatchesFilter, &geomMatchesFilter, &keypointsReindexer, &img_mapper, &poseEstimationPlanar, &refKeypoints, &refDescriptors, &bufferCamImagesToDetect, &bufferCameraPoseDetection, &bufferImagePoseDisplay, &isTrack, &overlay2DComponent]() {
			static int countToDetect = 0;			
			// get image to detect camera pose
			SRef<Image> camImage; 
			if (!bufferCamImagesToDetect.tryPop(camImage))
				return;
			countToDetect++;
			if (countToDetect % cameraPoseDetectionThreshold != 0)
				return;
			countToDetect = 0;

			std::vector<SRef<Keypoint>>		camKeypoints; // where to store detected keypoints in ref image and camera image
			SRef<DescriptorBuffer>			camDescriptors;
			std::vector<DescriptorMatch>	matches;
			std::vector<SRef<Point2Df>>		refMatched2Dpoints, camMatched2Dpoints;
			std::vector<SRef<Point3Df>>		ref3Dpoints;
			std::vector<SRef<Point2Df>>		imagePoints_inliers;
			std::vector<SRef<Point3Df>>		worldPoints_inliers;
			Transform3Df					pose = Transform3Df::Identity();
			bool							valid_pose = false;

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

				// Estimate the pose from the 2D-3D planar correspondence
				if (poseEstimationPlanar->estimate(camMatched2Dpoints, ref3Dpoints, imagePoints_inliers, worldPoints_inliers, pose) != FrameworkReturnCode::_SUCCESS)
				{
					valid_pose = false;
					LOG_DEBUG("Wrong homography for this frame");
				}
				else
				{
#ifdef TRACKING
					isTrack = true;
#endif
					valid_pose = true;
					//previousCamImage = camImage->copy();
					LOG_INFO("Start tracking", pose.matrix());
				}
			}
			if (valid_pose) {
				bufferCameraPoseDetection.push(std::make_tuple(camImage, pose, valid_pose));
			}

			SRef<Image> displayImage = camImage->copy();
#ifndef NDEBUG
			overlay2DComponent->drawCircles(imagePoints_inliers, displayImage);
#endif
#ifndef TRACKING
			bufferImagePoseDisplay.push(std::make_tuple(displayImage, pose, valid_pose));
#endif			
		};

		// Thread to track camera
		std::function<void(void)> cameraPoseTracking = [&bufferImagePoseDisplay, &bufferCamImagesToTrack, &isTrack, &bufferCameraPoseDetection, &projection, &markerWorldCorners, &kpDetectorRegion, &unprojection, &opticalFlowEstimator, &poseEstimationPlanar, &overlay2DComponent]() {
			static bool needNewTrackedPoints = false;
			static std::vector<SRef<Point2Df>> imagePoints_track;
			static std::vector<SRef<Point3Df>> worldPoints_track;
			static SRef<Image> previousCamImage;
			static Transform3Df pose;
			bool valid_pose = false;
			// get current image
			SRef<Image> camImage;
			if (!bufferCamImagesToTrack.tryPop(camImage))
				return;

			if (!isTrack) {
#ifdef TRACKING
				bufferImagePoseDisplay.push(std::make_tuple(camImage, pose, valid_pose));
#endif
				return;
			}			

			std::tuple< SRef<Image>, Transform3Df, bool> getCameraPoseDetection;			
			if (bufferCameraPoseDetection.tryPop(getCameraPoseDetection)) {		
				SRef<Image> detectedImage;
				Transform3Df detectedPose;
				bool isDetectedPose;
				std::tie(detectedImage, detectedPose, isDetectedPose) = getCameraPoseDetection;
				if (isDetectedPose) {
					previousCamImage = detectedImage->copy();
					pose = detectedPose;
					needNewTrackedPoints = true;
				}
			}

			if (needNewTrackedPoints) {
				imagePoints_track.clear();
				worldPoints_track.clear();
				std::vector<SRef<Point2Df>> projectedMarkerCorners;
				std::vector<SRef<Keypoint>> newKeypoints;
				// Get the projection of the corner of the marker in the current image
				projection->project(markerWorldCorners, projectedMarkerCorners, pose);

				// Detect the keypoints within the contours of the marker defined by the projected corners
				kpDetectorRegion->detect(previousCamImage, projectedMarkerCorners, newKeypoints);

				if (newKeypoints.size() > updateTrackedPointThreshold) {
					for (auto keypoint : newKeypoints)
						imagePoints_track.push_back(xpcf::utils::make_shared<Point2Df>(keypoint->getX(), keypoint->getY()));

					// get back the 3D positions of the detected keypoints in world space
					unprojection->unproject(imagePoints_track, worldPoints_track, pose);
					LOG_DEBUG("Reinitialize points to track");
				}
				else {
					isTrack = false;
					LOG_DEBUG("Cannot reinitialize points to track");
				}
				needNewTrackedPoints = false;
			}		

			if (!isTrack) {
				LOG_INFO("Tracking lost");
				return;
			}			

			std::vector<SRef<Point2Df>> trackedPoints, pts2D;
			std::vector<SRef<Point3Df>> pts3D;
			std::vector<unsigned char> status;
			std::vector<float> err;			

			// tracking 2D-2D
			opticalFlowEstimator->estimate(previousCamImage, camImage, imagePoints_track, trackedPoints, status, err);

			for (int i = 0; i < status.size(); i++)
			{
				if (status[i])
				{
					pts2D.push_back(trackedPoints[i]);
					pts3D.push_back(worldPoints_track[i]);
				}
			}

			// Estimate the pose from the 2D-3D planar correspondence
			if (poseEstimationPlanar->estimate(pts2D, pts3D, imagePoints_track, worldPoints_track, pose) != FrameworkReturnCode::_SUCCESS)
			{
				isTrack = false;
				valid_pose = false;
				needNewTrackedPoints = false;
				LOG_INFO("Tracking lost");
			}
			else
			{
				valid_pose = true;
				previousCamImage = camImage->copy();
				if (worldPoints_track.size() < updateTrackedPointThreshold) {
					needNewTrackedPoints = true;
					LOG_DEBUG("Need new point to track");
				}
			}

			SRef<Image> displayImage = camImage->copy();
#ifndef NDEBUG
			overlay2DComponent->drawCircles(imagePoints_track, displayImage);
#endif
#ifdef TRACKING
			bufferImagePoseDisplay.push(std::make_tuple(displayImage, pose, valid_pose));
#endif
		};


		xpcf::DelegateTask taskGetCameraImages(getCameraImages);
		xpcf::DelegateTask taskCameraPoseDetection(cameraPoseDetection);
		xpcf::DelegateTask taskCameraPoseTracking(cameraPoseTracking);

		taskGetCameraImages.start();
		taskCameraPoseDetection.start();
		taskCameraPoseTracking.start();

		// running loop process

		clock_t start, end;
		int count = 0;
		start = clock();

		while (true) {			
			std::tuple<SRef<Image>, Transform3Df, bool> imagePoseDisplay;
			if (bufferImagePoseDisplay.tryPop(imagePoseDisplay)) {
				SRef<Image> displayImage;
				Transform3Df pose;
				bool valid_pose;
				std::tie(displayImage, pose, valid_pose) = imagePoseDisplay;
				if (valid_pose)
					overlay3DComponent->draw(pose, displayImage);
				if (imageViewerResult->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
					break;
				count++;
			}			
		}

		taskGetCameraImages.stop();
		taskCameraPoseDetection.stop();
		taskCameraPoseTracking.stop();

		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);
		std::cout << "this is the end..." << '\n';
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch {}", e.what());
	}

	return 0;
}
