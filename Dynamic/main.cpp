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

#include <boost/log/core.hpp>

using namespace std;
#include "SolARModuleOpencv_traits.h"

#include "SolARModuleTools_traits.h"

#include "xpcf/xpcf.h"

#include "api/display/IImageViewer.h"
#include "api/input/devices/ICamera.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"

#include "SolAROpenCVHelper.h"

#include "SharedBuffer.hpp"

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::solver::pose;
namespace xpcf  = org::bcom::xpcf;

#include <string>

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

//    SolARLog::init();
    LOG_ADD_LOG_TO_CONSOLE();

    LOG_INFO("program is running");


    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_NaturalImageMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_NaturalImageMarker.xml", argv[1])
        return -1;
    }
    // declare and create components
    LOG_INFO("Start creating components");

    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageViewerKeypoints = xpcfComponentManager->create<SolARImageViewerOpencv>("keypoints")->bindTo<display::IImageViewer>();
    auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
    auto kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
    auto geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto homographyEstimation = xpcfComponentManager->create<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto homographyValidation = xpcfComponentManager->create<SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
    auto keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
    auto poseEstimation = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    //auto poseEstimation =xpcfComponentManager->create<SolARPoseEstimationPnpEPFL>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto overlay2DComponent = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
    auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
    auto img_mapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto transform2D = xpcfComponentManager->create<SolAR2DTransform>()->bindTo<geom::I2DTransform>();
    auto descriptorExtractor =  xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();

    /* in dynamic mode, we need to check that components are well created*/
    /* this is needed in dynamic mode */
    if (!camera || !imageViewerKeypoints || !imageViewerResult || !marker || !kpDetector || !descriptorExtractor || !matcher || !basicMatchesFilter || !geomMatchesFilter || !homographyEstimation ||
        !homographyValidation ||!keypointsReindexer || !poseEstimation || !overlay2DComponent || !overlay3DComponent || !img_mapper || !transform2D )
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }
    LOG_INFO("All components have been created");

	// the following code is common to the 3 samples (simple, compile-time, run-time)
	// 
	//
	// Declare data structures used to exchange information between components
	SRef<Image> refImage, camImage, kpImageCam;
	SRef<DescriptorBuffer> refDescriptors, camDescriptors;
	std::vector<DescriptorMatch> matches;

	Transform2Df Hm;
	std::vector< SRef<Keypoint> > refKeypoints, camKeypoints;  // where to store detected keypoints in ref image and camera image

	// load marker
	LOG_INFO("LOAD MARKER IMAGE ");
    marker->loadMarker();
	marker->getImage(refImage);

    // NOT WORKING ! Set the size of the box to the size of the natural image marker
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

	// detect keypoints in reference image
	LOG_INFO("DETECT MARKER KEYPOINTS ");
	kpDetector->detect(refImage, refKeypoints);

	// extract descriptors in reference image
	LOG_INFO("EXTRACT MARKER DESCRIPTORS ");
	descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);
	LOG_INFO("EXTRACT MARKER DESCRIPTORS COMPUTED");

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

	// initialize pose estimation
	poseEstimation->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

	// initialize image mapper with the reference image size and marker size
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(refImage->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(refImage->getSize().height);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(marker->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(marker->getSize().height);

	// to count the average number of processed frames per seconds
	boost::timer::cpu_timer mytimer;
	clock_t start, end;
	int count = 0;
	start = clock();

	// vector of 4 corners in the marker
	std::vector<SRef <Point2Df>> refImgCorners;
	Point2Df corner0(0, 0);
	Point2Df corner1((float)refImage->getWidth(), 0);
	Point2Df corner2((float)refImage->getWidth(), (float)refImage->getHeight());
	Point2Df corner3(0, (float)refImage->getHeight());
	refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner0));
	refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner1));
	refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner2));
	refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner3));


	// get images from camera in loop, and display them
	while (true) {
		count++;
		LOG_DEBUG("count : {}", count);
        Transform3Df pose;
		if (camera->getNextImage(camImage) == SolAR::FrameworkReturnCode::_ERROR_)
			break;

		Image matchesImage(refImage->getWidth() + camImage->getWidth(), refImage->getHeight(), refImage->getImageLayout(), refImage->getPixelOrder(), refImage->getDataType());
		SRef<Image> matchImage = xpcf::utils::make_shared<Image>(matchesImage);

		// detect keypoints in camera image
		kpDetector->detect(camImage, camKeypoints);
		// Not working, C2664 : cannot convert argument 1 from std::vector<boost_shared_ptr<Keypoint>> to std::vector<boost_shared_ptr<Point2Df>> !
#ifndef NDEBUG
        kpImageCam = camImage->copy();
        overlay2DComponent->drawCircles(camKeypoints, kpImageCam);
#endif
		/* you can either draw keypoints */
		// kpDetector->drawKeypoints(camImage,camKeypoints,kpImageCam);

		/* extract descriptors in camera image*/

		descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);

		/*compute matches between reference image and camera image*/
		matcher->match(refDescriptors, camDescriptors, matches);

        /* filter matches to remove redundancy and check geometric validity */
        basicMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
        geomMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);

		/* we declare here the Solar datastucture we will need for homography*/
		std::vector <SRef<Point2Df>> ref2Dpoints;
		std::vector <SRef<Point2Df>> cam2Dpoints;
		Point2Df point;
		std::vector <SRef<Point3Df>> ref3Dpoints;
		std::vector <SRef<Point2Df>> output2Dpoints;
		std::vector <SRef<Point2Df>> markerCornersinCamImage;
		std::vector <SRef<Point3Df>> markerCornersinWorld;

		/*we consider that, if we have less than 10 matches (arbitrarily), we can't compute homography for the current frame */

		if (matches.size()> 10) {
			// reindex the keypoints with established correspondence after the matching
			keypointsReindexer->reindex(refKeypoints, camKeypoints, matches, ref2Dpoints, cam2Dpoints);

			// mapping to 3D points
			img_mapper->map(ref2Dpoints, ref3Dpoints);

            Transform2DFinder::RetCode res = homographyEstimation->find(ref2Dpoints, cam2Dpoints, Hm);
			//test if a meaningful matrix has been obtained
            if (res == Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK)
			{
				//poseEstimation->poseFromHomography(Hm,pose,objectCorners,sceneCorners);
				// vector of 2D corners in camera image
				transform2D->transform(refImgCorners, Hm, markerCornersinCamImage);
				// draw circles on corners in camera image
				//overlay2DComponent->drawCircles(markerCornersinCamImage, 10, 5, kpImageCam);

				/* we verify is the estimated homography is valid*/
				if (homographyValidation->isValid(refImgCorners, markerCornersinCamImage))
				{
					// from the homography we create 4 points at the corners of the reference image
					// map corners in 3D world coordinates
					img_mapper->map(refImgCorners, markerCornersinWorld);

					// pose from solvePNP using 4 points.
					/* The pose could also be estimated from all the points used to estimate the homography */
                    poseEstimation->estimate(markerCornersinCamImage, markerCornersinWorld, pose);

					/* The pose last parameter can not be 0, so this is an error case*/
                    if (pose(3, 3) != 0.0)
					{
						/* We draw a box on the place of the recognized natural marker*/
#ifndef NDEBUG
                        overlay3DComponent->draw(pose, kpImageCam);
#else
                        overlay3DComponent->draw(pose, camImage);
#endif
                    }
					else
					{
						/* The pose estimated is false: error case*/
						LOG_INFO("no pose detected for this frame");
					}
				}
				else /* when homography is not valid*/
					LOG_INFO("Wrong homography for this frame");

			}
		}
#ifndef NDEBUG
        if (imageViewerResult->display(kpImageCam) == SolAR::FrameworkReturnCode::_STOP)
#else
        if (imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP)
#endif
            break;
	}

	end = clock();
	double duration = double(end - start) / CLOCKS_PER_SEC;
	printf("\n\nElasped time is %.2lf seconds.\n", duration);
	printf("Number of processed frame per second : %8.2f\n", count / duration);

	std::cout << "this is the end..." << '\n';

	// 
	//
	// end of the common code (simple,compile-time, run-time)
     return 0;
}
