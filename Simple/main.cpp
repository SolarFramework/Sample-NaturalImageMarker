#include <iostream>

using namespace std;
#include "IComponentManager.h"

#include "SolARCameraOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARMarker2DNaturalImageOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#include "SolARDescriptorsExtractorSIFTOpencv.h"
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARHomographyEstimationOpencv.h"
#include "SolARHomographyValidation.h"
#include "SolARKeypointsReIndexer.h"
#include "SolARPoseEstimationOpencv.h"
#include "SolAR2DOverlayOpencv.h"
#include "SolAR3DOverlayOpencv.h"
#include "SolARImage2WorldMapper4Marker2D.h"
#include "SolAR2DTransform.h"

#include <string>

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::solver::pose;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::NONFREEOPENCV;
using namespace SolAR::MODULES::TOOLS;

namespace xpcf  = org::bcom::xpcf;

#include <string>

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds


/* print this help message to explain which arguments to pass*/
/* the content of the message displayed is in the readme.adoc file*/
int printHelp(){
    LOG_INFO("Missing parameters");
    const char *filename = "readme.adoc";
    std::ifstream ifs(filename);
    if(!ifs){
        LOG_ERROR("File {} does not exist", filename)
        return 1;
    }
    std::string line;
    while(std::getline(ifs,line))
        LOG_INFO("{}", line)
    ifs.close();
    return 1;
}

int main(int argc, char *argv[])
{

    /* create a console logger*/
    LOG_ADD_LOG_TO_CONSOLE();

    /* create a file logger*/
    //LOG_ADD_LOG_TO_FILE(".SolarLog.log","r");

    if(argc<4) {
        printHelp();
         return -1;
    }

    LOG_INFO("program is running");

    // declare and instantiate components
    LOG_INFO("Start creating components");


    /* create components  */
	SolARCameraOpencv* camera =new SolARCameraOpencv();
    SolARImageViewerOpencv* imageViewer = new SolARImageViewerOpencv(); /* an imageviewer can be used for visual feedback */
    SolARMarker2DNaturalImageOpencv* marker = new SolARMarker2DNaturalImageOpencv();
    SolARKeypointDetectorOpencv* kpDetector = new SolARKeypointDetectorOpencv();
    SolARDescriptorsExtractorSIFTOpencv* descriptorExtractor = new SolARDescriptorsExtractorSIFTOpencv();
    SolARDescriptorMatcherKNNOpencv*  matcher = new SolARDescriptorMatcherKNNOpencv();
    SolARHomographyEstimationOpencv* homographyEstimation = new SolARHomographyEstimationOpencv();
    SolARHomographyValidation* homographyValidation = new SolARHomographyValidation();
    SolARKeypointsReIndexer*   keypointsReindexer = new SolARKeypointsReIndexer();
    SolARPoseEstimationOpencv* poseEstimation = new SolARPoseEstimationOpencv();
    SolAR2DOverlayOpencv* overlay2DComponent = new SolAR2DOverlayOpencv();
    SolAR3DOverlayOpencv* overlay3DComponent = new SolAR3DOverlayOpencv();
    SolARImage2WorldMapper4Marker2D* img_mapper = new SolARImage2WorldMapper4Marker2D();
    SolAR2DTransform* transform2D = new SolAR2DTransform();

    LOG_INFO("All components have been instantiated");


	// the following code is common to the 3 samples (simple, compile-time, run-time)
	// 
	//
	// Declare data structures used to exchange information between components
	SRef<Image> refImage, camImage, kpImageCam;
	SRef<DescriptorBuffer> refDescriptors, camDescriptors;
	std::vector<DescriptorMatch> matches;


	Transform2Df Hm;
	std::vector< SRef<Keypoint> > refKeypoints, camKeypoints;  // where to store detected keypoints in ref image and camera image

															   // initialize keypoint detector
	kpDetector->setType(features::KeypointDetectorType::SIFT);


	// load marker
	LOG_INFO("LOAD MARKER IMAGE ");
	marker->loadMarker(argv[1]);
	marker->getImage(refImage);


	// detect keypoints in reference image
	LOG_INFO("DETECT MARKER KEYPOINTS ");
	kpDetector->detect(refImage, refKeypoints);


	// extract descriptors in reference image
	LOG_INFO("EXTRACT MARKER DESCRIPTORS ");
	descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);
	LOG_INFO("EXTRACT MARKER DESCRIPTORS COMPUTED");

#ifdef DEBUG
	// display keypoints in reference image
	// copy reference image
	SRef<Image> kpImage = refImage->copy();
	// draw circles on keypoints

	overlay2DComponent->drawCircles(refKeypoints, 3, 1, kpImage);
	// displays the image with circles in an imageviewer
	imageViewer->display("reference keypoints", kpImage);
#endif

	//  initalizes camera with the 3rd parameter of the program
	std::string cameraArg = std::string(argv[3]);

	//  checks if a video is given in parameters
	if (cameraArg.find("mp4") != std::string::npos || cameraArg.find("wmv") != std::string::npos || cameraArg.find("avi") != std::string::npos)
	{
		if (camera->start(argv[3]) != FrameworkReturnCode::_SUCCESS) // videoFile
		{
			LOG_ERROR("Video with url {} does not exist", argv[3]);
			return -1;
		}
	}
	else
	{  //no video in parameters, then the input camera is used
		if (camera->start(atoi(argv[3])) != FrameworkReturnCode::_SUCCESS) // Camera
		{
			LOG_ERROR("Camera with id {} does not exist", argv[3]);
			return -1;
		}
	}
        // load camera parameters from yml input file
        camera->loadCameraParameters(argv[2]);

	// initialize overlay 3D component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
	overlay3DComponent->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

	// initialize pose estimation
	poseEstimation->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());


	// initialize image mapper with the reference image size and marker size
	img_mapper->setParameters(refImage->getSize(), marker->getSize());

	// to count the average number of processed frames per seconds
	boost::timer::cpu_timer mytimer;
	clock_t start, end;
	int count = 0;
	start = clock();

	// The escape key to exit the sample
	char escape_key = 27;

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
		Pose pose;
		if (camera->getNextImage(camImage) == SolAR::FrameworkReturnCode::_ERROR_)
			break;

		Image matchesImage(refImage->getWidth() + camImage->getWidth(), refImage->getHeight(), refImage->getImageLayout(), refImage->getPixelOrder(), refImage->getDataType());
		SRef<Image> matchImage = xpcf::utils::make_shared<Image>(matchesImage);

		// detect keypoints in camera image
		kpDetector->detect(camImage, camKeypoints);
		// Not working, C2664 : cannot convert argument 1 from std::vector<boost_shared_ptr<Keypoint>> to std::vector<boost_shared_ptr<Point2Df>> !
		kpImageCam = camImage->copy();
		overlay2DComponent->drawCircles(camKeypoints, 3, 1, kpImageCam);

		/* you can either draw keypoints */
		// kpDetector->drawKeypoints(camImage,camKeypoints,kpImageCam);

		/* extract descriptors in camera image*/

		descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);

		/*compute matches between reference image and camera image*/
		matcher->match(refDescriptors, camDescriptors, matches);


		/* we declare here the Solar datastucture we will need for homography*/
		std::vector <SRef<Point2Df>> ref2Dpoints;
		std::vector <SRef<Point2Df>> cam2Dpoints;
		Point2Df point;
		std::vector <SRef<Point3Df>> ref3Dpoints;
		std::vector <SRef<Point2Df>> output2Dpoints;
		std::vector <SRef<Point2Df>> markerCornersinCamImage;
		std::vector <SRef<Point3Df>> markerCornersinWorld;

		std::vector<unsigned int> bgrValues;


		/*we consider that, if we have less than 10 matches (arbitrarily), we can't compute homography for the current frame */

		if (matches.size()> 10) {
			// reindex the keypoints with established correspondence after the matching
			keypointsReindexer->reindex(refKeypoints, camKeypoints, matches, ref2Dpoints, cam2Dpoints);

			// mapping to 3D points
			img_mapper->map(ref2Dpoints, ref3Dpoints);

			HomographyEstimation::RetCode res = homographyEstimation->findHomography(ref2Dpoints, cam2Dpoints, Hm);
			//test if a meaningful matrix has been obtained
			if (res == HomographyEstimation::RetCode::HOMOGRAPHY_ESTIMATION_OK)
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
					poseEstimation->poseFromSolvePNP(pose, markerCornersinCamImage, markerCornersinWorld);


					//LOG_INFO("Pose : {}", pose.toString());
					//poseEstimation->poseFromSolvePNP(pose, cam2Dpoints,ref3Dpoints);

					Transform3Df affineTransform = Transform3Df::Identity();

					/* The pose last parameter can not be 0, so this is an error case*/
					if (pose.getPoseTransform()(3, 3) != 0.0)
					{
						/* We draw a box on the place of the recognized natural marker*/
						overlay3DComponent->drawBox(pose, marker->getWidth(), marker->getHeight(), marker->getWidth()*0.5f, affineTransform, kpImageCam);
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

		if (imageViewer->display("camera keypoints", kpImageCam, &escape_key) == SolAR::FrameworkReturnCode::_STOP)
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

	/* delete components  */
	delete camera;
	delete imageViewer;
	delete marker;
	delete kpDetector;
	delete descriptorExtractor;
	delete matcher;
	delete homographyEstimation;
	delete homographyValidation;
	delete keypointsReindexer;
	delete poseEstimation;
	delete overlay2DComponent;
	delete overlay3DComponent;
	delete img_mapper;
	delete transform2D;

    return 0;
}