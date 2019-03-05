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

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"


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


//const int THRES_TRACK1 = 80;

const int THRES_TRACK1 = 30;
const int THRES_TRACK2 = 200;
//const int THRES_AR = 80;
//const int THRES_AR = 20;
const int THRES_AR = 10;
//const int THRES_TO_TRACK = 80;
const int THRES_TO_TRACK = 30;
const int MIN_HEIGHT_DETECT = 480;


const cv::Scalar				RED(0, 0, 255);
const cv::Scalar				GREEN(0, 255, 0);
const cv::Scalar				BLUE(255, 0, 0);
const cv::Scalar				YELLOW(0, 255, 255);


bool checkUpdate(cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &curPts);
void get2D3DPoints(cv::Mat &img, std::vector<cv::Point2f> &pts2D, std::vector<cv::Point2f> &pts3D);
void getProj(cv::Mat &proj);
bool computeCameraPose(std::vector<cv::Point2f> &pts2D_obj, std::vector<cv::Point2f> &pts2D, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &inliers);

void matchFeatures(cv::Mat &img_sce, std::vector<cv::Point2f> &pts2D, std::vector<cv::Point2f> &pts2D_obj);

void visualAR(cv::Mat &img,cv::Mat &rvec, cv::Mat &tvec,cv::Mat &cameraMatrix,cv::Mat &distCoeffs);

/*	====================================================================== Variables ==============================================================================								
*/

// camera parameters
int width_camera;
int height_camera;
std::string pIntrinsic;
cv::Mat cameraMatrix;// = (cv::Mat_<double>(3, 3) << 649.7, 0.0, 319.5, 0.0, 649.7, 239.5, 0.0, 0.0, 1.0);
cv::Mat distCoeffs;// = cv::Mat::zeros(5, 1, CV_64FC1);

// Marker
std::string pMarker;
cv::Mat img_object;
std::vector<cv::KeyPoint> kps_object;
cv::Mat des_object;
float size_x = 0.285f;
float size_y= 0.197f;
float size_z=1.0f;

// feature detector
std::string featureType;
cv::Ptr<cv::FeatureDetector> detector;

// optical flow
cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
cv::Size subPixWinSize(10, 10), winSize(31, 31);

// pose and results
cv::Mat							rvec, tvec, inliers;
float							reprojErr, confidence;
bool							isPose, isTrack;
std::vector<cv::Point3f>		point3DAR;


bool valid_pose = false;
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

    //setup variable
    width_camera = camera->getResolution().width;
    height_camera = camera->getResolution().height;

    cameraMatrix.create(3, 3, CV_64FC1);
    distCoeffs.create(5, 1, CV_64FC1);


    rvec= cv::Mat::ones(3, 3, CV_64FC1);
    tvec= cv::Mat::zeros(3, 1, CV_64FC1);

    distCoeffs.at<double>(0, 0) = camera->getDistorsionParameters()(0);
    distCoeffs.at<double>(1, 0) = camera->getDistorsionParameters()(1);
    distCoeffs.at<double>(2, 0) = camera->getDistorsionParameters()(2);
    distCoeffs.at<double>(3, 0) = camera->getDistorsionParameters()(3);
    distCoeffs.at<double>(4, 0) = camera->getDistorsionParameters()(4);

    cameraMatrix.at<double>(0, 0) = camera->getIntrinsicsParameters()(0, 0);
    cameraMatrix.at<double>(0, 1) = camera->getIntrinsicsParameters()(0, 1);
    cameraMatrix.at<double>(0, 2) = camera->getIntrinsicsParameters()(0, 2);
    cameraMatrix.at<double>(1, 0) = camera->getIntrinsicsParameters()(1, 0);
    cameraMatrix.at<double>(1, 1) = camera->getIntrinsicsParameters()(1, 1);
    cameraMatrix.at<double>(1, 2) = camera->getIntrinsicsParameters()(1, 2);
    cameraMatrix.at<double>(2, 0) = camera->getIntrinsicsParameters()(2, 0);
    cameraMatrix.at<double>(2, 1) = camera->getIntrinsicsParameters()(2, 1);
    cameraMatrix.at<double>(2, 2) = camera->getIntrinsicsParameters()(2, 2);


    point3DAR = {
        cv::Point3f(0.f, 0.f, 0.f),
        cv::Point3f(size_x, 0.f, 0.f),
        cv::Point3f(size_x, size_y, 0.f),
        cv::Point3f(0.f, size_y, 0.f),
        cv::Point3f(0.f, 0.f, size_z),
        cv::Point3f(size_x, 0.f, size_z),
        cv::Point3f(size_x, size_y, size_z),
        cv::Point3f(0.f, size_y, size_z)
    };


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


	// Marker tracking
	std::vector<cv::Point2f> pts3D;
	cv::Mat frame, gray, prevGray;
	std::vector<cv::Point2f> prePts, curPts;
	isTrack = false;

    //detector = cv::AKAZE::create(5, 0, 3, 0.0001);
    detector = cv::ORB::create(15000);

    img_object = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(refImage);
    detector->detectAndCompute(img_object, cv::Mat(), kps_object, des_object);



	// get images from camera in loop, and display them
	while (true) {

        valid_pose = false;
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

        frame = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(camImage);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		if (isTrack) {

			std::vector<uchar> status;
			std::vector<float> err;
			// tracking 2D-2D
			
			cv::calcOpticalFlowPyrLK(prevGray, gray, prePts, curPts, status, err, winSize,
				3, termcrit, 0, 0.001);

			size_t i, k;
			for (i = k = 0; i < curPts.size(); i++)
			{
				if (!status[i]) {
					continue;
				}

				curPts[k] = curPts[i];
				pts3D[k] = pts3D[i];
				k++;

				circle(frame, curPts[i], 3, cv::Scalar(0, 255, 0), -1, 8);
			}

			curPts.resize(k);
			pts3D.resize(k);			

			// calculate camera pose
			isPose = computeCameraPose(pts3D, curPts, rvec, tvec, inliers);
			std::cout << "OF>> Number of Inliers: " << inliers.rows << std::endl;

			if (inliers.rows < THRES_TRACK1) {
				prePts.clear();
				curPts.clear();
				isTrack = false;
			}
			else {
				if ((inliers.rows < THRES_TRACK2) || checkUpdate(rvec, tvec, curPts)) {
					get2D3DPoints(gray, prePts, pts3D);
				}
				else {
					std::swap(curPts, prePts);
				}
				cv::swap(prevGray, gray);
			}								
		}
        else
        {
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


                    std::vector<cv::Point2f> tmpPts2D;
                    std::vector<cv::Point2f> tmpPts2D_obj;
                    matchFeatures(gray, tmpPts2D, tmpPts2D_obj);

                    computeCameraPose(tmpPts2D_obj, tmpPts2D, rvec, tvec, inliers);
                    std::cout<<"INLIERS "<< inliers.rows << std::endl;
                    if (inliers.rows > THRES_TO_TRACK) {
                                    pts3D.clear();
                                    get2D3DPoints(gray, prePts, pts3D);
                                    prevGray = gray.clone();
                                    isTrack = true;
                                   // std::cout<<"START TRACKING"<<std::endl;
                                   // cv::waitKey(0);
                    }
                    valid_pose = true;

				}
				else /* when homography is not valid*/
					LOG_INFO("Wrong homography for this frame");

			}


		}
		}//end of detector once

        //draw if valid pose
        if (pose(3, 3) != 0.0 && valid_pose)
        {
            // We draw a box on the place of the recognized natural marker
             #ifndef NDEBUG
            overlay3DComponent->draw(pose, kpImageCam);
            #else
            overlay3DComponent->draw(pose, camImage);
            #endif
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



/*	Compute pose
*/

bool computeCameraPose(std::vector<cv::Point2f> &pts2D_obj, std::vector<cv::Point2f> &pts2D, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &inliers)
{
    /* calculate camera pose
	*/
	inliers = cv::Mat();
    bool pnp = false;
	if (pts2D_obj.size() < 10){
		return false;
	}

	std::vector<cv::Point2f> imagePoints;
	cv::undistortPoints(pts2D, imagePoints, cameraMatrix, distCoeffs);

    std::cout<<"cameraMatrix >> :"<< cameraMatrix<<std::endl;

	cv::Mat status;
    cv::Mat oHw = findHomography(pts2D_obj, imagePoints, cv::RANSAC, 100.0, status);


    std::cout<<"oHw >> :"<< oHw<<std::endl;

    for (int i = 0; i < status.rows; ++i){
		if (status.at<uchar>(i, 0) == 1)
			inliers.push_back(i);
    }

    if (inliers.rows < THRES_AR)

        return false;

    {
		std::vector<cv::Point2f> tmp_pts1, tmp_pts2;
		for (int i = 0; i < inliers.rows; ++i) {
			int j = inliers.at<int>(i, 0);
			tmp_pts1.push_back(pts2D_obj[j]);
			tmp_pts2.push_back(imagePoints[j]);
		}
		oHw = cv::findHomography(tmp_pts1, tmp_pts2);
	}

	// Normalization to ensure that ||c1|| = 1
	double norm = sqrt(oHw.at<double>(0, 0)*oHw.at<double>(0, 0)
		+ oHw.at<double>(1, 0)*oHw.at<double>(1, 0)
		+ oHw.at<double>(2, 0)*oHw.at<double>(2, 0));
	oHw /= norm;

	cv::Mat c1 = oHw.col(0);
	cv::Mat c2 = oHw.col(1);
	cv::Mat c3 = c1.cross(c2);
	cv::Mat otw(3, 1, CV_64F); // Translation vector
	cv::Mat oRw(3, 3, CV_64F); // Rotation matrix
	otw = oHw.col(2);

	for (int i = 0; i < 3; i++) {
		oRw.at<double>(i, 0) = c1.at<double>(i, 0);
		oRw.at<double>(i, 1) = c2.at<double>(i, 0);
		oRw.at<double>(i, 2) = c3.at<double>(i, 0);
	}

	cv::Mat W, U, Vt;

	SVDecomp(oRw, W, U, Vt);
	oRw = U*Vt;
	tvec = otw.clone();	
	Rodrigues(oRw, rvec);
	
	return true;
}

void getProj(cv::Mat &proj) {
	cv::Mat rotW2C;
	cv::Rodrigues(rvec, rotW2C);
	cv::Mat ext = (cv::Mat_<double>(3, 3) <<
		rotW2C.at<double>(0, 0), rotW2C.at<double>(0, 1), tvec.at<double>(0, 0),
		rotW2C.at<double>(1, 0), rotW2C.at<double>(1, 1), tvec.at<double>(1, 0),
		rotW2C.at<double>(2, 0), rotW2C.at<double>(2, 1), tvec.at<double>(2, 0));
	cv::Mat proj_mat = cameraMatrix * ext;
	proj = proj_mat.inv();
}

void get2D3DPoints(cv::Mat &img, std::vector<cv::Point2f> &pts2D, std::vector<cv::Point2f> &pts3D) {

	pts2D.clear();
	pts3D.clear();
	// get project matrix
	cv::Mat proj;
	getProj(proj);

	// get bouding box
	std::vector<cv::Point2f> corners;
	cv::projectPoints(std::vector<cv::Point3f>(point3DAR.begin(), point3DAR.begin() + 4), rvec, tvec, cameraMatrix, distCoeffs, corners);
	
	for (int i = 0; i < corners.size(); ++i) {
		corners[i].x = std::min(corners[i].x, width_camera - 2.f);
		corners[i].x = std::max(corners[i].x, 2.f);
		corners[i].y = std::min(corners[i].y, height_camera - 2.f);
		corners[i].y = std::max(corners[i].y, 2.f);
	}

	cv::Rect box = cv::boundingRect(corners);
	cv::Mat roi = img(box);

	int MAX_COUNT = 1500;
	std::vector<cv::Point2f> lPts2D;

	// Next step: set adaptive parameter based on size of marker
	//cv::goodFeaturesToTrack(roi, corners, MAX_COUNT, 0.01, 9, cv::Mat(), 3, false, 0.001);
	cv::goodFeaturesToTrack(roi, lPts2D, MAX_COUNT, 0.01, 9, cv::Mat(), 3, false, 0.001);
	cornerSubPix(roi, lPts2D, subPixWinSize, cv::Size(-1, -1), termcrit);

	//rectangle(img, box, cv::Scalar(255), 2);

	for (int i = 0; i < lPts2D.size(); ++i) {
		
		lPts2D[i] = lPts2D[i] + (cv::Point2f)box.tl();
		cv::Mat p2D = (cv::Mat_<double>(3, 1) << lPts2D[i].x, lPts2D[i].y, 1.f);
		cv::Mat p3D = proj * p2D;
		
		double x = p3D.at<double>(0, 0) / p3D.at<double>(2, 0);
		double y = p3D.at<double>(1, 0) / p3D.at<double>(2, 0);
		
		if ((x > 0) && (x < size_x) && (y > 0) && (y < size_y)) {

			pts2D.push_back(lPts2D[i]);
			pts3D.push_back(cv::Point2f(x, y));
			//circle(img, lPts2D[i], 3, Scalar(0, 255, 0), -1, 8);
		
		}
	}	
	//std::cout << "Number of new point: " << pts3D.size() << std::endl;
}

bool checkUpdate(
	cv::Mat &rvec,
	cv::Mat &tvec,
	std::vector<cv::Point2f> &curPts)
{

	cv::Point2f sum = std::accumulate( curPts.begin(), curPts.end(),  cv::Point2f(0.0f, 0.0f),  std::plus<cv::Point2f>());
	cv::Point2f mean1 = sum / (int)curPts.size(); // Divide by count to get mean

	std::vector<cv::Point2f> corners;
	cv::projectPoints(std::vector<cv::Point3f>(point3DAR.begin(), point3DAR.begin() + 4), rvec, tvec, cameraMatrix, distCoeffs, corners);
	float disThres = 0.06 * (cv::norm(corners[0] - corners[1]) + cv::norm(corners[1] - corners[2]));
	for (int i = 0; i < corners.size(); ++i) {
		corners[i].x = std::min(corners[i].x, width_camera - 2.f);
		corners[i].x = std::max(corners[i].x, 2.f);
		corners[i].y = std::min(corners[i].y, height_camera - 2.f);
		corners[i].y = std::max(corners[i].y, 2.f);
	}
	sum = std::accumulate(corners.begin(), corners.end(), cv::Point2f(0.0f, 0.0f),  std::plus<cv::Point2f>());
	cv::Point2f mean2 = sum / (int)corners.size(); // Divide by count to get mean

	if (cv::norm(mean1 - mean2) > disThres) {
		return true;
	}
	else
		return false;
}


void matchFeatures(cv::Mat &img_sce, std::vector<cv::Point2f> &pts2D, std::vector<cv::Point2f> &pts2D_obj)
{
	// resize of image to accelerate 
	cv::Mat tmp_img;
	float scale;
	if (img_sce.rows > MIN_HEIGHT_DETECT) {

		scale = (float)img_sce.rows / MIN_HEIGHT_DETECT;
		cv::resize(img_sce, tmp_img, cv::Size((int)(img_sce.cols / scale), MIN_HEIGHT_DETECT));
	}
	else {
		tmp_img = img_sce;
		scale = 1.0;
	}

	// extract feature of current frame
	std::vector<cv::KeyPoint>	kps_sce;
	cv::Mat						des_sce;

	detector->detect(tmp_img, kps_sce);
	
	if (kps_sce.size() == 0)
		return;

	detector->compute(tmp_img, kps_sce, des_sce);

	for(unsigned int k = 0; k < kps_sce.size(); k++){
		kps_sce[k].pt *= scale ; 
	}
	cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_L2, false);
	
	std::vector< std::vector<cv::DMatch> > matches_2nn_12;
	std::vector< std::vector<cv::DMatch> > matches_2nn_21;
	
	matcher->knnMatch(des_object, des_sce, matches_2nn_12, 2);	
	matcher->knnMatch(des_sce, des_object, matches_2nn_21, 2);
	
	const double ratio = 0.7;
	std::vector< cv::DMatch > good_matches;
	
	for (int i = 0; i < matches_2nn_12.size(); i++) {
		// i is queryIdx
		if ((matches_2nn_12[i][0].distance / matches_2nn_12[i][1].distance < ratio)
			&&
			(matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].distance / matches_2nn_21[matches_2nn_12[i][0].trainIdx][1].distance < ratio))
		{
			if (matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].trainIdx == matches_2nn_12[i][0].queryIdx)
			{
				cv::Point2f pts_obj = kps_object[matches_2nn_12[i][0].queryIdx].pt;
				cv::Point2f pts_sce = kps_sce[matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].queryIdx].pt;
				pts2D_obj.push_back(cv::Point2f(pts_obj.x / (float)img_object.cols * size_x, pts_obj.y / (float)img_object.rows * size_y));
				pts2D.push_back(pts_sce);
				good_matches.push_back(matches_2nn_12[i][0]);
			}
		}
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )				
	cv::Mat img_matches;
	drawMatches(img_object, kps_object, img_sce, kps_sce,
	good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cv::imshow("Matches", img_matches);

}



void visualAR(
	cv::Mat &img,
	cv::Mat &rvec,
	cv::Mat &tvec,
	cv::Mat &cameraMatrix,
	cv::Mat &distCoeffs)
{
	int lineSize = 8;
	int circleSize = 6;
	std::vector<cv::Point2f> point2D;
	cv::projectPoints(point3DAR, rvec, tvec, cameraMatrix, distCoeffs, point2D);

	cv::line(img, point2D[0], point2D[1], YELLOW, lineSize);
	cv::line(img, point2D[1], point2D[2], YELLOW, lineSize);
	cv::line(img, point2D[2], point2D[3], YELLOW, lineSize);
	cv::line(img, point2D[3], point2D[0], YELLOW, lineSize);
	cv::line(img, point2D[4], point2D[5], GREEN, lineSize);
	cv::line(img, point2D[5], point2D[6], GREEN, lineSize);
	cv::line(img, point2D[6], point2D[7], GREEN, lineSize);
	cv::line(img, point2D[7], point2D[4], GREEN, lineSize);
	cv::line(img, point2D[0], point2D[4], BLUE, lineSize);
	cv::line(img, point2D[1], point2D[5], BLUE, lineSize);
	cv::line(img, point2D[2], point2D[6], BLUE, lineSize);
	cv::line(img, point2D[3], point2D[7], BLUE, lineSize);

	cv::circle(img, point2D[0], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[1], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[2], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[3], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[4], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[5], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[6], circleSize, RED, cv::FILLED);
	cv::circle(img, point2D[7], circleSize, RED, cv::FILLED);
}
