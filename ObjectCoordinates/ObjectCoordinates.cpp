#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <ArucoMarkers.h>
#include <CamCalibWI.h>
#include <fanucModel.h>
#include <Fanuc.h>
#include <TrajectoryMovement.h>

#include "ObjectCoordinates.h"


cv::Mat createTransformationMatrix(const cv::Vec3d rotationVector,
	const cv::Vec3d translationVector)
{
	cv::Mat rotationMatrix;
	cv::Rodrigues(rotationVector, rotationMatrix);
	cv::Mat transformationMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
	rotationMatrix.copyTo(transformationMatrix(cv::Rect(0, 0, 3, 3)));
	cv::Mat(translationVector * 1000).copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));
	transformationMatrix.at<double>(3, 3) = 1;
	return transformationMatrix;
}

std::array<double, 3> calculateMarkerPose(const cv::Mat transformationMatrix,
	const std::array<double, 6> jointCorners)
{
	FanucModel robot;
	const cv::Mat p6 = robot.fanucForwardTask(jointCorners);
	cv::Mat res = p6 * robot.getToCamera() * transformationMatrix * robot.getToSixth();
	return std::array<double, 3>{res.at<double>(0, 3), res.at<double>(1, 3), res.at<double>(2, 3)};
}

int startWebcamMonitoring(cv::VideoCapture& vid, const float arucoSqureDimension,
	const cv::Mat cameraMatrix, const cv::Mat distanceCoefficients)
{
	if (!vid.isOpened())
	{
		return -1;
	}

	cv::Mat frame;
	timur::ArucoMarkers arucoMarkers(arucoSqureDimension, false);
	std::vector<cv::Vec3d> rotationVectors, translationVectors;
	std::vector<int> markerIds;

	cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	FanucM20iA fanuc;
	fanuc.startWorking();
	fanuc.setWorldFrame();

	FanucModel robot;
	const cv::Mat p6 = robot.fanucForwardTask({ 0., 0., 0., 0., -90., 0. });

	Point startPoint = { p6.at<double>(0,3) , p6.at<double>(1, 3), p6.at<double>(2, 3) };
	while (true)
	{
		if (!vid.read(frame))
		{
			break;
		}

		const bool foundMarkers = arucoMarkers.estimateMarkersPose(frame, cameraMatrix,
			distanceCoefficients,
			rotationVectors,
			translationVectors, markerIds);

		if (foundMarkers)
		{
			const cv::Mat res = p6 * robot.getToCamera() * createTransformationMatrix(cv::Vec3d(0, 0, 0), translationVectors[0]) * robot.getToSixth();
			Point finalPoint = { res.at<double>(0, 3), res.at<double>(1, 3), res.at<double>(2, 3) };
			TrajectoryMovement trajectory(startPoint,finalPoint, 10);
			Point curPoint = { 0, 0, 0 };
			while (curPoint != finalPoint)
			{
				curPoint = trajectory.getTrajPoint(finalPoint);
				std::cout << "-----" << curPoint[0] << ' ' << curPoint[1] << ' ' << curPoint[2] << '\n';
				fanuc.goToCoordinates(curPoint[0],
					                  curPoint[1],
					                  curPoint[2],
					                          180,
					                            0,
					                            0);
				fanuc.getJointAngles();
				cv::imshow("Webcam", frame);
				if (cv::waitKey(1) == 27)
				{
					break;
				}
			}
			system("pause");
			break;
		}
		cv::imshow("Webcam", frame);
		cv::waitKey(1);
	}
	cv::destroyWindow("Webcam");
	return 0;
}

int main()
{
	///const cv::Size boardDimensions = cv::Size(4, 11);
	const float arucoSqureDimension = 0.062f;

	cv::VideoCapture vid(1);
	timur::CamCalibWi camera("CamCalibStable.txt");

	startWebcamMonitoring(vid, arucoSqureDimension, camera.cameraMatrix(),
		camera.distortionCoefficients());
	return 0;
}
