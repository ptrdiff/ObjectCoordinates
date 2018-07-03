#include "CamCalibWI.h"
#include <fstream>
#include <iostream>
#include <opencv2/shape/hist_cost.hpp>
#include <opencv2/calib3d.hpp>

timur::CamCalibWi::CamCalibWi(const cv::Size boardDimension, const uint patternCode)
	: CameraCalibration(boardDimension, patternCode)
{
}

timur::CamCalibWi::CamCalibWi(const std::string& calibrationFileName)
	: CameraCalibration(cv::Size(), 0)
{
	loadCameraCalibration(calibrationFileName);
}

void timur::CamCalibWi::saveCameraCalibration(const std::string& name) const
{
	std::ofstream outStream(name);
	if (outStream)
	{
		outStream << _cameraMatrix.rows << '\n';
		outStream << _cameraMatrix.cols << '\n';

		for (int r = 0; r < _cameraMatrix.rows; ++r)
		{
			for (int c = 0; c < _cameraMatrix.cols; ++c)
			{
				outStream << _cameraMatrix.at<double>(r, c) << '\n';
			}
		}

		outStream << _distortionCoefficients.rows << '\n';
		outStream << _distortionCoefficients.cols << '\n';

		for (int r = 0; r < _distortionCoefficients.rows; ++r)
		{
			for (int c = 0; c < _distortionCoefficients.cols; ++c)
			{
				outStream << _distortionCoefficients.at<double>(r, c) << '\n';
			}
		}

		outStream.close();
	}
}

void timur::CamCalibWi::loadCameraCalibration(const std::string& name)
{
	std::ifstream inStream(name);
	if (!inStream.is_open())
	{
		std::cout << "Can not open file! Wrong name!" << '\n';
		return;
	}

	int rows;
	int columns;

	inStream >> rows;
	inStream >> columns;

	_cameraMatrix = cv::Mat(cv::Size(columns, rows), cv::DataType<double>::type);

	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < columns; ++c)
		{
			inStream >> _cameraMatrix.at<double>(r, c);
			std::cout << _cameraMatrix.at<double>(r, c) << '\n';
		}
	}

	inStream >> rows;
	inStream >> columns;

	_distortionCoefficients = cv::Mat::zeros(rows, columns, cv::DataType<double>::type);

	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < columns; ++c)
		{
			inStream >> _distortionCoefficients.at<double>(r, c);
			std::cout << _distortionCoefficients.at<double>(r, c) << '\n';
		}
	}
	std::cout << '\n';
	inStream.close();
}

cv::Mat timur::CamCalibWi::undistort(cv::Mat& inputImage) const
{
	if (_cameraMatrix.empty() || _distortionCoefficients.empty())
	{
		return inputImage;
	}
	cv::Mat outputImage;
	cv::undistort(inputImage, outputImage, _cameraMatrix, _distortionCoefficients);
	return outputImage;
}

float timur::CamCalibWi::calcBlurriness(const cv::Mat& src)
{
	cv::Mat gx, gy;
	cv::Sobel(src, gx, CV_32F, 1, 0);
	cv::Sobel(src, gy, CV_32F, 0, 1);
	const double normGx = cv::norm(gx);
	const double normGy = cv::norm(gy);
	const double sumSq = normGx * normGx + normGy * normGy;
	return static_cast<float>(1. / (sumSq / src.size().area() + 1e-6));
}

void timur::CamCalibWi::focusSetting(cv::VideoCapture& vid)
{
	while (true)
	{
		cv::Mat temp;
		vid >> temp;
		cv::putText(temp, std::to_string(calcBlurriness(temp)), cv::Point(50, 50),
			cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255));
		cv::imshow("Image View", temp);
		if (cv::waitKey(1) == 27)
		{
			break;
		}
	}
}

void timur::CamCalibWi::cameraCalibrationImagesFromFolder(const std::string& folderName,
	const uint countOfImages)
{
	if (_boardSize.empty())
	{
		return;
	}
	std::vector<cv::Mat> calibrationImages;
	cv::Mat imageGray;
	for (uint i = 0; i < countOfImages; ++i)
	{
		cv::cvtColor(cv::imread(folderName + '/' + std::to_string(i) + ".png"), imageGray,
			CV_BGR2GRAY);
		calibrationImages.push_back(imageGray);
	}
	calculateIntrinsicParameters(calibrationImages);
}

void timur::CamCalibWi::cameraCalibrationProcess(cv::VideoCapture& vid, const uint countOfFrames)
{
	if (!vid.isOpened() || _boardSize.empty())
	{
		return;
	}

	cv::Mat frame, frameGray;
	std::vector<cv::Mat> savedImages;
	std::vector<cv::Point2f> foundPoints;

	cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	uint countOfGoodFrames = 0;
	while (true)
	{
		if (!vid.read(frame))
		{
			break;
		}
		cv::cvtColor(frame, frameGray, CV_BGR2GRAY);
		bool found = false;
		switch (_pattern)
		{
		case Pattern::CHESSBOARD:
			found = cv::findChessboardCorners(frameGray, _boardSize, foundPoints,
				cv::CALIB_CB_ADAPTIVE_THRESH
				+ cv::CALIB_CB_NORMALIZE_IMAGE
				+ cv::CALIB_CB_FILTER_QUADS
				+ cv::CALIB_CB_FAST_CHECK);
			break;
		case Pattern::CIRCLES_GRID:
			found = cv::findCirclesGrid(frameGray, _boardSize, foundPoints);
			break;
		case Pattern::ASYMMETRIC_CIRCLES_GRID:
			found = cv::findCirclesGrid(frameGray, _boardSize, foundPoints,
				cv::CALIB_CB_ASYMMETRIC_GRID);
			break;
		}
		if (found)
		{
			cv::Mat frameToDraw(frame.clone());
			cv::drawChessboardCorners(frameToDraw, _boardSize, cv::Mat(foundPoints), found);
			cv::imshow("Webcam", frameToDraw);
			if (cv::waitKey(1) == ' ')
			{
				++countOfGoodFrames;
				cv::Mat temp;
				frameGray.copyTo(temp);
				savedImages.push_back(temp);
				std::cout << countOfGoodFrames << '/' << countOfFrames << '\n';
				if (savedImages.size() >= countOfFrames)
				{
					std::cout << "Started calibration.." << '\n';
					calculateIntrinsicParameters(savedImages);
					std::cout << "Saving calibration parametrs.." << '\n';
					saveCameraCalibration("CamCalib.txt");
					std::cout << "Saved!" << '\n';
					break;
				}
			}
		}
		else
		{
			cv::imshow("Webcam", frame);
			if (cv::waitKey(1) == 27)
			{
				break;
			}
		}
	}
	cv::destroyWindow("Webcam");
}

cv::Mat timur::CamCalibWi::cameraMatrix() const
{
	return _cameraMatrix.clone();
}

cv::Mat timur::CamCalibWi::distortionCoefficients() const
{
	return _distortionCoefficients.clone();
}
