#include "ArucoMarkers.h"

#include <string>
#include <iostream>
#include <filesystem>

#include <opencv2/core/core.hpp>
#include "dictionary.hpp"
#include "aruco.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/shape/hist_cost.hpp>

timur::ArucoMarkers::ArucoMarkers(const float arucoSqureDimension,
                                  const bool usePredefinedDictionary)
    : _arucoSqureDimension(arucoSqureDimension)
{
    if (usePredefinedDictionary)
    {
        std::cout
                << "DICT_4X4_50 = 0" << '\n'
                << "DICT_4X4_100 = 1" << '\n'
                << "DICT_4X4_250 = 2" << '\n'
                << "DICT_4X4_1000 = 3" << '\n';
        int dictionaryNumber;
        std::cout << "Enter the number of dictionary you want: ";
        std::cin >> dictionaryNumber;
        std::cout << '\n';
        _markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryNumber);
    }
    else
    {
        int countOfMarkers, markerSize;
        std::cout << "Enter the count of markers and theirs size: ";
        std::cin >> countOfMarkers >> markerSize;
        std::cout << '\n';
        _markerDictionary = cv::aruco::generateCustomDictionary(countOfMarkers, markerSize);
        std::cout << "Markers Dictionary was created! " << '\n';
    }
}

float timur::ArucoMarkers::arucoSqureDimension() const
{
    return _arucoSqureDimension;
}

void timur::ArucoMarkers::createArucoMarkers(const std::string& folderName, const uint& imageSize,
                                             const uint& borderSize) const
{
    std::experimental::filesystem::create_directory(folderName);

    cv::Mat outputMarker;
    for (int i = 0; i < _markerDictionary->bytesList.rows; ++i)
    {
        cv::aruco::drawMarker(_markerDictionary, i, imageSize, outputMarker, borderSize);
        std::stringstream convert;
        convert << folderName << '/' << i << ".png";
        cv::imwrite(convert.str(), outputMarker);
    }
}

bool timur::ArucoMarkers::estimateMarkersPose(const cv::Mat frame, const cv::Mat cameraMatrix,
                                              const cv::Mat distanceCoefficients,
                                              std::vector<cv::Vec3d>& rotationVectors,
                                              std::vector<cv::Vec3d>& translationVectors,
                                              std::vector<int>& markerIds) const
{
    cv::Mat frameHsv;
    std::vector<cv::Mat> hsvChannels;
    cv::cvtColor(frame, frameHsv, CV_BGR2HSV);
    cv::split(frameHsv, hsvChannels);

    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(hsvChannels[2], _markerDictionary, markerCorners, markerIds);
    if (!markerCorners.empty())
    {
        cv::aruco::estimatePoseSingleMarkers(markerCorners, _arucoSqureDimension, cameraMatrix,
                                             distanceCoefficients, rotationVectors,
                                             translationVectors);
        cv::aruco::drawDetectedMarkers(frame, markerCorners);
        return true;
    }
    return false;
}
