#ifndef OBJECTCOORDINATES_H_2017_TIMUR
#define OBJECTCOORDINATES_H_2017_TIMUR

#include <vector>
#include <array>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio/videoio.hpp>

double calculateMedian(std::vector<double> valueVector);

cv::Mat createTransformationMatrix(const cv::Vec3d rotationVector,
                                   const cv::Vec3d translationVector);

std::array<double, 3> calculateMarkerPose(const cv::Mat transformationMatrix,
                                          const std::array<double, 6> jointCorners);

int startWebcamMonitoring(cv::VideoCapture& vid, const float arucoSqureDimension,
                          const cv::Mat cameraMatrix,
                          const cv::Mat distanceCoefficients);

int main();

#endif
