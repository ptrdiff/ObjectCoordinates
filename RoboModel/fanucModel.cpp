#include "fanucModel.h"
#include <opencv2/core.hpp>
#include <vector>
#define PI 3.14159265

FanucModel::FanucModel()
    : RoboModel(std::vector<std::array<double, 4>>{
          {0, 0, 150, PI / 2},
          {0, 0, 790, 0},
          {0, 0, 250, PI / 2},
          {835, 0, 0, -PI / 2},
          {0, 0, 0, PI / 2},
          {100, 0, 0, 0},
          {130, PI / 2, -90, 0},
          {-190, 0, 0, 0}
      }),
      _toCamera((cv::Mat_<double>(4, 4) << 0, -1, 0, -43, 1, 0, 0, -90, 0, 0, 1, 130, 0, 0, 0, 1)),
      _toSixth((cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -190, 0, 0, 0, 1)),
      _forMovingToCamera((cv::Mat_<double>(4, 4) << 0, -1, 0, -43, 1, 0, 0, 90, 0, 0, 1, 130, 0, 0,
                          0,
                          1))
{
}

std::vector<double> FanucModel::jointsToQ(std::array<double, 6> j)
{
    //degrees to radians
    for (int i = 0; i < 6; ++i)
    {
        j[i] *= PI / 180.0;
    }
    std::vector<double> q;
    q.resize(6);
    q[0] = j[0];
    q[1] = -j[1] + PI / 2;
    q[2] = j[2] + j[1];
    q[3] = -j[3];
    q[4] = j[4];
    q[5] = -j[5];
    return q;
}

cv::Mat FanucModel::fanucForwardTask(const std::array<double, 6> inputjoints)
{
    const std::vector<double> q = jointsToQ(inputjoints);
    return forwardTask(q);
}

std::array<double, 3> FanucModel::anglesFromMat(const cv::Mat p6)
{
    std::array<double, 3> angleVector;
    angleVector.at(0) = atan2(p6.at<double>(2, 1), p6.at<double>(2, 2));
    angleVector.at(1) = atan2(-p6.at<double>(2, 0),
                              sqrt(p6.at<double>(2, 1) * p6.at<double>(2, 1) + p6.at<double>(2, 2)
                                   * p6.at<double>(2, 2)));
    angleVector.at(2) = atan2(p6.at<double>(1, 0), p6.at<double>(0, 0));
    return angleVector;
}

std::array<double, 6> FanucModel::getCoordsFromMat(cv::Mat transformMatrix)
{
    std::array<double, 3> wprAngles = anglesFromMat(transformMatrix);

    std::array<double, 6> res;
    res[0] = transformMatrix.at<double>(0, 3);
    res[1] = transformMatrix.at<double>(1, 3);
    res[2] = transformMatrix.at<double>(2, 3);
    res[3] = wprAngles.at(0);
    res[4] = wprAngles.at(1);
    res[5] = wprAngles.at(2);
    return res;
}

cv::Mat FanucModel::getToCamera() const
{
    return _toCamera;
}

cv::Mat FanucModel::getToSixth() const
{
    return _toSixth;
}

cv::Mat FanucModel::getForMovingToCamera() const
{
    return _forMovingToCamera;
}
