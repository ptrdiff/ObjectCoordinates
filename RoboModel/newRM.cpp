#include "newRM.h"
#include <opencv2/core.hpp>
#define PI 3.14159265

struct RoboModel::DhParameters
{
    double _dParam;
    double _qParam;
    double _aParam;
    double _alphaParam;

    DhParameters(const double d, const double q, const double a, const double alpha)
        : _dParam(d),
          _qParam(q),
          _aParam(a),
          _alphaParam(alpha)
    {
    }
};


RoboModel::RoboModel(std::vector<std::array<double, 4>> input)
{
    _kinematicChain.reserve(input.size());
    for (int i = 0; i < input.size(); ++i)
        _kinematicChain.push_back(DhParameters(input[i][0], input[i][1], input[i][2], input[i][3]));
}

RoboModel::~RoboModel()
{
}

cv::Mat RoboModel::forwardTask(std::vector<double> inputq)
{
    _kinematicChain[0]._qParam = inputq[0];
    cv::Mat transformMatrix = prevMatTransform(0);
    for (int i = 1; i < inputq.size(); ++i)
    {
        _kinematicChain[i]._qParam = inputq[i];
        transformMatrix = transformMatrix * prevMatTransform(i);
    }

    return transformMatrix;
}

cv::Mat RoboModel::prevMatTransform(const int i)
{
    cv::Mat result(4, 4, CV_64F);
    result.at<double>(0, 0) = cos(_kinematicChain[i]._qParam);
    result.at<double>(0, 1) = -cos(_kinematicChain[i]._alphaParam) *
                              sin(_kinematicChain[i]._qParam);
    result.at<double>(0, 2) = sin(_kinematicChain[i]._alphaParam) * sin(_kinematicChain[i]._qParam);
    result.at<double>(0, 3) = _kinematicChain[i]._aParam * cos(_kinematicChain[i]._qParam);

    result.at<double>(1, 0) = sin(_kinematicChain[i]._qParam);
    result.at<double>(1, 1) = cos(_kinematicChain[i]._alphaParam) * cos(_kinematicChain[i]._qParam);
    result.at<double>(1, 2) = -sin(_kinematicChain[i]._alphaParam) *
                              cos(_kinematicChain[i]._qParam);
    result.at<double>(1, 3) = _kinematicChain[i]._aParam * sin(_kinematicChain[i]._qParam);

    result.at<double>(2, 0) = 0;
    result.at<double>(2, 1) = sin(_kinematicChain[i]._alphaParam);
    result.at<double>(2, 2) = cos(_kinematicChain[i]._alphaParam);
    result.at<double>(2, 3) = _kinematicChain[i]._dParam;

    result.at<double>(3, 0) = result.at<double>(3, 1) = result.at<double>(3, 2) = 0;
    result.at<double>(3, 3) = 1;

    return result;
}
