#include "CameraCalibration.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/shape/hist_cost.hpp>
#include <vector>

void timur::CameraCalibration::createKnownBoardPosition(std::vector<cv::Point3f>& corners) const
{
    switch (_pattern)
    {
        case Pattern::CHESSBOARD:
            [[fallthrough]];
        case Pattern::CIRCLES_GRID:
            for (decltype(_boardSize.height) i = 0; i < _boardSize.height; ++i)
            {
                for (decltype(_boardSize.width) j = 0; j < _boardSize.width; ++j)
                {
                    corners.emplace_back(static_cast<float>(j), static_cast<float>(i), 0.0f);
                }
            }
            break;

        case Pattern::ASYMMETRIC_CIRCLES_GRID:
            for (decltype(_boardSize.height) i = 0; i < _boardSize.height; ++i)
            {
                for (decltype(_boardSize.width) j = 0; j < _boardSize.width; ++j)
                {
                    corners.emplace_back(static_cast<float>(2 * j + i % 2), static_cast<float>(i),
                                         0.0f);
                }
            }
            break;
    }
}

void timur::CameraCalibration::getBoardCorners(const std::vector<cv::Mat>& images,
                                               std::vector<std::vector<cv::Point2f>>&
                                               allFoundCorners) const
{
    switch (_pattern)
    {
        case Pattern::CHESSBOARD:
            for (const auto image : images)
            {
                std::vector<cv::Point2f> pointBuf;
                const bool found = cv::findChessboardCorners(image, _boardSize, pointBuf,
                                                             cv::CALIB_CB_ADAPTIVE_THRESH
                                                             + cv::CALIB_CB_NORMALIZE_IMAGE
                                                             + cv::CALIB_CB_FILTER_QUADS);
                if (found)
                {
                    cv::cornerSubPix(image, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
                                     cv::TermCriteria(cv::TermCriteria::EPS
                                                      + cv::TermCriteria::COUNT, 50, 0.001));
                    allFoundCorners.push_back(pointBuf);
                }
            }
            break;
        case Pattern::CIRCLES_GRID:
            for (const auto image : images)
            {
                std::vector<cv::Point2f> pointBuf;
                const bool found = findCirclesGrid(image, _boardSize, pointBuf);
                if (found)
                {
                    allFoundCorners.push_back(pointBuf);
                }
            }
            break;
        case Pattern::ASYMMETRIC_CIRCLES_GRID:
            for (const auto image : images)
            {
                std::vector<cv::Point2f> pointBuf;
                const bool found = findCirclesGrid(image, _boardSize, pointBuf,
                                                   cv::CALIB_CB_ASYMMETRIC_GRID);
                if (found)
                {
                    allFoundCorners.push_back(pointBuf);
                }
            }
            break;
    }
}

void timur::CameraCalibration::calculateIntrinsicParameters(
    const std::vector<cv::Mat>& calibrationImages)
{
    std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;
    getBoardCorners(calibrationImages, chessboardImageSpacePoints);

    std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);
    createKnownBoardPosition(worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    std::vector<cv::Mat> tVec;
    std::vector<cv::Mat> rVec;
    cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, _boardSize,
                        _cameraMatrix,
                        _distortionCoefficients, rVec, tVec);
}

timur::CameraCalibration::CameraCalibration(const cv::Size boardDimension, const uint patternCode)
    : _pattern(static_cast<Pattern>(patternCode)),
      _boardSize(boardDimension)
{
}
