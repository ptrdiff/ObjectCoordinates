/**
 * \file
 * \brief Header file with class description for camera calibration.
*/
#ifndef CAMERA_CALIBRATION_2017
#define CAMERA_CALIBRATION_2017
#include <opencv2/core.hpp>
#include <vector>

namespace timur
{
/**
 * \brief Class for working with camera (calibration and removal of distortion).
 */
class CameraCalibration
{
protected:

    /**
     * \brief Pattern for calibration.(chessboard = 0, circlesGrid = 1, asymmetricCirclesGrid = 2).
     */
    enum class Pattern
    {
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID
    } const _pattern;

    /**
     * \brief Dimension of chessboard or circle grid pattern (Number of items by width and height).
     */
    const cv::Size _boardSize;

    /**
    * \brief Camera matrix - intrinsic parameters of the camera.
    * 
    * \f[ camera \; matrix = \left [ \begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ] \f]
    * focal length (fx,fy), optical centers (cx,cy).
    */
    cv::Mat _cameraMatrix;

    /**
    * \brief Distortion coefficients = (k1 k2 p1 p2 k3)
    * 
    * Due to radial distortion, straight lines will appear curved. Its effect is more as we move away 
    * from the center of image.
    * \f[x_{distorted}=x(1+k_1r^2+k_2r^4+k_3r^6)\f]
    * \f[y_{distorted}=y(1+k_1r^2+k_2r^4+k_3r^6)\f].
    * Similarly, another distortion is the tangential distortion which occurs because image taking 
    * lense is not aligned perfectly parallel to the imaging plane.
    * \f[x_{distorted} = x+[2p_1xy+p_2(r^2+2x^2)]\f]
    * \f[y_{distorted} = y+[p_1(r^2+2y^2)+2p_2xy]\f].
    */
    cv::Mat _distortionCoefficients;

    /**
     * \brief Creation of real 3-dimensional coordinates for chessboard/circle grid points. 
     * \param[out] corners Output vector of 3-dimenshion points.
     */
    void createKnownBoardPosition(std::vector<cv::Point3f>& corners) const;

    /**
     * \brief Find chessboard/circle grid points on collection of images.
     * \param[in] images Input vector of matrices, where need to find a chessboard/circle grid.
     * \param[out] allFoundCorners Output vector of 2-dimenshion coordinates of chessboard/circle grid points.
     */
    void getBoardCorners(const std::vector<cv::Mat>& images,
                         std::vector<std::vector<cv::Point2f>>& allFoundCorners) const;

    /**
     * \brief CameraCalibration constructor.
     * \param[in] boardDimension Dimension of chessboard or circle grid pattern 
     * (Number of items by width and height).
     * \param[in] patternCode Code of using pattern
     * (chessboard = 0, circlesGrid = 1, asymmetricCirclesGrid = 2).
     */
    explicit CameraCalibration(const cv::Size boardDimension, const uint patternCode);

    /**
     * \brief CameraCalibration destructor.
     */
    virtual ~CameraCalibration() = default;

    /**
    * \brief Calibrating the camera on the image collection.
    * \param[in] calibrationImages Input vector of images with chessboard/circle grid.
    */
    void calculateIntrinsicParameters(const std::vector<cv::Mat>& calibrationImages);
};
}

#endif //!CAMERA_CALIBRATION_2017
