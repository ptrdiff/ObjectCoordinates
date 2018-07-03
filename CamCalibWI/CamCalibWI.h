#ifndef CAMERA_CALIBRATION_2017_WITH_INTERFACE
#define CAMERA_CALIBRATION_2017_WITH_INTERFACE

#include <opencv2/highgui/highgui.hpp>

#include <CameraCalibration.h>

namespace timur
{
class CamCalibWi : private CameraCalibration
{
private:
    /**
    * \brief Saving camera calibration parameters in file.
    * \param[in] name Name of file to save.
    */
    void saveCameraCalibration(const std::string& name) const;

    /**
    * \brief Download camera calibration parameters from file.
    * \param[in] name File name for download.
    */
    void loadCameraCalibration(const std::string& name);

    /**
    * \brief Calculates image blurriness.
    * \param[in] src Input image.
    * \return Rate of blurriness.
    */
    static float calcBlurriness(const cv::Mat& src);

public:
    /**
    * \brief CamCalibWi constructor.
    * \param[in] boardDimension Dimension of chessboard(the intersection of cells).
    * \param[in] patternCode Code of using pattern.
    * (chessboard = 0, circlesGrid = 1, asymmetricCirclesGrid = 2). 
    */
    CamCalibWi(const cv::Size boardDimension, const uint patternCode);

    /**
    * \brief CamCalibWi constructor.
    * \param[in] calibrationFileName Filename with calibration parameters.
    */
    explicit CamCalibWi(const std::string& calibrationFileName);

    ~CamCalibWi() override = default;

    /**
    * \brief Download images from file and calibrate camera. (image name: i.png, i = 0,countOfImages)
    * \param[in] folderName File name for download images.
    * \param[in] countOfImages Count of images.
    */
    void cameraCalibrationImagesFromFolder(const std::string& folderName, const uint countOfImages);

    /**
    * \brief Starting camera calibration.
    * \param[in] vid Opencv camera object initialized with needed camera.
    * \param[in] countOfFrames Count of images for calibration.
    */
    void cameraCalibrationProcess(cv::VideoCapture& vid, const uint countOfFrames);

    /**
    * \brief Returning property of _cameraMatrix.
    * \return Value of the private field _cameraMatrix.
    */
    cv::Mat cameraMatrix() const;

    /**
    * \brief Returning property of _distortionCoefficientss.
    * \return Value of the private field _distortionCoefficients.
    */
    cv::Mat distortionCoefficients() const;

    /**
    * \brief Transforms an image to compensate for lens distortion.
    * \param[in] inputImage Input image.
    * \return Output image without distortion.
    */
    cv::Mat undistort(cv::Mat& inputImage) const;

    /**
    * \brief Helps adjust the camera's manual focus.
    * \param[in] vid Opencv camera object initialized with needed camera.
    */
    static void focusSetting(cv::VideoCapture& vid);
};
}
#endif //!CAMERA_CALIBRATION_2017_WITH_INTERFACE
