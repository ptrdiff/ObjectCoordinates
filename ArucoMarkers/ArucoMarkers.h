/**
* \file
* \brief Header file with class description for working with aruco markers.
*/
#ifndef ARUCO_DETECTION_MARKERS_2017
#define ARUCO_DETECTION_MARKERS_2017

#include <string>

#include <dictionary.hpp>

namespace timur
{
/**
 * \brief Class for working with aruco markers(generate it and estimate position of found markers).
 */
class ArucoMarkers
{
private:
    /**
     * \brief The length of the aruco marker's side.
     */
    const float _arucoSqureDimension;

    /**
     * \brief Dictionary of markers.
     */
    cv::Ptr<cv::aruco::Dictionary> _markerDictionary;

public:

    /**
     * \brief ArucoMarkers constructor, generating dictionary of markers with certain size.
     * \param[in] arucoSqureDimension The length of the aruco marker's side.
     * \param[in] usePredefinedDictionary If false generate custom dictionary.
     * (need to enter count of markers and theirs size).
     */
    explicit ArucoMarkers(float arucoSqureDimension, bool usePredefinedDictionary = true);

    /**
     * \brief ArucoMarkers destructor.
     */
    virtual ~ArucoMarkers() = default;

    /**
     * \brief Returning property of _arucoSqureDimension.
     * \return Value of the private field _arucoSqureDimension.
     */
    float arucoSqureDimension() const;

    /**
     * \brief Creating aruco markers images from dictionary and saving them.
     * \param[in] folderName Folder name for saving markers images.
     * \param[in] imageSize Length of the image side in pixels.
     * \param[in] borderSize Width of the marker border in pixels.
     */
    void createArucoMarkers(const std::string& folderName, const uint& imageSize = 500,
                            const uint& borderSize = 1) const;

    /**
     * \brief Estimate markers positions on frame and draw them.
     * \param[in] frame Frame for calculating markers positions.
     * \param[in] cameraMatrix Intrinsic parameters of the camera.
     * \param[in] distanceCoefficients Distortion coefficients.
     * \param[out] rotationVectors Array of output rotation vectors.
     * \param[out] translationVectors Array of output translation vectors.
     * \param[out] markerIds Array of identifiers of the detected markers.
     * \return True, if the markers are on the frame, and false, if not.
     */
    bool estimateMarkersPose(const cv::Mat frame, const cv::Mat cameraMatrix,
                                                  const cv::Mat distanceCoefficients,
                                                  std::vector<cv::Vec3d>& rotationVectors,
                                                  std::vector<cv::Vec3d>& translationVectors,
                                                  std::vector<int>& markerIds) const;
};
}

#endif //!ARUCO_DETECTION_MARKERS_2017
