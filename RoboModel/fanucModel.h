#ifndef FANUC_MODEL
#define FANUC_MODEL
#include "newRM.h"
#define PI 3.14159265

/**
* \brief class for matematics of robot manipulator Fanuc M20ia based on Denavit-Hartenberg parameters
*/
class FanucModel : RoboModel
{
    /**
    * \brief function for conversion joints angles to Denavit-Hartenberg generalized angles
    * \param[in] j joints angles
    * \return Denavit-Hartenberg generalized angles
    */
    static std::vector<double> jointsToQ(std::array<double, 6> j);

    /**
    * \brief function to calculate three rotation angles from transformation matrix
    * \param[in] p6 transofrmation matrix(4x4) or rotation matrix(3x3)
    * \return three Tait-Bryan angles
    */
    static std::array<double, 3> anglesFromMat(const cv::Mat p6);

    const cv::Mat _toCamera, _toSixth, _forMovingToCamera;
public:
    /**
    * \brief default constructor with Fanuc M20ia parameters
    */
    FanucModel();

    /**
    * \brief function for solving forward kinematic task for Fanuc M20ia
    * \param[in] inputjoints joints angles
    * \return coordinates of end-effector in world frame: x, y, z in mm and w, p, r in radians
    */
    cv::Mat fanucForwardTask(std::array<double, 6> inputjoints);

    cv::Mat getToCamera() const;

    cv::Mat getToSixth() const;

    cv::Mat FanucModel::getForMovingToCamera() const;

    static std::array<double, 6> getCoordsFromMat(cv::Mat transformMatrix);
};

#endif
