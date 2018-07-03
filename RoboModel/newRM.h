#ifndef NEW_RM
#define NEW_RM
#include <vector>
#include <opencv2/core/mat.hpp>
#include <array>

/**
* \brief class for matematics of robot manipulator based on Denavit-Hartenberg parameters
*/
class RoboModel
{
private:

    /**
    * \brief struct of D-H parameters
    *
    * d: offset along previous z to the common normal;
    * q: angle about previous  z, from old  x to new  x;
    * a: offset along x in current frame;
    * alpha: angle about x in current frame;
    */
    struct DhParameters;

    /**
    * \brief vector of parameters for each joint
    */
    std::vector<DhParameters> _kinematicChain;

protected:
    /**
    * \brief function to calculate a transform matrix from i-th frame to (i-1)-th
    * \param[in] i number of frame
    * \return transform matrix (4x4)
    */
    cv::Mat prevMatTransform(const int i);

    /**
    * \brief constructor with parameters for any robot
    * \param[in] input vector of secuences d, q, a, alpha
    */
    explicit RoboModel(std::vector<std::array<double, 4>> input);

    ~RoboModel();

    /**
    * \brief function for solving forward kinematic task
    * \param[in] inputq generalized D-H coordinates
    * \return coordinates of end-effector in world frame: x, y, z in mm and w, p, r in radians
    */
    cv::Mat forwardTask(std::vector<double> inputq);
};

#endif
