#ifndef FANUC_H
#define FANUC_H

#include <winsock2.h>
#include <array>
#include <map>
#include <string>

#pragma comment(lib, "ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

/**
 * \brief Class of robot Fanuc M-20iA
 * \details You will connect to the robot and you can control it in world or joint frame by sending coordinate and getting current joint angles
 * \details You should use functions of this class only in the next sequence:
 * \code
 * int main()
 * {
 *	FanucM20iA fedya;
 *	fedya.startWorking();
 *	fedya.setWorldFrame();
 *	fedya.goToCoordinates(0., 0., 0., 0., 0., 0.);
 *	std::array<double, 6> answer = fedya.getJointAngles();
 * }
 * \endcode
 * You should to use this client with server: ZZZSTCAMERA
*/
class FanucM20iA
{
	/**
	 * \brief Socket for send coordinates
	 */
	SOCKET _sockSend;
	/**
	 * \brief Socket for receive coordinates
	 */
	SOCKET _sockReceive;
	/**
	 * \brief Port for send coordinates
	 */
	const int _portS;
	/**
	 * \brief Port for send coordinates
	 */
	const int _portR;
	/**
	 * \brief Robot's IP
	 */
	std::string _tmpAddrString;
	/**
	 * \brief Structure to initialize WinSock
	 */
	WSADATA _wsaData;

	/**
	 * \brief Flag of working in the Joint coordinate system
	 */
	bool _workInJoint;

	/**
	 * \brief Table of WinSock errors, which you can get from function WSAGetLastError();
	 */
	static const std::map<int, std::string> tableOfErrors;

	/**
	* \brief Size of buffer for receiving data
	*/
	const static int buffLength;

	/**
	 * \brief Class accumulate maximum boundary values
	 */
	class ExtremeValuesMax
	{
		const double _x;
		const double _y;
		const double _z;
	public:
		ExtremeValuesMax(const double& x, const double& y, const double& z);

		double x() const;
		double y() const;
		double z() const;
	} _maxValueOf;

	/**
	* \brief Class accumulate minimum boundary values
	*/
	class ExtremeValuesMin
	{
		const double _x;
		const double _y;
		const double _z;
	public:
		ExtremeValuesMin(const double& x, const double& y, const double& z);

		double x() const;
		double y() const;
		double z() const;
	} _minValueOf;

	/**
	* \brief Function to initialize the sockets
	*/
	void initSockets();

	/**
	* \brief Function to parse string with degrees from Fedya
	* \param[in] str Array of char from "recv" function of WinSock
	* \return Array of 6 doubles with coordinates or degrees
	*/
	static std::array<double, 6> parseString(char* str);

	/**
	 * \brief Function to create string for "send" function of WinSock
	 * \param[in] xToRobot 1st coordinate
	 * \param[in] yToRobot 2nd coordinate
	 * \param[in] zToRobot 3rd coordinate
	 * \param[in] wToRobot 4th coordinate
	 * \param[in] pToRobot 5th coordinate
	 * \param[in] rToRobot 6th coordinate
	 * \return Created array of char with coordinates
	 */
	static const char* createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
	                                      const double& wToRobot, const double& pToRobot, const double& rToRobot);

	/**
	* \brief Function to create string for "send" function of WinSock
	* \param[in] xToRobot 1st coordinate
	* \param[in] yToRobot 2nd coordinate
	* \param[in] zToRobot 3rd coordinate
	* \param[in] wToRobot 4th coordinate
	* \param[in] pToRobot 5th coordinate
	* \param[in] rToRobot 6th coordinate
	* \param[in] seg Segtime
	* \return Created array of char with coordinates
	*/
	static const char* createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
	                                      const double& wToRobot, const double& pToRobot, const double& rToRobot,
	                                      const int& seg);

	/**
	* \brief Function to create string for "send" function of WinSock
	* \param[in] xToRobot 1st coordinate
	* \param[in] yToRobot 2nd coordinate
	* \param[in] zToRobot 3rd coordinate
	* \param[in] wToRobot 4th coordinate
	* \param[in] pToRobot 5th coordinate
	* \param[in] rToRobot 6th coordinate
	* \param[in] seg Segtime
	* \param[in] type Type of moving: Nodecel or Fine
	* \return Created array of char with coordinates
	*/
	static const char* createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
	                                      const double& wToRobot, const double& pToRobot, const double& rToRobot,
	                                      const int& seg, const int& type);

	/**
	 * \brief Function of checking sending coordinates for work in the allowed area
	 * \param[in] x Cartesian coordinate: X
	 * \param[in] y Cartesian coordinate: Y
	 * \param[in] z Cartesian coordinate: Z
	 */
	void checkCoordsLimits(const double& x, const double& y, const double& z) const;

	/**
	 * \brief Function of display WinSock error
	 */
	static void errorHandler();

public:
	/**
	 * \brief Constructor of the object with setting of the boundary
	 * \details Creating exemplar of object with initialization the Fed's sockets
	 */
	explicit FanucM20iA(const double xMin = 680., const double yMin = -465., const double zMin = 420.,
	                    const double xMax = 1380., const double yMax = 465., const double zMax = 1200.);

	/**
	* \brief Function to connect to the sockets
	*/
	void startWorking() const;

	/**
	 * \brief Function to stand world coordinate system
	 * \warning You should use this function in the beginning of work with the robot
	 */
	void setWorldFrame();

	/**
	* \brief Function to stand joint coordinate system
	* \warning You should use this function in the beginning of work with the robot
	*/
	void setJointFrame();

	/**
	 * \brief Send Fedya to the given coordinates
	 * \param[in] x Cartesian cordinate
	 * \param[in] y Cartesian cordinate
	 * \param[in] z Cartesian cordinate
	 * \param[in] w Angle: roll
	 * \param[in] p Angle: pitch
	 * \param[in] r Angle: yaw
	 * \warning You should use this function only after setting coordinate system
	 */
	void goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
	                     const double& r) const;

	/**
	* \brief Send Fedya to the given coordinates and segtime
	* \param[in] x Cartesian cordinate
	* \param[in] y Cartesian cordinate
	* \param[in] z Cartesian cordinate
	* \param[in] w Angle: roll
	* \param[in] p Angle: pitch
	* \param[in] r Angle: yaw
	* \param[in] seg Segtime
	* \warning You should use this function only after setting coordinate system
	*/
	void goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
	                     const double& r, const int& seg) const;

	/**
	* \brief Send Fedya to the given coordinates, segtime and type of moving
	* \param[in] x Cartesian cordinate
	* \param[in] y Cartesian cordinate
	* \param[in] z Cartesian cordinate
	* \param[in] w Angle: roll
	* \param[in] p Angle: pitch
	* \param[in] r Angle: yaw
	* \param[in] seg Segtime
	* \param[in] type Type of moving
	* \warning You should use this function only after setting coordinate system
	*/
	void goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
	                     const double& r, const int& seg, const int& type) const;

	/**
	* \brief Return Joint coordinates after moving to the point
	* \return Array of 6 angles
	* \warning You should use this function only after sending coordinates to the robot
	*/
	std::array<double, 6> getJointAngles() const;

	/**
	 * \brief Destructor of the elenemts of class
	 * \details Send Fedya to the Homepos and close connections
	 */
	~FanucM20iA();
};

#endif // FANUC_H
