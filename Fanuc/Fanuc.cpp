#include "Fanuc.h"
#include <iostream>
#include <sstream>
#include <Ws2tcpip.h>


FanucM20iA::FanucM20iA(const double xMin, const double yMin, const double zMin,
					   const double xMax, const double yMax, const double zMax)
	: _portS(59002),
	  _portR(59003),
	  _tmpAddrString("172.27.221.60"),
	  _wsaData(),
	  _workInJoint(false),
	  _maxValueOf(xMax, yMax, zMax),
	  _minValueOf(xMin, yMin, zMin)

{
	initSockets();
}

constexpr int FanucM20iA::buffLength = 64;

const std::map<int, std::string> FanucM20iA::tableOfErrors =
{
	{6, "Specified event object handle is invalid."},
	{8, "Insufficient memory available."},
	{87, "One or more parameters are invalid."},
	{995, "Overlapped operation aborted."},
	{996, "Overlapped I/O event object not in signaled state."},
	{997, "Overlapped operations will complete later."},
	{10004, "Interrupted function call."},
	{10009, "File handle is not valid."},
	{10013, "Permission denied."},
	{10014, "Bad address"},
	{10022, "Invalid argument."},
	{10024, "Too many open files."},
	{10035, "Resource temporarily unavailable."},
	{10036, "Operation now in progress."},
	{10037, "Operation already in progress."},
	{10038, "Socket operation on nonsocket."},
	{10039, "Destination address required."},
	{10040, "Message too long."},
	{10041, "Protocol wrong type for socket."},
	{10042, "Bad protocol option."},
	{10043, "Protocol not supported."},
	{10044, "Socket type not supported."},
	{10045, "Operation not supported."},
	{10046, "Protocol family not supported."},
	{10047, "Address family not supported by protocol family."},
	{10048, "Address already in use."},
	{10049, "Cannot assign requested address."},
	{10050, "Network is down."},
	{10051, "Network is unreachable."},
	{10052, "Network dropped connection on reset."},
	{10053, "Software caused connection abort."},
	{10054, "Connection reset by peer."},
	{10055, "No buffer space available."},
	{10056, "Socket is already connected."},
	{10057, "Socket is not connected."},
	{10058, "Cannot send after socket shutdown."},
	{10059, "Too many references."},
	{10060, "Connection timed out."},
	{10061, "Connection refused."},
	{10062, "Cannot translate name."},
	{10063, "Name too long."},
	{10064, "Host is down."},
	{10065, "No route to host."},
	{10066, "Directory not empty."},
	{10067, "Too many processes."},
	{10068, "User quota exceeded."},
	{10069, "Disk quota exceeded."},
	{10070, "Stale file handle reference."},
	{10071, "Item is remote."},
	{10091, "Network subsystem is unavailable."},
	{10092, "Winsock.dll version out of range."},
	{10093, "Successful WSAStartup not yet performed."},
	{10101, "Graceful shutdown in progress."},
	{10102, "No more results."},
	{100000, "Some another error occured..."}
};

FanucM20iA::ExtremeValuesMax::ExtremeValuesMax(const double& x, const double& y, const double& z):
	_x(x),
	_y(y),
	_z(z)
{
}

double FanucM20iA::ExtremeValuesMax::x() const
{
	return _x;
}

double FanucM20iA::ExtremeValuesMax::y() const
{
	return _y;
}

double FanucM20iA::ExtremeValuesMax::z() const
{
	return _z;
}

FanucM20iA::ExtremeValuesMin::ExtremeValuesMin(const double& x, const double& y, const double& z):
	_x(x),
	_y(y),
	_z(z)
{
}

double FanucM20iA::ExtremeValuesMin::x() const
{
	return _x;
}

double FanucM20iA::ExtremeValuesMin::y() const
{
	return _y;
}

double FanucM20iA::ExtremeValuesMin::z() const
{
	return _z;
}

void FanucM20iA::initSockets()
{
	if (WSAStartup(MAKEWORD(2, 2), &_wsaData) != 0)
	{
		errorHandler();
	}

	if ((_sockSend = socket(AF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR)
	{
		errorHandler();
	}
	if ((_sockReceive = socket(AF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

void FanucM20iA::startWorking() const
{
	const char* serveraddr = _tmpAddrString.c_str();

	sockaddr_in destAddr;
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(_portS);
	inet_pton(AF_INET, serveraddr, &destAddr.sin_addr);
	if (connect(_sockSend, reinterpret_cast<sockaddr*>(&destAddr), sizeof(destAddr)) == SOCKET_ERROR)
	{
		errorHandler();
	}

	sockaddr_in recvAddr;
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(_portR);
	inet_pton(AF_INET, serveraddr, &recvAddr.sin_addr);
	if (connect(_sockReceive, reinterpret_cast<sockaddr*>(&recvAddr), sizeof(recvAddr)) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

std::array<double, 6> FanucM20iA::parseString(char* str)
{
	std::array<double, 6> tmp;
	int k = 1;
	for (double& coord : tmp)
	{
		char sum[7] = "";
		int j = 0;
		while (str[k] != ' ' && str[k] != '\n')
		{
			sum[j] = str[k];
			++j;
			++k;
		}
		coord = atof(sum) / 1000;
		++k;
	}
	return tmp;
}

const char* FanucM20iA::createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
										   const double& wToRobot, const double& pToRobot, const double& rToRobot)
{
	std::stringstream tmpBuf;
	tmpBuf << static_cast<int>(xToRobot * 1000.) << ' ' << static_cast<int>(yToRobot * 1000.) << ' '
		<< static_cast<int>(zToRobot * 1000.) << ' ' << static_cast<int>(wToRobot * 1000.) << ' '
		<< static_cast<int>(pToRobot * 1000.) << ' ' << static_cast<int>(rToRobot * 1000.) << " 10 2 0";
	return _strdup(tmpBuf.str().c_str());
}

const char* FanucM20iA::createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
										   const double& wToRobot, const double& pToRobot, const double& rToRobot,
										   const int& seg)
{
	std::stringstream tmpBuf;
	tmpBuf << createStringToSend(xToRobot, yToRobot, zToRobot, wToRobot, pToRobot, rToRobot) << ' ' << seg << " 2 0";
	return _strdup(tmpBuf.str().c_str());
}

const char* FanucM20iA::createStringToSend(const double& xToRobot, const double& yToRobot, const double& zToRobot,
										   const double& wToRobot, const double& pToRobot, const double& rToRobot,
										   const int& seg, const int& type)
{
	std::stringstream tmpBuf;
	tmpBuf << createStringToSend(xToRobot, yToRobot, zToRobot, wToRobot, pToRobot, rToRobot) << ' ' << seg << ' ' << type
		<< " 0";
	return _strdup(tmpBuf.str().c_str());
}

void FanucM20iA::checkCoordsLimits(const double& x, const double& y, const double& z) const
{
	std::string errorMessage;
	if (!_workInJoint)
	{
		if (x > _maxValueOf.x()) errorMessage = "Exceeding the maximum limits along the X axis";
		if (x < _minValueOf.x()) errorMessage = "Exceeding the minimum limits along the X axis";

		if (y > _maxValueOf.y()) errorMessage = "Exceeding the maximum limits along the Y axis";
		if (y < _minValueOf.y()) errorMessage = "Exceeding the minimum limits along the Y axis";

		if (z > _maxValueOf.z()) errorMessage = "Exceeding the maximum limits along the Z axis";
		if (z < _minValueOf.z()) errorMessage = "Exceeding the minimum limits along the Z axis";
	}
	if (!errorMessage.empty())
	{
		std::cout << errorMessage << std::endl;
		std::cin.get();
		exit(-1);
	}
}

void FanucM20iA::setWorldFrame()
{
	_workInJoint = false;
	const char* c = "2";
	if (send(_sockSend, c, static_cast<int>(strlen(c)), 0) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

void FanucM20iA::setJointFrame()
{
	_workInJoint = true;
	const char* c = "0";
	if (send(_sockSend, c, static_cast<int>(strlen(c)), 0) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

void FanucM20iA::goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
								 const double& r) const
{
	checkCoordsLimits(x, y, z);
	const char* str = createStringToSend(x, y, z, w, p, r);
	if (send(_sockSend, str, static_cast<int>(strlen(str)), 0) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

void FanucM20iA::goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
								 const double& r, const int& seg) const
{
	checkCoordsLimits(x, y, z);
	const char* str = createStringToSend(x, y, z, w, p, r, seg);
	if (send(_sockSend, str, static_cast<int>(strlen(str)), 0) == SOCKET_ERROR)
	{
		errorHandler();
	}
}

void FanucM20iA::goToCoordinates(const double& x, const double& y, const double& z, const double& w, const double& p,
								 const double& r, const int& seg, const int& type) const
{
	checkCoordsLimits(x, y, z);
	const char* str = createStringToSend(x, y, z, w, p, r, seg, type);
	if (send(_sockSend, str, static_cast<int>(strlen(str)), 0) == SOCKET_ERROR)
	{
		errorHandler();
	}
}


std::array<double, 6> FanucM20iA::getJointAngles() const
{
	char buff[buffLength];
	const int valRead = recv(_sockReceive, buff, buffLength, 0);
	if (valRead == SOCKET_ERROR && valRead == 0)
	{
		errorHandler();
	}

	buff[valRead] = '\0';

	return parseString(buff);
}

FanucM20iA::~FanucM20iA()
{
	if (_workInJoint)
	{
		goToCoordinates(0., 0., 0., 0., -90., 0.);
		getJointAngles();
	}
	else
	{
		goToCoordinates(985., 0., 940., 180., 0., 0.);
		getJointAngles();
	}
	closesocket(_sockSend);
	closesocket(_sockReceive);
	WSACleanup();
}

void FanucM20iA::errorHandler()
{
	const int error = WSAGetLastError();
	auto tmpIter = tableOfErrors.find(error);
	if (tmpIter == tableOfErrors.end())
	{
		tmpIter = std::prev(tableOfErrors.end());
	}
	std::cout << "Error " << tmpIter->first << ":\t" << tmpIter->second << std::endl;
	std::cin.get();
	exit(error);
}
