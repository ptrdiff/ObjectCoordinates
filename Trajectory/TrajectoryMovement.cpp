#include "TrajectoryMovement.h"

Point operator+(const Point &p1, const Point &p2)
{
	return Point{ p1.at(0) + p2.at(0), p1.at(1) + p2.at(1), p1.at(2) + p2.at(2) };
}

Point operator-(const Point &p1, const Point &p2)
{
	return Point{ p1.at(0) - p2.at(0), p1.at(1) - p2.at(1), p1.at(2) - p2.at(2) };
}

Point operator*(const Point &p1, const int i)
{
	return Point{ p1.at(0) * i, p1.at(1) * i, p1.at(2) * i};
}

Point operator/(const Point & p1, const int i)
{
	return Point{ p1.at(0) / i, p1.at(1) / i, p1.at(2) / i };
}

void TrajectoryMovement::createTrajectory(Point StartCoords, Point FinalCoords)
{
	_movementQueue.clear();
	for (int i = 0; i <= _count; ++i) {
		_movementQueue.push_back(StartCoords + (FinalCoords - StartCoords) * i / _count);
	}
}

TrajectoryMovement::TrajectoryMovement(Point StartCoords, Point FinalCoords, int count)
	:_count(count)
{
	createTrajectory(StartCoords, FinalCoords);
}

Point TrajectoryMovement::getTrajPoint(Point FinalCoords)
{
	if (_movementQueue.size() == 1) 
	{
		return _movementQueue.front();
	}
	if (FinalCoords == _movementQueue.back())
	{
		_movementQueue.pop_front();
		return Point(_movementQueue.front());
	}
	else
	{
		createTrajectory(_movementQueue.front(), FinalCoords);
		_movementQueue.pop_front();
		return Point(_movementQueue.front());
	}
}

