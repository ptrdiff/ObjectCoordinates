#pragma once
#include <deque>
#include <array>

using Point = std::array<double, 3>;

Point operator+ (const Point &p1, const Point &p2);
Point operator- (const Point &p1, const Point &p2);
Point operator* (const Point &p1, const int i);
Point operator/ (const Point &p1, const int i);

class TrajectoryMovement
{
private:
	std::deque<Point> _movementQueue;
	const int _count;

	void createTrajectory(Point StartCoords, Point FinalCoords);
public:
	TrajectoryMovement(Point StartCoords, Point FinalCoords, int count = 1);
	~TrajectoryMovement() = default;
	Point getTrajPoint(Point FinalCoords);
};

