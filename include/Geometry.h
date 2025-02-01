#pragma once
#include <iostream>

constexpr double Tolerance_small = 0.0001;

class P3D;     // 3D point
class V3D;     // 3D vector
class Plane3D; // 3D plane


// 3D Point class
//-------------------------------------------------------------------------
class P3D
{
public:
	P3D() : _x(0), _y(0), _z(0) {}
	explicit P3D(const double x, const double y, const double z) : _x(x), _y(y), _z(z) {}
	P3D(const P3D& p) : _x(p.x()), _y(p.y()), _z(p.z()) {}
	~P3D() = default;

	void set(const double x, const double y, const double z) { _x = x;  _y = y;  _z = z; }
	[[nodiscard]] double x() const { return _x; }
	[[nodiscard]] double y() const { return _y; }
	[[nodiscard]] double z() const { return _z; }

	P3D operator+(const V3D& vec) const;
	P3D operator-(const V3D& vec) const;
	bool operator== (const P3D& rhs) const;

	double distanceToLineSegment(const P3D& pA, const P3D& pB) const; // Minimal distance to line segment (pA, pB)

private:
	double _x;
	double _y;
	double _z;
};
//-------------------------------------------------------------------------


// 3D Vector class
//-------------------------------------------------------------------------
class V3D
{
public:
	V3D() : _x(0), _y(0), _z(0) {}
	explicit V3D(const double x, const double y, const double z) : _x(x), _y(y), _z(z) {}
	explicit V3D(const P3D& start, const P3D& end);
	explicit V3D(const V3D& v) : _x(v.x()), _y(v.y()), _z(v.z()) {}
	~V3D() = default;

	void set(const double x, const double y, const double z) { _x = x;  _y = y;  _z = z; }
	[[nodiscard]] double x() const { return _x; }
	[[nodiscard]] double y() const { return _y; }
	[[nodiscard]] double z() const { return _z; }

	V3D normal() const;  // Returns the unit vector in the direction of this vector
	double length() const;  // Length of the vector    
	bool isNull() const;    // Returns true if the vector is null
	V3D& operator *=(double k);  // Scales the vector by a scalar

	V3D crossProduct(const V3D& vA) const;  // Cross product of two vectors
	double dotProduct(const V3D& vA) const;   // Dot product of two vectors

private:
	double _x;
	double _y;
	double _z;
};
//-------------------------------------------------------------------------


// 3D Plane class
//-------------------------------------------------------------------------
class Plane3D
{
public:

	Plane3D() : _A(1), _B(0), _C(0), _D(0) {}
	explicit Plane3D(const P3D& point,const V3D& norm);

	~Plane3D() = default;

	double distanceToPoint(const P3D& point) const;  // Distance from the plane to a point
	bool isOn(const P3D& point) const;  // Check if the point lies on the plane
	V3D normal() const { return V3D{ A(),B(), C() }.normal(); }

	double A() const { return _A; }
	double B() const { return _B; }
	double C() const { return _C; }
	double D() const { return _D; }

private:

	double _A;
	double _B;
	double _C;
	double _D;
};
//-------------------------------------------------------------------------

// DistanceCalculator class - calculates the minimal distance between two 3D segments
//-------------------------------------------------------------------------
class DistanceCalculator
{
public:

	explicit DistanceCalculator(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4);
	double calculate() const;

private:
	// Check if two line segments share a common endpoint
	static bool haveCommonEndpoint(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4);

	// Check if two line segments intersect
	// Note that they need to be on the same plane
	static bool doLineSegmentsIntersect(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4, const V3D& dir);

	// project point on the plane
	static P3D projectPointOnPlane(const P3D& point, const Plane3D& plane);

	// Calculate the minimum distance between the ends of two line segments
	static double calculateMinDistanceBetweenEnds(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4);

	P3D p1() const { return _p1; }
	P3D p2() const { return _p2; }
	P3D p3() const { return _p3; }
	P3D p4() const { return _p4; }

	P3D _p1;
	P3D _p2;
	P3D _p3;
	P3D _p4;
};
//-------------------------------------------------------------------------
