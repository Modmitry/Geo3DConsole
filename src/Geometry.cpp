#include "Geometry.h"

// class P3D
//-------------------------------------------------------------------------
P3D P3D::operator+(const V3D& vec) const
{
	return P3D{x() + vec.x(), y() + vec.y(), z() + vec.z()};
}
//-------------------------------------------------------------------------
P3D P3D::operator-(const V3D& vec) const
{
	return P3D{ x() - vec.x(), y() - vec.y(), z() - vec.z()};
}
//-------------------------------------------------------------------------
bool P3D::operator==(const P3D& rhs) const
{
	return (abs(x() - rhs.x()) < Tolerance_small &&
		abs(y() - rhs.y()) < Tolerance_small &&
		abs(z() - rhs.z()) < Tolerance_small);
}
//-------------------------------------------------------------------------
double P3D::distanceToLineSegment(const P3D& pA, const P3D& pB) const
{
	// The distance from a point to a segment is either the perpendicular dropped from the point onto the segment, 
	// or the minimum distance from the point to one of the segment's endpoints.

	// If the triangle, with vertices at the given point and the endpoints of the segment, 
	// is obtuse (the check for obtuseness is done by examining the sign of the dot product 
	// of the corresponding vectors formed along the sides of the triangle (the cosine of the obtuse angle is negative)), 
	// (i.e., it is not possible to drop a perpendicular from the point onto the segment), 
	// then the distance from the point to the segment is considered to be the minimum distance from the point 
	// to one of the segment's endpoints.

	const V3D v1(pA, *this);
	const V3D v2(pB, *this);
	const V3D v_p1_p2(pA, pB);
	const V3D v_p2_p1(pB, pA);

	if (v_p1_p2.dotProduct(v1) < 0 || v_p2_p1.dotProduct(v2) < 0)
	{
		return std::min(v1.length(), v2.length());
	}

	// Find the length of the perpendicular
	// First, let's calculate the area of the triangle
	// It is half the area of the parallelogram formed by vectors v1 and v2

	// Let's find the area multiplied by 2 (so we don't have to multiply by 2 again later)
	const double square2x = V3D(v_p1_p2.crossProduct(v1)).length();

	// Now we can express the height, knowing the area and the length of the base.
	return square2x / v_p1_p2.length();
}
//-------------------------------------------------------------------------

// class V3D
//-------------------------------------------------------------------------
V3D::V3D(const P3D& start, const P3D& end)
	: _x (end.x() - start.x())
	, _y(end.y() - start.y())
	, _z(end.z() - start.z())
{
}
//-------------------------------------------------------------------------
V3D V3D::normal() const
{
	const double len = length();
	return V3D{ x() / len, y() / len, z() / len };
}
//-------------------------------------------------------------------------
double V3D::length() const
{
	return sqrt(pow(x(), 2)+ pow(y(), 2) + pow(z(), 2));
}
//-------------------------------------------------------------------------
bool V3D::isNull() const
{
	return abs(x()) < Tolerance_small &&
		abs(y()) < Tolerance_small &&
		abs(z()) < Tolerance_small;
}
//-------------------------------------------------------------------------
V3D& V3D::operator*=(double k)
{
	_x *= k;
	_y *= k;
	_z *= k;

	return *this;
}
//-------------------------------------------------------------------------
V3D V3D::crossProduct(const V3D& vA) const
{
	return V3D{ y() * vA.z() - z() * vA.y(),
		z() * vA.x() - x() * vA.z(),
		x() * vA.y() - y() * vA.x() };
}
//-------------------------------------------------------------------------
double V3D::dotProduct(const V3D& vA) const
{
	return x() * vA.x() + y() * vA.y() + z() * vA.z();
}
//-------------------------------------------------------------------------

// class Plane3D
//-------------------------------------------------------------------------
Plane3D::Plane3D(const P3D& point, const V3D& norm): _A(0), _B(0), _C(0), _D(0)
{
	// normalize vector
	const auto normVector = norm.normal();

	_A = normVector.x();
	_B = normVector.y();
	_C = normVector.z();
	_D = -_A * point.x() - _B * point.y() - _C * point.z();
}

//-------------------------------------------------------------------------
double Plane3D::distanceToPoint(const P3D& point) const
{
	const double normFactor = sqrt(pow(A(), 2) + pow(B(), 2) + pow(C(), 2));
	return abs(A() * point.x() + B() * point.y() + C() * point.z() + D()) / normFactor;
}
//-------------------------------------------------------------------------
bool Plane3D::isOn(const P3D& point) const
{
	return abs(A() * point.x() + B() * point.y() + C() * point.z() + D()) < Tolerance_small;
}
//-------------------------------------------------------------------------

// class DistanceCalculator
//-------------------------------------------------------------------------
DistanceCalculator::DistanceCalculator(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4)
	:_p1(p1)
	, _p2(p2)
	, _p3(p3)
	, _p4(p4)
{
}
//-------------------------------------------------------------------------
double DistanceCalculator::calculate() const
{
	double min_dist{ 0.0 };

	// Check in case the segments have a common endpoint
	if (haveCommonEndpoint(p1(), p2(), p3(), p4()))
	{
		return min_dist;
	}

	// Find the direction vectors
	const V3D v12(p1(), p2());
	const V3D v34(p3(), p4());

	// Check if the segment is a point (the points coincide)
	if (v12.isNull() || v34.isNull())
	{
		return -1;
	}

	// perpendicular vector
	const V3D cross = v12.crossProduct(v34);

	if (cross.isNull())
	{
		// If the vector is zero, it means the vectors are collinear, and therefore, the lines containing the segments are parallel
		// So, it means that the segments are in the same plane
		// The minimum distance is the smallest of the 4 distances from the endpoints of one segment to the other segment
		min_dist = calculateMinDistanceBetweenEnds(p1(), p2(), p3(), p4());
	}
	else
	{
		// Find the distance between the skew lines.
		// Project all points onto a single plane.
		const Plane3D plane(p1(), cross);
		const double distanceBetweenPlanes = plane.distanceToPoint(p3()); //or p4

		const P3D p1_proj{ p1()};
		const P3D p2_proj = projectPointOnPlane(p2(), plane);
		const P3D p3_proj = projectPointOnPlane(p3(), plane);
		const P3D p4_proj = projectPointOnPlane(p4(), plane);

		if (!doLineSegmentsIntersect(p1_proj, p2_proj, p3_proj, p4_proj, cross))
		{
			// If the segments projected onto a single plane do not intersect,
			// the minimum distance is the smallest of the 4 distances from the endpoints of one segment to the other segment
			min_dist = calculateMinDistanceBetweenEnds(p1(), p2(), p3(), p4());
		}
		else
		{
			// if they intersect we should return the distance between planes
			min_dist = distanceBetweenPlanes;
		}
	}

	return min_dist;
}
//-------------------------------------------------------------------------
bool DistanceCalculator::haveCommonEndpoint(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4)
{
	return (p1 == p3 || p1 == p4 || p2 == p3 || p2 == p4);
}
//-------------------------------------------------------------------------
bool DistanceCalculator::doLineSegmentsIntersect(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4,
	const V3D& dir)
{
	const Plane3D plane(p1, dir);

	// Check if all points lie on the same plane
	if (!plane.isOn(p1) || !plane.isOn(p2) || !plane.isOn(p3) || !plane.isOn(p4))
	{
		return false;
	}

	// Check if any segments have a common endpoint
	if (haveCommonEndpoint(p1, p2, p3, p4))
	{
		return true;
	}

	// The essence of the method is as follows: if the segments p1p2 and p3p4 intersect, then for each segment, 
	// the ends of the other segment must lie on opposite sides.
	const V3D vec1 = (V3D(p3, p4)).crossProduct(V3D(p3, p1));
	const V3D vec2 = (V3D(p3, p4)).crossProduct(V3D(p3, p2));
	const V3D vec3 = (V3D(p1, p2)).crossProduct(V3D(p1, p3));
	const V3D vec4 = (V3D(p1, p2)).crossProduct(V3D(p1, p4));


	// If the vectors are collinear (same direction), the angle between them is 0°. Since the cosine of 0° equals 1, 
	// the dot product of collinear vectors is the product of their lengths -> 0.

	// If the vectors are directed in opposite directions, the angle between them is 180°.
	// Since the cosine of 180° equals -1, the dot product of oppositely directed vectors is the product of their lengths, 
	// taken with the opposite sign -> < 0.
	const double v1 = vec1.dotProduct(dir);
	const double v2 = vec2.dotProduct(dir);
	const double v3 = vec3.dotProduct(dir);
	const double v4 = vec4.dotProduct(dir);

	// So, if v1 * v2 < 0 and v3 * v4 < 0, then the segments intersect.
	return  (v1 * v2 < 0) && (v3 * v4 < 0);
}
//-------------------------------------------------------------------------
P3D DistanceCalculator::projectPointOnPlane(const P3D& point, const Plane3D& plane)
{
	// Find the distance to point
	const double dist = plane.distanceToPoint(point);

	// Create a vector to connect the planes
	V3D vecNorm = plane.normal();
	vecNorm *= dist;

	// сделаем проекцию второго отрезка на плоскость
	P3D pointProjected = point + vecNorm;
	if (!plane.isOn(pointProjected))
	{
		pointProjected = point - vecNorm;
	}

	return { pointProjected };
}
//-------------------------------------------------------------------------
double DistanceCalculator::calculateMinDistanceBetweenEnds(const P3D& p1, const P3D& p2, const P3D& p3, const P3D& p4)
{
	// The minimum distance is the smallest of the 4 distances from the endpoints of one segment to the other segment.
	const double dist1 = p1.distanceToLineSegment(p3, p4);
	const double dist2 = p2.distanceToLineSegment(p3, p4);
	const double dist3 = p3.distanceToLineSegment(p1, p2);
	const double dist4 = p4.distanceToLineSegment(p1, p2);

	return std::min(std::min(dist1, dist2), std::min(dist3, dist4));
}
