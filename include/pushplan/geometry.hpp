#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <pushplan/types.hpp>
#include <pushplan/helpers.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <cmath>

namespace clutter
{

inline
bool PointInRectangle(
	const Pointf& P,
	const std::vector<Pointf>& R)
{
	Pointf AB = vector(R.at(0), R.at(1));
	Pointf AP = vector(R.at(0), P);
	Pointf BC = vector(R.at(1), R.at(2));
	Pointf BP = vector(R.at(1), P);

	float ABAP, ABAB, BCBP, BCBC;
	ABAP = dot(AB, AP);
	ABAB = dot(AB, AB);
	BCBP = dot(BC, BP);
	BCBC = dot(BC, BC);

	return ((0 <= ABAP) && (ABAP <= ABAB) && (0 <= BCBP) && (BCBP <= BCBC));
}

inline
bool LineSegCircleIntersect(
	const Pointf& C, float r,
	const Pointf& A, const Pointf& B)
{
	Pointf AB = vector(A, B);
	Pointf CA = vector(C, A);

	float a = dot(AB, AB);
	float b = 2 * dot(CA, AB);
	float c = dot(CA, CA) - r*r;
	float D = b*b - 4*a*c;

	if (D < 0) {
		// no intersection
		return false;
	}
	else
	{
		D = std::sqrt(D);
		float t1 = (-b - D)/(2*a);
		float t2 = (-b + D)/(2*a);

		if (t1 >= 0 && t1 <= 1) {
			// impale or poke
			return true;
		}

		if (t2 >= 0 && t2 <= 1) {
			// exit wound
			return true;
		}

		if (t1 < 0 && t2 > 1) {
			// completely inside
			return true;
		}

		// fall short (t1>1,t2>1), or past (t1<0,t2<0)
		return false;
	}
}

inline
bool is_between_ordered(float val, float lb, float ub)
{
	return lb <= val && val <= ub;
}

inline
bool overlaps(float min1, float max1, float min2, float max2)
{
	return is_between_ordered(min2, min1, max1) || is_between_ordered(min1, min2, max2);
}

inline
void test_SAT(const Pointf& axis, const std::vector<Pointf>& rect, float& rmin, float& rmax)
{
	rmin = std::numeric_limits<float>::max();
	rmax = std::numeric_limits<float>::lowest();
	for(int i = 0; i < rect.size(); i++)
	{
		float dot_prod = dot(rect.at(i), axis);
		if (dot_prod < rmin) {
			rmin = dot_prod;
		}
		if (dot_prod > rmax) {
			rmax = dot_prod;
		}
	}
}

inline
bool RectanglesIntersect(
	const std::vector<Pointf>& R1,
	const std::vector<Pointf>& R2)
{
	std::vector<Pointf> normals;
	float r1min, r1max, r2min, r2max;

	normals.push_back(vector(R1.at(0), R1.at(1)));
	normals.push_back(vector(R1.at(1), R1.at(2)));
	for (int i = 0; i < normals.size(); ++i)
	{
		test_SAT(normals.at(i), R1, r1min, r1max);
		test_SAT(normals.at(i), R2, r2min, r2max);

		if (!overlaps(r1min, r1max, r2min, r2max)) {
			return false;
		}
	}
	normals.clear();

	normals.push_back(vector(R2.at(0), R2.at(1)));
	normals.push_back(vector(R2.at(1), R2.at(2)));
	for (int i = 0; i < normals.size(); ++i)
	{
		test_SAT(normals.at(i), R1, r1min, r1max);
		test_SAT(normals.at(i), R2, r2min, r2max);

		if (!overlaps(r1min, r1max, r2min, r2max)) {
			return false;
		}
	}
	normals.clear();

	return true;
}

inline
void MakeObjectRectangle(
	const Object& o, std::vector<Pointf>& rect)
{
	rect.clear();
	rect.emplace_back(o.o_x - o.x_size, o.o_y - o.y_size);
	rect.emplace_back(o.o_x + o.x_size, o.o_y - o.y_size);
	rect.emplace_back(o.o_x + o.x_size, o.o_y + o.y_size);
	rect.emplace_back(o.o_x - o.x_size, o.o_y + o.y_size);
}

inline
void GetRectObjAtPt(
	const Pointf& p,
	const Object& o,
	std::vector<Pointf>& rect)
{
	Eigen::Matrix2d rot; // 2D rotation matrix for (o_yaw)
	rot(0, 0) = std::cos(o.o_yaw);
	rot(0, 1) = -std::sin(o.o_yaw);
	rot(1, 0) = std::sin(o.o_yaw);
	rot(1, 1) = std::cos(o.o_yaw);

	MakeObjectRectangle(o, rect); // axis-aligned at (o_x, o_y)
	Eigen::MatrixXd R(2, 4); // axis-aligned at (origin)
	for (int i = 0; i < (int)rect.size(); ++i)
	{
		R(0, i) = rect.at(i).x - o.o_x;
		R(1, i) = rect.at(i).y - o.o_y;
	}
	R = rot * R; // rotate at (origin) by rot(o_yaw)
	// R = rot2 * R; // can rotate again by some rot2(theta) matrix

	// translate rotated rectangle at (origin) to (p)
	for (int i = 0; i < (int)rect.size(); ++i)
	{
		rect.at(i).x = R(0, i) + p.x;
		rect.at(i).y = R(1, i) + p.y;
	}
}

inline
float EuclideanDist(const Pointf& p1, const Pointf& p2)
{
	return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

} // namespace clutter


#endif // GEOMETRY_HPP
