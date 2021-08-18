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
	const State& P,
	const std::vector<State>& R)
{
	State AB = vector2D(R.at(0), R.at(1));
	State AP = vector2D(R.at(0), P);
	State BC = vector2D(R.at(1), R.at(2));
	State BP = vector2D(R.at(1), P);

	double ABAP, ABAB, BCBP, BCBC;
	ABAP = dot(AB, AP);
	ABAB = dot(AB, AB);
	BCBP = dot(BC, BP);
	BCBC = dot(BC, BC);

	return ((0 <= ABAP) && (ABAP <= ABAB) && (0 <= BCBP) && (BCBP <= BCBC));
}

inline
bool LineSegCircleIntersect(
	const State& C, double r,
	const State& A, const State& B)
{
	State AB = vector2D(A, B);
	State CA = vector2D(C, A);

	double a = dot(AB, AB);
	double b = 2 * dot(CA, AB);
	double c = dot(CA, CA) - r*r;
	double D = b*b - 4*a*c;

	if (D < 0) {
		// no intersection
		return false;
	}
	else
	{
		D = std::sqrt(D);
		double t1 = (-b - D)/(2*a);
		double t2 = (-b + D)/(2*a);

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
bool is_between_ordered(double val, double lb, double ub)
{
	return lb <= val && val <= ub;
}

inline
bool overlaps(double min1, double max1, double min2, double max2)
{
	return is_between_ordered(min2, min1, max1) || is_between_ordered(min1, min2, max2);
}

inline
void test_SAT(const State& axis, const std::vector<State>& rect, double& rmin, double& rmax)
{
	rmin = std::numeric_limits<double>::max();
	rmax = std::numeric_limits<double>::lowest();
	for(int i = 0; i < rect.size(); i++)
	{
		double dot_prod = dot(rect.at(i), axis);
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
	const std::vector<State>& R1,
	const std::vector<State>& R2)
{
	std::vector<State> normals;
	double r1min, r1max, r2min, r2max;

	normals.push_back(vector2D(R1.at(0), R1.at(1)));
	normals.push_back(vector2D(R1.at(1), R1.at(2)));
	for (int i = 0; i < normals.size(); ++i)
	{
		test_SAT(normals.at(i), R1, r1min, r1max);
		test_SAT(normals.at(i), R2, r2min, r2max);

		if (!overlaps(r1min, r1max, r2min, r2max)) {
			return false;
		}
	}
	normals.clear();

	normals.push_back(vector2D(R2.at(0), R2.at(1)));
	normals.push_back(vector2D(R2.at(1), R2.at(2)));
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
	const Object& o, std::vector<State>& rect)
{
	rect.clear();
	rect.push_back({o.o_x - o.x_size, o.o_y - o.y_size});
	rect.push_back({o.o_x + o.x_size, o.o_y - o.y_size});
	rect.push_back({o.o_x + o.x_size, o.o_y + o.y_size});
	rect.push_back({o.o_x - o.x_size, o.o_y + o.y_size});
}

inline
void GetRectObjAtPt(
	const State& p,
	const Object& o,
	std::vector<State>& rect)
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
		R(0, i) = rect.at(i).at(0) - o.o_x;
		R(1, i) = rect.at(i).at(1) - o.o_y;
	}
	R = rot * R; // rotate at (origin) by rot(o_yaw)
	// R = rot2 * R; // can rotate again by some rot2(theta) matrix

	// translate rotated rectangle at (origin) to (p)
	for (int i = 0; i < (int)rect.size(); ++i)
	{
		rect.at(i).at(0) = R(0, i) + p.at(0);
		rect.at(i).at(1) = R(1, i) + p.at(1);
	}
}

inline
double EuclideanDist(const State& p1, const State& p2)
{
	assert(p1.size() == p2.size());
	double val = 0.0;
	for (size_t i = 0; i < p1.size(); ++i) {
		val += std::pow(p1.at(i) - p2.at(i), 2);
	}
	return std::sqrt(val);
}

inline
double PtDistFromLine(
	const State& p,
	const State& A, const State& B)
{
	double d = std::fabs((B.at(0) - A.at(0))*(A.at(1) - p.at(1)) - (A.at(0) - p.at(0))*(B.at(1) - A.at(1)));
	d /= EuclideanDist(B, A);
	return d;
}

inline
void ArmRectObj(
	const State& F1, const State& F2,
	const double& b,
	Object& o)
{
	// double e = EuclideanDist(F1, F2)/2.0;
	// double a = b/std::sqrt(1 - (e*e));

	o.o_x = (F1.at(0) + F2.at(0))/2.0;
	o.o_y = (F1.at(1) + F2.at(1))/2.0;
	o.o_yaw = std::atan2(F2.at(1) - F1.at(1), F2.at(0) - F1.at(0));
	o.x_size = EuclideanDist(F1, F2)/2.0;
	o.y_size = b;
}

inline
void LineLineIntersect(
	const State& l1a, const State& l1b,
	const State& l2a, const State& l2b,
	State& p)
{
	p.clear();
	p.resize(2, 0.0);

	double D = ((l1a.at(0) - l1b.at(0)) * (l2a.at(1) - l2b.at(1))) -
					((l1a.at(1) - l1b.at(1)) * (l2a.at(0) - l2b.at(0)));

	p.at(0) = ((l1a.at(0)*l1b.at(1) - l1a.at(1)*l1b.at(0)) * (l2a.at(0) - l2b.at(0))) -
				((l1a.at(0) - l1b.at(0)) * (l2a.at(0)*l2b.at(1) - l2a.at(1)*l2b.at(0)));
	p.at(1) = ((l1a.at(0)*l1b.at(1) - l1a.at(1)*l1b.at(0)) * (l2a.at(1) - l2b.at(1))) -
				((l1a.at(1) - l1b.at(1)) * (l2a.at(0)*l2b.at(1) - l2a.at(1)*l2b.at(0)));

	p.at(0) /= D;
	p.at(1) /= D;
}

} // namespace clutter


#endif // GEOMETRY_HPP
