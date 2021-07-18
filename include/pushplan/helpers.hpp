#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <pushplan/types.hpp>
#include <pushplan/constants.hpp>

#include <smpl/time.h>

namespace clutter
{

static double GetTime()
{
	using namespace smpl;
	return to_seconds(clock::now().time_since_epoch());
}

template <typename T>
inline
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


inline
void ContToDisc(const Pointf& in, Point& out)
{
	out.x = (in.x / RES) + sgn(in.x) * 0.5;
	out.y = (in.y / RES) + sgn(in.y) * 0.5;
}

inline
void DiscToCont(const Point& in, Pointf& out)
{
	out.x = in.x * RES;
	out.y = in.y * RES;
}

inline
float dot(const Pointf& a, const Pointf& b)
{
	return a.x*b.x + a.y*b.y;
}

inline
Pointf vector(const Pointf& from, const Pointf& to)
{
	Pointf v;
	v.x = to.x - from.x;
	v.y = to.y - from.y;

	return v;
}

}

#endif // HELPERS_HPP
