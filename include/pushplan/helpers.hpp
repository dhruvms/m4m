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
void ContToDisc(const State& in, Coord& out)
{
	out.clear();
	out.resize(in.size(), 0);

	out.at(0) = (in.at(0) / RES) + sgn(in.at(0)) * 0.5;
	out.at(1) = (in.at(1) / RES) + sgn(in.at(1)) * 0.5;
}

inline
void DiscToCont(const Coord& in, State& out)
{
	out.clear();
	out.resize(in.size(), 0.0);

	out.at(0) = in.at(0) * RES;
	out.at(1) = in.at(1) * RES;
}

inline
double dot(const State& a, const State& b)
{
	assert(a.size() == b.size());
	double val = 0.0;
	for (size_t i = 0; i < a.size(); ++i) {
		val += a.at(i) * b.at(i);
	}
	if (std::abs(val) < 1e-6) {
		val = 0.0;
	}
	return val;
}

inline
State vector2D(const State& from, const State& to)
{
	assert(from.size() == to.size());
	State v(from.size(), 0.0);
	v.at(0) = to.at(0) - from.at(0);
	v.at(1) = to.at(1) - from.at(1);

	return v;
}

template <class T>
inline
static auto ParseMapFromString(const std::string& s)
	-> std::unordered_map<std::string, T>
{
	std::unordered_map<std::string, T> map;
	std::istringstream ss(s);
	std::string key;
	T value;
	while (ss >> key >> value) {
		map.insert(std::make_pair(key, value));
	}
	return map;
}

}

#endif // HELPERS_HPP
