#ifndef CONFLICTS_HPP
#define CONFLICTS_HPP

#include <pushplan/types.hpp>

#include <memory>
#include <list>

namespace clutter
{

struct Constraint
{
	int m_me, m_other, m_time;
	bool m_on;
	LatticeState m_q;
};

struct Conflict
{
	int m_a1, m_a2;
	int m_t;
	bool m_on;
	LatticeState m_q1, m_q2;
	std::shared_ptr<Constraint> m_c1, m_c2;

	void InitConflict(int a1, int a2, int t, LatticeState q1, LatticeState q2, bool robot=false)
	{
		m_a1 = a1;
		m_a2 = a2;
		m_t = t;
		m_q1 = q1;
		m_q2 = q2;

		m_c1 = std::make_shared<Constraint>();
		m_c1->m_me = m_a1;
		m_c1->m_other = m_a2;
		m_c1->m_time = m_t;
		m_c1->m_q = q2;

		m_c2 = std::make_shared<Constraint>();
		m_c2->m_me = m_a2;
		m_c2->m_other = robot ? m_a2 : m_a1;
		m_c2->m_time = m_t;
		m_c2->m_q = robot ? q2 : q1;
	}

	int at(std::size_t i) const {
        if (i == 0) {
        	return m_a1;
        }
        else if (i == 1) {
        	return m_a2;
        }
        else {
        	assert(false);
        	return -1;
        }
    }
};

inline
void VecConstraint(const Constraint& constraint, std::vector<double>& c_vec)
{
	c_vec.clear();
	c_vec.push_back(constraint.m_time);
	c_vec.push_back(constraint.m_other);
	c_vec.insert(c_vec.end(), constraint.m_q.state.begin(), constraint.m_q.state.end());
}

inline
void VecConstraints(const std::list<std::shared_ptr<Constraint> >& constraints, std::vector<std::vector<double> >& c_vecs)
{
	c_vecs.clear();
	std::vector<double> c_vec;
	for (const auto& c: constraints)
	{
		VecConstraint(*(c.get()), c_vec);
		c_vecs.push_back(c_vec);
	}
}

} // namespace clutter


#endif // CONFLICTS_HPP
