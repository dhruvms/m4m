#ifndef CONFLICTS_HPP
#define CONFLICTS_HPP

#include <pushplan/types.hpp>

#include <memory>

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
};

} // namespace clutter


#endif // CONFLICTS_HPP
