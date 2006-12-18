#ifndef _BODY_H_
#define _BODY_H_

class Body
{
	double Mass;
	double Inertia[4][4];
	int Label;
	int m_Verbosity;
public:
	Body(void);
	Body(const char *, const char *);
	~Body(void);
};
#endif
