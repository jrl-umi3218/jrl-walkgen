#ifndef _HRP2DISPLAYNS_BODY_H_
#define _HRP2DISPLAYNS_BODY_H_

namespace HRP2DisplayNS
{

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
};
#endif
