#include "Matrice.h"

#pragma once

CMatrice Rodrigues(CVecteur w, double dt);

CMatrice hat(CVecteur c);

class HRP2
{
protected:
	int mother[31], sister[31], child[31];
	/* a = vecteur de rotation,
	   b = vecteur de translation,
	   c = centre de masse,
	   p = position,
	   v0 = vitesse,
	   dv = derivee de la vitesse,
	   w = vitesse angulaire,
	   dw = derivee de w,
	   w_c = com dans le repere de base,
	   sv, sw = spatial velocity,
	   cv, cw = cross velocity term,
	   pph, ppb = v x Iv */
    CVecteur a[31], b[31] ,c[31],
			 v0[31], p[31], dv[31], w[31], dw[31],
			 sw[31], sv[31], cv[31], cw[31],
			 w_c[31], hhv[31], hhw[31], pph[31], ppb[31];
	/* m = masse,
	   q, dq, ddq = coordonnees articulaires, vitesse et acceleration, 
	   u = couple */
    double m[31], q[31], dq[31], ddq[31], u[31], dd[31], uu[31];
	/* I = inertie,
	   R = orientation,
	   w_I = inertie dans le repere de base,
	   Ivv, Iwv, Iww = inerties (p.198, 6.24) */
    CMatrice I[31], R[31], w_I[31], Ivv[31], Iwv[31], Iww[31];

public:
	HRP2(void);
	double acces_q(int i)
	{
		return q[i];
	}
	CVecteur acces_p(int i)
	{
		return p[i];
	}

	double acces_R(int i, int j)
	{
		return R[0].get(j,i);
	}

	void load_data(void);

	CVecteur recursive(void);
	void recursive1(int j);
	void recursive2(int j);
	void recursive3(int j);
	void ForwardKinematics(int j);

	// Integration d'Euler
	void Euler(double couple1, double couple2, double couple3, double couple4, double couple5, double couple6,
			   double couple7, double couple8, double couple9, double couple10, double couple11, double couple12,
			   double couple13, double couple14, double couple15, double couple16,
			   double couple17, double couple18, double couple19, double couple20, double couple21, double couple22, double couple23,
			   double couple24, double couple25, double couple26, double couple27, double couple28, double couple29, double couple30);
	void IntegrateEuler(int j);

	// Integration de Runge-Kutta
	void RungeKutta(double couple1, double couple2, double couple3, double couple4, double couple5, double couple6,
					double couple7, double couple8, double couple9, double couple10, double couple11, double couple12,
					double couple13, double couple14, double couple15, double couple16,
					double couple17, double couple18, double couple19, double couple20, double couple21, double couple22, double couple23,
					double couple24, double couple25, double couple26, double couple27, double couple28, double couple29, double couple30);
	CVecteur UpdateState(double dt, CVecteur pl, CMatrice Rl, CVecteur q0, CVecteur dq0, CVecteur dq1, CVecteur ddq1);

	void SE3exp(int j, double dt, CVecteur *p2, CMatrice *R2);

	void ForwardDynamics(void);

	~HRP2(void);
};
