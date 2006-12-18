#pragma once

#include "Body.h"
#include "../Utils/linalg.h"

#define FIX_JOINT			0
#define REVOLUTE_JOINT		1
#define PRISMATIC_JOINT		2


class Body;

static int compteurLiaison= 0;

class Link
{
	int label;
	int type;
	Body *corps;
	vector3d axeRotation;
	double angleRotation;
	vector3d axeTranslation;
	double quantiteTranslation;

public:
	Link(void);
	Link(int type, vector3d& v);
	~Link(void);

	bool lierA(Body& b, int extremite=-1);
	bool lier(Body& a, Body& b);
	bool delier(Body& b);

};
