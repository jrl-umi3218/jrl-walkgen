#ifndef _PARSER_ASE_H_
#define  _PARSER_ASE_H_

#include "fileReader.h"
#include "glut.h"
#include "vector3.h"
#include <iostream>
#include <string>

using namespace std;

#define LISTE_VIDE 0

namespace HRP2DisplayNS
{
struct geometrieCorps {
	int nb;
	int *liste;
	vector3<float> *couleur;
};

int parserASE(const char* nom, const char *option);


geometrieCorps lireBOD(string path, string nom, const char *option);
};
#endif /*_PARSER_ASE_H_ */
