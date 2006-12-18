#ifndef _FILE_READER_H
#define _FILE_READER_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
using namespace std;

#define CROCHET_OUVRANT		0
#define CROCHET_FERMANT		1
#define CHILDREN		2
#define JOINT			3
#define FR_NAME                    4

#define AXE_X				0
#define AXE_Y				1
#define AXE_Z				2
#define JOINT_TRANSLATION	3
#define JOINT_ROTATION		4

// Fonction qui recherche un mot dans un fichier
char look_for(FILE* fichier, const char *str);

bool immediatlyAppears(FILE* fichier, const char *str) ;

int nextKeyWord(FILE* fich);

int nextJointKeyWord(FILE* fichier);

int typeOfJoint(FILE* fichier);

#endif /* _FILE_READER_H */
