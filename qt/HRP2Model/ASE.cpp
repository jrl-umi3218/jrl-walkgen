#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <GL/gl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "Body.h"

using namespace::std;

namespace HRP2DisplayNS
{
  // Fonction qui recherche un mot dans un fichier
  char look_for(FILE* fichier, const char *str)
  {
    int i = 0;
    char c;
    do {
      fscanf(fichier, "%c", &c);
      if (c==str[i] && i < (int)(strlen(str)))
	i++;
      else
	if ((unsigned int)i == strlen(str))
	  return 1;
	else
	  i = 0;
    }
    while (!feof(fichier));
    return 0;
  }

  // Constructeur par defaut
  Body::Body(void)
  {
    int i,j;
    Mass = 0;
    for (i=0; i<4; i++)
      for (j=0; j<4; j++)
	Inertia[i][j] = 0;
    Label = 0;
  }

  // Constructeur necessitant un fichier en entree
  Body::Body(const char *name, const char *option)
  {
    m_Verbosity = 0;
    //------------------------------------------------
    // Declaration et initialisation des variables

    int numberofvertex, numberofface, numberofnormal;
    int indice = 0;

    int i,j;

    float (*point)[3],(*normal)[3];
    int (*face)[3];

    FILE * ase;

    Mass = 0;
    for (i=0; i<4; i++)
      for (j=0; j<4; j++)
	Inertia[i][j] = 0;

    // Fin declaration et initialisation des variables
    //------------------------------------------------

    //------------------------------------------------------
    // Ouverture du fichier

    ase = fopen (name, option);

    if (ase == 0)
      {
	cout << "Fichier " << name << " inexistant !"<< endl;
	return;
      }
    else
      if (m_Verbosity>3)
	cout << "Processing "<< name <<endl;

    // Fin ouverture du fichier
    //------------------------------------------------------
    
    // On recherche LABEL
    if (look_for(ase, "LABEL"))
      {
	fscanf(ase, "%d", &Label);
      }

    // Construction des Segments
    glNewList(Label, GL_COMPILE);
    glBegin(GL_TRIANGLES);

    do {
      //-------------------------------------------
      // Recherche des informations necessaires

      if (look_for(ase, "GEOMOBJECT"))
	{
	}
      else
	break;

      if (look_for(ase, "MESH_NUMVERTEX"))
	fscanf(ase, "%d", &numberofvertex);
      else
	break;

      if (look_for(ase, "MESH_NUMFACE"))
	fscanf(ase, "%d", &numberofface);
      else
	break;

      numberofnormal = numberofface * 3;

      // Fin recherche des informations necessaires
      //-------------------------------------------

      point = new float [numberofvertex][3];
      face  = new int [numberofface][3];
      normal = new float [numberofnormal][3];

      //-------------------------------------------
      // Lecture des informations necessaires

      // Lecture des vertex
      for (i=0; i<numberofvertex; i++)
	{
	  if (look_for(ase, "MESH_VERTEX "))
	    fscanf(ase, " %d %f %f %f ", &indice, &point[i][0], &point[i][1], &point[i][2]);
	  else
	    break;
	}

      // Lecture des faces
      for (i=0; i<numberofface; i++)
	{
	  if (look_for(ase, "MESH_FACE ")) 
	    fscanf(ase, " %d:", &indice);
	  else
	    break;
	  if (look_for(ase, "A:"))
	    fscanf(ase, "%d", &face[i][0]);
	  else
	    break;
	  if (look_for(ase, "B:"))
	    fscanf(ase, "%d", &face[i][1]);
	  else
	    break;
	  if (look_for(ase, "C:"))
	    fscanf(ase, "%d", &face[i][2]);
	  else
	    break;
	}

      // Lecture des normales
      for (i=0; i<numberofnormal; i++)
	{
	  if (look_for(ase, "MESH_VERTEXNORMAL"))
	    fscanf(ase, "%d %f %f %f ", &indice, &normal[i][0], &normal[i][1], &normal[i][2]);
	  else
	    break;
	}
	
      // Lecture des indices permettant la constuction des polygones
      for(i=0; i<numberofface; i++)
	{
	  glNormal3f(normal[i*3][0], normal[i*3][1], normal[i*3][2]);
	  glVertex3f(point[face[i][0]][0], point[face[i][0]][1], point[face[i][0]][2]);
			
	  glNormal3f(normal[i*3+1][0], normal[i*3+1][1], normal[i*3+1][2]);
	  glVertex3f(point[face[i][1]][0], point[face[i][1]][1], point[face[i][1]][2]);
			
	  glNormal3f(normal[i*3+2][0], normal[i*3+2][1], normal[i*3+2][2]);
	  glVertex3f(point[face[i][2]][0], point[face[i][2]][1], point[face[i][2]][2]);
	}

      // Fin lecture des informations necessaires
      //-------------------------------------------
		
      delete [] point;
      delete [] normal;
      delete [] face;

    }
    while (!feof(ase));

    glEnd();
    glEndList();

    /*if (fatal_error == 1)
      {
      glDeleteLists(Label, Label);
      }*/
  }

  Body::~Body(void)
  {
  }

};
