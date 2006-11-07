/* @doc Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   This version has been modified to get rid of all the OpenGL code.

   CVS Information:
   $Id: MultiBody.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/MultiBody.h,v $
   $Log: MultiBody.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Adrien Escande, Abderrahmane Kheddar, Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MULTIBODY_H
#define MULTIBODY_H

#define INTERFACE

#include "linalg.h"
#include "fileReader.h"
#include "Body.h"
#include <vector>
#include <algorithm>	//pour utiliser la fonction find
#include <iostream>
#include <string>

using namespace std;

#define FREE_JOINT		   -1
#define FIX_JOINT			0
#define REVOLUTE_JOINT		1
#define PRISMATIC_JOINT		2

#define ROTATION		0x1000
#define TRANSLATION		0x1001
#define ROTATION_LIBRE	0x1002

#define PROFONDEUR_MAX	30

//static int cptLiaison= 0;	//pour labeliser les liaisons


namespace PatternGeneratorJRL
{

/**
   Structure transfo :
   Definit une transformation de passage d'un corps a un autre
   Types supportes : 
 	- Rotation autour d'un axe axe de l'angle quantite (type = ROTATION)
 	- Translation d'un vecteur quantite*axe	(type = TRANSLATION)
 	- Rotation de matrice homogene *rotation (type = ROTATION_LIBRE)
*/
class transfo {
  public: 

  int type;
  vector3d axe;	//pour les liens a 1DoF
  float quantite;

  transfo(int ltype, vector3d laxe, 
	  float lquantite, float *lrotation=0);
  transfo(const transfo &r); 
  ~transfo();
  transfo & operator=(const transfo &r);
  float rotation(unsigned r) const;
  float * rotation() const;

  protected:
    float *m_rotation; //pour les rotations libre; 
};

/**
   Structure internalLink :
   Definit une liaison entre deux corps
   Types supportes :
   - Liaison libre (6 DoFs) (type = FREE_JOINT)
   - Liaison fixe (0 DoF) (type = FIX_JOINT)
   - Liaison prismatique (1 DoF) (type = PRISMATIC_JOINT)
   - Liaison pivot (1 DoF) (type = REVOLUTE_JOINT)
   
   En amont des transformations de listeTransformation, se font une
   translation (translationStatique) et une rotation (d'angle angleRotationStatique
   et d'axe axeRotationStatique)
 */
struct internalLink {
  int label;
  int type;
  vector3d translationStatique;
  vector3d axeRotationStatique;
  double angleRotationStatique;
  vector<transfo> listeTransformation;
  int indexCorps1;
  int indexCorps2;
  string Name;
  int IDinVRML;
};

/**
   Structure appariement :
   - corps designe la place du corps lie a celui auquel appartient l'instance
   de la structure par cette instance
   Cette donnee n'est pas veritablement utile puisqu'elle correspond a l'un
   des deux membres indexCorps1 ou indexCorps2 de l'instance d'internalLink
   designee par liaison
   - liaison indique la place de la liaison correspondante
   cf la description 
   
   L'operateur d'egalite est utilise avec la fonction find de la librairie
   <algorithm>
*/
struct appariement {
  int corps;
  int liaison;
};

bool operator==(const appariement a1, const appariement a2);


/**
   Classe Multibody :
   Cette classe implemente un graphe non oriente dont les noeuds sont des corps
   (Body) et les arcs des liaisons (internalLink)
   
   La description se fait comme suit :
   On dispose d'une liste des noeud, listeCorps, et d'une liste d'arcs,
   listeLiaisons. Les arcs partant du ieme noeud de listeCorps se trouvent dans
   le vecteur liaisons[i], encapsule dans une instance d'appariement dont le
   membre corps designe le second noeud a l'extremite de l'arc.
   
   La suppresion d'un noeud n'entraine pas la suppression effective du noeud et
   des arcs correspondants dans les vecteurs concernes, mais seulement le
   marquage des instances en questions comme "supprimees" par la mise a -1 de 
   leur label.
   
   Les methodes necessitant un parcours du graphe se base sur l'hypothese qu'il
   ne contienne pas de cycle.   
*/
class MultiBody
{
 protected:

  /// Robot's mass.
  double masse;
  /// CoM of the robot
  vector3d positionCoMPondere;			//CoM*masse
  /// List of links.
  vector<internalLink> listeLiaisons;
  /// List Of Bodies.
  vector<Body> listeCorps;
  /// Links with other bodies.
  vector<vector<appariement> > liaisons;
  /// List of Transformation.
  vector<float*> listeVariableTransformation;
  /// Counter for links.
  int cptLiaison;

public:

  /// Constructor 
  MultiBody(void);
  /// Destructor
  virtual ~MultiBody(void);
  
  // Returns the masse.
  double getMasse();

  /// Adds a body.
  void ajouterCorps(Body &b);

  /// Adds a link.
  void ajouterLiaison(Body &corps1, Body &corps2, internalLink l);

  /// Adds a fixed link.
  void ajouterLiaisonFixe(Body &corps1, Body &corps2, vector3d translationStat = 0, vector3d axeRotationStat = 0, 
			  double angleRotationStat = 0);
  /// Adds a link with rotation.
  void ajouterLiaisonRotation(Body &corps1, Body &corps2, vector3d axe = 0, vector3d translationStat = 0, 
			      vector3d axeRotationStat = 0, double angleRotationStat = 0);
  /// Remove a link between body corps1 and body corps.
  void supprimerLiaisonEntre(Body &corps1, Body &corps2);

  /// Remove a link by index.
  void supprimerLiaison(int index);

  /// Remove a link by label.
  void supprimerLiaisonLabel(int label);

  /// Remove the body b.
  void supprimerCorps(Body &b);

  /// Remove a body by index.
  void supprimerCorps(int index);

  /// Remove body by label.
  void supprimerCorpsLabel(int label);

  /// Inverse the link i.
  void inverserLiaison(int i);

  /// Returns the CoM position. 
  vector3d getPositionCoM(void);

  //Construction a partir d'un fichier VRML
  virtual void parserVRML(string path, string nom, const char* option);
  
  //Lien avec l'interface
  vector<float*> getListeVariableTransformation(void);

  /// Display bodies
  void afficherCorps(void);
  /// Display links
  void afficherLiaisons(void);
  /// Display everything.
  void afficher(void);

  /// Returns the number of links.
  int NbOfLinks();
};
};

#endif
