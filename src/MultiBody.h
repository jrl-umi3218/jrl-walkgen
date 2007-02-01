/* Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   This version has been modified to get rid of all the OpenGL code.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


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

#ifndef __PG_MULTIBODY_H
#define __PG_MULTIBODY_H

#define INTERFACE
#include "Body.h"
#include "fileReader.h"
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

  /** @ingroup forwardynamics
       Define a transformation from a body to another
      Supported type:
      - Rotation around an axis with a quantity (type = ROTATION)
      - Translation of a vector : quantite*axe	(type = TRANSLATION)
      - Rotation through a homogeneous matrix : *rotation (type = ROTATION_LIBRE)
  */
  class transfo 
  {
  public: 
    
    /// Type of the transformation
    int type;
    /// Axis of the transformation
    MAL_S3_VECTOR(,double) axe;	//pour les liens a 1DoF
    float quantite;
    
    transfo(int ltype, MAL_S3_VECTOR(,double) laxe, 
	    float lquantite, float *lrotation=0);
    transfo(const transfo &r); 
    ~transfo();
    transfo & operator=(const transfo &r);
    float rotation(unsigned r) const;
    float * rotation() const;
    
  protected:
    float *m_rotation; //pour les rotations libre; 
  };

  /** @ingroup forwardynamics
       internallink structure:
      Define a link between two bodys:
      Supported types:
      - Free joint (6 DoFs) (type = FREE_JOINT)
      - Fixed joint (0 DoF) (type = FIX_JOINT)
      - Prismatic joint (1 DoF) (type = PRISMATIC_JOINT)
      - Revolute joint (1 DoF) (type = REVOLUTE_JOINT)
      
      The sequence for the transformation is
      - translation (translationStatique
      - rotation (angleRotationStatique)
      - the list of transformation (listeTransformation).
  */
  struct internalLink {
    int label;
    int type;
    MAL_S3_VECTOR(,double) translationStatique;
    MAL_S3_VECTOR(,double) axeRotationStatique;
    double angleRotationStatique;
    vector<transfo> listeTransformation;
    int indexCorps1;
    int indexCorps2;
    string Name;
    int IDinVRML;
  };

  /** @ingroup forwardynamics
      
  Binding structure:
  - \a corps is the body to which this structure belongs. This should be equal
  to one of the field \a indexCorps1 or \a indexCorps of the internalLink's instance indicated 
  by the field \a liaison.
  - \a liaison is the corresponding link.   

  The equality operator is used with the find function of <algorithm>.
  */
  struct appariement {
    int corps;
    int liaison;
  };

  bool operator==(const appariement a1, const appariement a2);

  /** @ingroup forwardynamics
       
      This class implements a non-oriented graph for which the nodes are 
      bodies (Body) and the edges are joints (internalLink).
      
      The description is:
      There is a list of nodes (listeCorps) and a list of edges (listeLiaisons).
      The edges starting from an i-th node of listeCorps are stored in vector
      liaisons[i], and store an instance of appariement for which the body 
      Body is the second node of the edge.

      
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
    
    /*!  Robot's mass. */
    double masse;
    /*!  CoM of the robot */
    MAL_S3_VECTOR(,double) positionCoMPondere;			//CoM*masse
    /*!  List of links. */
    vector<internalLink> listeLiaisons;
    /*!  List Of Bodies. */
    vector<Body> listeCorps;
    /*!  Links with other bodies. */
    vector<vector<appariement> > liaisons;
    /*!  List of Transformation. */
    vector<float*> listeVariableTransformation;
    /*!  Counter for links. */
    int cptLiaison;

  public:

    /*!  Constructor */
    MultiBody(void);
    /*!  Destructor */
    virtual ~MultiBody(void);
    
    /*! Returns the masse. */
    double getMasse();

    /*!  Adds a body. */
    void ajouterCorps(Body &b);

    /*!  Adds a link. */
    void ajouterLiaison(Body &corps1, Body &corps2, internalLink l);

    /*!  Adds a fixed link. */
    void ajouterLiaisonFixe(Body &corps1, Body &corps2, 
			    MAL_S3_VECTOR(,double) translationStat, 
			    MAL_S3_VECTOR(,double) axeRotationStat, 
			    double angleRotationStat = 0);
    /*!  Adds a link with rotation. */
    void ajouterLiaisonRotation(Body &corps1, Body &corps2, 
				MAL_S3_VECTOR(,double) axe , 
				MAL_S3_VECTOR(,double) translationStat, 
				MAL_S3_VECTOR(,double) axeRotationStat, 
				double angleRotationStat = 0);

    /*!  Remove a link between body corps1 and body corps. */
    void supprimerLiaisonEntre(Body &corps1, Body &corps2);

    /*!  Remove a link by index. */
    void supprimerLiaison(int index);

    /*!  Remove a link by label. */
    void supprimerLiaisonLabel(int label);

    /*!  Remove the body b. */
    void supprimerCorps(Body &b);

    /*!  Remove a body by index. */
    void supprimerCorps(int index);

    /*!  Remove body by label. */
    void supprimerCorpsLabel(int label);

    /*!  Inverse the link i. */
    void inverserLiaison(int i);

    /*!  Returns the CoM position. */ 
    MAL_S3_VECTOR(,double) getPositionCoM(void);

    //Construction a partir d'un fichier VRML
    virtual void parserVRML(string path, string nom, const char* option);
  
    /*! Link with the interface */
    vector<float*> getListeVariableTransformation(void);

    /*!  Display bodies */
    void afficherCorps(void);
    /*!  Display links */
    void afficherLiaisons(void);
    /*!  Display everything.  */
    void afficher(void);

    /*!  Returns the number of links. */
    int NbOfLinks();
  };
};

#endif
