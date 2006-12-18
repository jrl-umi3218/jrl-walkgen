#ifndef MULTIBODY_H
#define MULTIBODY_H

#define INTERFACE

#include "linalg.h"
#include "fileReader.h"
#include "parserASE.h"
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

#define PROFONDEUR_MAX	50

static int cptLiaison= 0;	//pour labeliser les liaisons



/***********************************************************************
 *Structure transfo :
 *Definit une transformation de passage d'un corps a un autre
 *Types supportes : 
 *	- Rotation autour d'un axe axe de l'angle quantite (type = ROTATION)
 *	- Translation d'un vecteur quantite*axe	(type = TRANSLATION)
 *	- Rotation de matrice homogene *rotation (type = ROTATION_LIBRE)
 ***********************************************************************/
struct transfo {
	int type;
	vector3d axe;	//pour les liens a 1DoF
	float quantite;
	float *rotation; //pour les rotations libre; 
};

/*********************************************************************************
 *Structure internalLink :
 *Definit une liaison entre deux corps
 *Types supportes :
 *	- Liaison libre (6 DoFs) (type = FREE_JOINT)
 *	- Liaison fixe (0 DoF) (type = FIX_JOINT)
 *	- Liaison prismatique (1 DoF) (type = PRISMATIC_JOINT)
 *	- Liaison pivot (1 DoF) (type = REVOLUTE_JOINT)
 *
 *En amont des transformations de listeTransformation, se font une
 *translation (translationStatique) et une rotation (d'angle angleRotationStatique
 *et d'axe axeRotationStatique)
 *********************************************************************************/
struct internalLink {
	int label;
	int type;
	vector3d translationStatique;
	vector3d axeRotationStatique;
	double angleRotationStatique;
	vector<transfo> listeTransformation;
	int indexCorps1;
	int indexCorps2;
};

/****************************************************************************
 *Structure appariement :
 * - corps designe la place du corps lie a celui auquel appartient l'instance
 *de la structure par cette instance
 *Cette donnee n'est pas veritablement utile puisqu'elle correspond a l'un
 *des deux membres indexCorps1 ou indexCorps2 de l'instance d'internalLink
 *designee par liaison
 * - liaison indique la place de la liaison correspondante
 *cf la description 
 *
 *L'operateur d'egalite est utilise avec la fonction find de la librairie
 *<algorithm>
 ****************************************************************************/
struct appariement {
	int corps;
	int liaison;
};

bool operator==(const appariement a1, const appariement a2);


/****************************************************************************
*Classe Multibody :
*Cette classe implemente un graphe non oriente dont les noeuds sont des corps
*(Body) et les arcs des liaisons (internalLink)
*
*La description se fait comme suit :
*On dispose d'une liste des noeud, listeCorps, et d'une liste d'arcs,
*listeLiaisons. Les arcs partant du ieme noeud de listeCorps se trouvent dans
*le vecteur liaisons[i], encapsule dans une instance d'appariement dont le
*membre corps designe le second noeud a l'extremite de l'arc.
*
*La suppresion d'un noeud n'entraine pas la suppression effective du noeud et
*des arcs correspondants dans les vecteurs concernes, mais seulement le
*marquage des instances en questions comme "supprimees" par la mise a -1 de 
*leur label.
*
*Les methodes necessitant un parcours du graphe se base sur l'hypothese qu'il
*ne contienne pas de cycle.
*
****************************************************************************/
class MultiBody
{
 public:
	double masse;
	vector3d positionCoMPondere;			//CoM*masse
	vector<internalLink> listeLiaisons;
	vector<vector<appariement> > liaisons;
	vector<float*> listeVariableTransformation;


public:
	MultiBody(void);
	~MultiBody(void);

	void ajouterCorps(Body &b);
	void ajouterLiaison(Body &corps1, Body &corps2, internalLink l);
	void ajouterLiaisonFixe(Body &corps1, Body &corps2, vector3d translationStat = 0, vector3d axeRotationStat = 0, 
							double angleRotationStat = 0);
	void ajouterLiaisonRotation(Body &corps1, Body &corps2, vector3d axe = 0, vector3d translationStat = 0, 
								vector3d axeRotationStat = 0, double angleRotationStat = 0);
	void supprimerLiaisonEntre(Body &corps1, Body &corps2);
	void supprimerLiaison(int index);
	void supprimerLiaisonLabel(int label);
	void supprimerCorps(Body &b);
	void supprimerCorps(int index);
	void supprimerCorpsLabel(int label);
	void inverserLiaison(int i);
	void changerCorpsInitial(int nouveauCorps);
	void calculerMatriceTransformationEntre(int corps1, int corps2, float* matrice);
	vector<int> trouverCheminEntre(int corps1, int corps2);
	void trouverCheminEntreAux(int corpsCourant, int corpsVise, int liaisonDeProvenance, vector<int> &chemin);

	//deux fonctions non encore implementees dont le but est de retirer des vecteurs correspondant
	//les corps et liaisons marques comme effaces, tout en remettant a jour les indices
	void rafraichirListeCorps(void);		
	void rafraichirListeLiaisons(void);

	vector3d getPositionCoM(void);
	vector3d getPositionCoM(double *mat);

	//Construction a partir d'un fichier VRML
	void parserVRML(string path, string nom, const char* option);

	//Dessin
	void dessiner(int corpsCourant, int liaisonDeProvenance, float alpha);
	inline void empilerTransformationsLiaisonDirecte(int liaison);
	inline void empilerTransformationsLiaisonInverse(int liaison);

	//Lien avec l'interface
	vector<float*> getListeVariableTransformation(void);
	vector<int*> getListeDraw(void);

	void afficher(void);
	void afficherLiaisons(void);

	vector<Body> listeCorps;
};


#endif
