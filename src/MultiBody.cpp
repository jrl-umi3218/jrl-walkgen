/* Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.
   OS (10/01/2007): Put the abstract layer for small matrix library.
   
   Copyright (c) 2005-2006, 
   Adrien Escande,
   Abderrahmane Kheddar,  
   Olivier Stasse,
   Ramzi Sellouati
   
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
#include "MultiBody.h"

using namespace PatternGeneratorJRL;

// surcharge operateur
bool PatternGeneratorJRL::operator==(const PatternGeneratorJRL::appariement a1, 
				     const PatternGeneratorJRL::appariement a2) 
{
  return (a1.corps==a2.corps);
}


transfo::transfo(int ltype, MAL_S3_VECTOR(,double) laxe, 
		 float lquantite, float *lrotation):
  type(ltype),
  axe(laxe),
  quantite(lquantite),
  m_rotation(lrotation)
{
}

transfo::transfo(const transfo &r)
{
  type = r.type;
  axe = r.axe;
  quantite=r.quantite;
  float *rrot;
  if ((rrot=r.rotation())!=0)
    {
      m_rotation = new float[16];
      for(int i=0;i<16;i++)
	m_rotation[i] = rrot[i] ;
    }
  else
    m_rotation = 0;

}

transfo::~transfo() 
{
  if (m_rotation!=0) 
    delete [] m_rotation;
}
float transfo::rotation(unsigned r)  const 
{
  if (m_rotation!=0)
    return m_rotation[r];
  return 0.0;
}

float * transfo::rotation() const
{
  return m_rotation;
}

transfo & transfo::operator=(const transfo & r) 
{
  type = r.type;
  axe = r.axe;
  quantite=r.quantite;
  float *rrot;
  if ((rrot=r.rotation())!=0)
    {
      m_rotation = new float[16];
      for(int i=0;i<16;i++)
	m_rotation[i] = rrot[i] ;
    }
  else
    m_rotation = 0;

  return *this;
};


// Caution: This operator is specific to OpenGL matrices: it transposes the
// matrix before multiplication.
MAL_S3_VECTOR(,double) operator * (double* m, MAL_S3_VECTOR(,double) v) 
{
  MAL_S3_VECTOR(,double) result;

  result[0] = m[0]*v[0] + m[4]*v[1] +  m[8]*v[2]+ m[12];
  result[1] = m[1]*v[0] + m[5]*v[1] +  m[9]*v[2]+ m[13];
  result[2] = m[2]*v[0] + m[6]*v[1] + m[10]*v[2]+ m[14];
  return result;
}


//Pour matrice OpenGL
void Matrix2AxeAngle(float R[16],double Axis[3], double & Angle)
{
  double q[4];
  double sum_x, sum_y, sum_z, sum_w, sum_max, S;
  
  sum_w = 1 + R[0] + R[5] + R[10];
  sum_x = 1 + R[0] - R[5] - R[10];
  sum_y = 1 + R[5] - R[0] - R[10];
  sum_z = 1 + R[10] - R[0] - R[5];
  sum_max = max(max(sum_w,sum_x), max(sum_y, sum_z));
  
  if (sum_max == sum_w)
    {
      S = sqrt(sum_w)*2;
      q[0] = (R[9] - R[6])/S;
      q[1] = (R[2] - R[8])/S;
      q[2] = (R[4] - R[1])/S;
      q[3] = 0.25*S;
    }
  else if (sum_max == sum_x)
    {
      S  = sqrt(sum_x) * 2;
      q[0] = 0.25 * S;
      q[1] = (R[4] + R[1] ) / S;
      q[2] = (R[2] + R[8] ) / S;
      q[3] = (R[9] - R[6] ) / S;
    } 
  else if (sum_max == sum_y) 
    { 
      S  = sqrt(sum_y) * 2;
      q[0] = (R[4] + R[1] ) / S;
      q[1] = 0.25 * S;
      q[2] = (R[9] + R[6] ) / S;
      q[3] = (R[2] - R[8] ) / S;
    } 
  else
    {
      S  = sqrt(sum_z) * 2;
      q[0] = (R[2] + R[8] ) / S;
      q[1] = (R[9] + R[6] ) / S;
      q[2] = 0.25 * S;
      q[3] = (R[4] - R[1] ) / S;
    } 

  //passage des quaternions aux axes et angles
  double sum;
  double cos_a,  sin_a;

  // Normalize
  sum = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  sum = sqrt(sum);

  if (sum==0)
    {
      Angle = Axis[0] = Axis[1] = Axis[2] = 0.0;
      return;
    }

  for(int i=0;i<4;i++)
    q[i] /= sum;

  cos_a = q[3];
  Angle = acos( cos_a ) * 2;
  sin_a = sqrt( 1.0 - cos_a * cos_a );
  if ( fabs( sin_a ) < 0.0005 ) 
    sin_a = 1;

  Axis[0] = -q[0] / sin_a;
  Axis[1] = -q[1] / sin_a;
  Axis[2] = -q[2] / sin_a;
}



MultiBody::MultiBody(void)
{

}


MultiBody::~MultiBody(void)
{
}

void MultiBody::ajouterCorps(Body &b)
{
  listeCorps.push_back(b);
  liaisons.push_back(vector<appariement>());
}

void MultiBody::ajouterLiaison(Body &corps1, Body &corps2, internalLink l)
{
  //recherche de l'index du premier corps dans listeCorps
  unsigned int index1;
  for (index1 = 0; index1<listeCorps.size(); index1++) {
    if (corps1.getLabel() == listeCorps[index1].getLabel()) {
      break;
    }
  }

  //recherche de l'index du deuxieme corps dans listeCorps
  unsigned int index2;
  for (index2 = 0; index2<listeCorps.size(); index2++) {
    if (corps2.getLabel() == listeCorps[index2].getLabel()) {
      break;
    }
  }

  l.indexCorps1 = index1;
  l.indexCorps2 = index2;
  listeLiaisons.push_back(l);		//ajout de la liaison a la liste
  //creation de l'appariement corps2, liaison
  appariement a2 = {index2, listeLiaisons.size()-1};
  liaisons[index1].push_back(a2);	//et ajout
  //creation de l'appariement corps1, liaison
  appariement a1 = {index1, listeLiaisons.size()-1};
  liaisons[index2].push_back(a1);	//et ajout
  for (unsigned int i=0; i<listeLiaisons[listeLiaisons.size()-1].listeTransformation.size(); i++) {
    listeVariableTransformation.push_back(&(listeLiaisons[listeLiaisons.size()-1].listeTransformation[i].quantite));
  }
}

void MultiBody::ajouterLiaisonFixe(Body &corps1, Body &corps2, 
				   MAL_S3_VECTOR(,double) translationStat, 
				   MAL_S3_VECTOR(,double) axeRotationStat, 
				   double angleRotationStat)
{
  //recherche de l'index du premier corps dans listeCorps
  unsigned int index1;
  for (index1 = 0; index1<listeCorps.size(); index1++) {
    if (corps1.getLabel() == listeCorps[index1].getLabel()) {
      break;
    }
  }

  //recherche de l'index du deuxieme corps dans listeCorps
  unsigned int index2;
  for (index2 = 0; index2<listeCorps.size(); index2++) {
    if (corps2.getLabel() == listeCorps[index2].getLabel()) {
      break;
    }
  }

  //creation de la liaison
  internalLink l = {cptLiaison++, FIX_JOINT, 
		    translationStat, axeRotationStat, angleRotationStat, 
		    vector<transfo>(), index1, index2};
  listeLiaisons.push_back(l);		//ajout de la liaison a la liste
  //creation de l'appariement corps2, liaison
  appariement a2 = {index2, listeLiaisons.size()-1};
  liaisons[index1].push_back(a2);	//et ajout
  //creation de l'appariement corps1, liaison
  appariement a1 = {index1, listeLiaisons.size()-1};
  liaisons[index2].push_back(a1);	//et ajout
  for (unsigned int i=0; i<l.listeTransformation.size(); i++) {
    listeVariableTransformation.push_back(&(l.listeTransformation[i].quantite));
  }
}

void MultiBody::ajouterLiaisonRotation(Body &corps1, Body &corps2, 
				       MAL_S3_VECTOR(,double) axe, 
				       MAL_S3_VECTOR(,double) translationStat, 
				       MAL_S3_VECTOR(,double) axeRotationStat, 
				       double angleRotationStat)
{
  //recherche de l'index du premier corps dans listeCorps
  unsigned int index1;
  for (index1 = 0; index1<listeCorps.size(); index1++) {
    if (corps1.getLabel() == listeCorps[index1].getLabel()) {
      break;
    }
  }

  //recherche de l'index du deuxieme corps dans listeCorps
  unsigned int index2;
  for (index2 = 0; index2<listeCorps.size(); index2++) {
    if (corps2.getLabel() == listeCorps[index2].getLabel()) {
      break;
    }
  }

  //creation de la rotation
  transfo t (ROTATION, axe, 0, NULL);

  //creation de la liaison
  internalLink l = {cptLiaison++, REVOLUTE_JOINT, translationStat, axeRotationStat, 
		    angleRotationStat, vector<transfo>(), index1, index2};
  l.listeTransformation.push_back(t);	//ajout rotation dynamique;
  listeLiaisons.push_back(l);		//ajout de la liaison a la liste
  //creation de l'appariement corps2, liaison
  appariement a2 = {index2, listeLiaisons.size()-1};
  liaisons[index1].push_back(a2);	//et ajout
  //creation de l'appariement corps1, liaison
  appariement a1 = {index1, listeLiaisons.size()-1};
  liaisons[index2].push_back(a1);	//et ajout
  for (unsigned int i=0; i<l.listeTransformation.size(); i++) {
    listeVariableTransformation.push_back(&(l.listeTransformation[i].quantite));
  }
}
void MultiBody::supprimerLiaisonEntre(Body &corps1, Body &corps2)
{
  //recherche de l'index du premier corps dans listeCorps
  unsigned int c1;
  for (c1 = 0; c1<listeCorps.size(); c1++) {
    if (corps1.getLabel() == listeCorps[c1].getLabel()) {
      break;
    }
  }

  //recherche de l'index du deuxieme corps dans listeCorps
  unsigned int c2;
  for (c2 = 0; c2<listeCorps.size(); c2++) {
    if (corps2.getLabel() == listeCorps[c2].getLabel()) {
      break;
    }
  }

  appariement a1 = {c2, 0};
  appariement a2 = {c1, 0};
  //recherche de l'iterateur de la liaison parmis les liaisons de corps1
  vector<appariement>::iterator it = find(liaisons[c1].begin(), liaisons[c1].end(), a1);
  int index = it->liaison;
  listeLiaisons[index].indexCorps1 = -1;
  listeLiaisons[index].indexCorps2 = -1;
  listeLiaisons[index].label = -1;
  liaisons[c1].erase(it);		//et on l'enleve
  //recherche de l'iterateur de la liaison parmis les liaisons de corps2
  it = find(liaisons[c2].begin(), liaisons[c2].end(), a2);
  liaisons[c2].erase(it);		//et on l'enleve
}

void MultiBody::supprimerLiaison(int index)
{
  int c1 = listeLiaisons[index].indexCorps1;
  int c2 = listeLiaisons[index].indexCorps2;
  appariement a1 = {c2, 0};
  appariement a2 = {c1, 0};
  //recherche de l'iterateur de la liaison parmis les liaisons de corps1
  vector<appariement>::iterator it = find(liaisons[c1].begin(), liaisons[c1].end(), a1);
  listeLiaisons[index].indexCorps1 = -1;
  listeLiaisons[index].indexCorps2 = -1;
  listeLiaisons[index].label = -1;
  liaisons[c1].erase(it);		//et on l'enleve
  //recherche de l'iterateur de la liaison parmis les liaisons de corps2
  it = find(liaisons[c2].begin(), liaisons[c2].end(), a2);
  liaisons[c2].erase(it);		//et on l'enleve
}

void MultiBody::supprimerLiaisonLabel(int label)
{
  //recherche de l'index de la liaison
  unsigned int i;
  for (i=0; i<listeLiaisons.size(); i++) {
    if (listeLiaisons[i].label == label)
      break;
  }
  if (i==listeLiaisons.size())
    return;

  supprimerLiaison(i);
}

void MultiBody::supprimerCorps(Body &b)
{
  unsigned int i;
  for (i=0; i<listeCorps.size(); i++) {
    if (listeCorps[i].getLabel() == b.getLabel())
      break;
  }
  if (i==listeCorps.size())
    return;

  supprimerCorps(i);
}
void MultiBody::supprimerCorps(int index)
{
  int j = 0;
  for (unsigned int i=0; i<liaisons[index].size(); j++) {
    supprimerLiaison(liaisons[index][i].liaison);
    cout << "suppression liaison\n";
  }
  listeCorps[index].setLabel(-1);

}
void MultiBody::supprimerCorpsLabel(int label)
{
  unsigned int i;
  for (i=0; i<listeCorps.size(); i++) {
    if (listeCorps[i].getLabel() == label)
      break;
  }
  if (i==listeCorps.size())
    return;

  supprimerCorps(i);
}

void MultiBody::inverserLiaison(int i)
{
  cout << "inverserLiaison n'est pas correctement implemente" << endl;
  /*
    if ((unsigned int)i >= listeLiaisons.size()) {
    cout << i << " : indice hors limite du vecteur de liaison" << endl;
    return;
    }
    for (unsigned int j=0; j<listeLiaisons[i].listeTransformation.size(); j++) {
    listeLiaisons[i].listeTransformation[j].axe = -listeLiaisons[i].listeTransformation[j].axe;
    }
  */
}



MAL_S3_VECTOR(,double) MultiBody::getPositionCoM(void)
{
  return (positionCoMPondere/masse);
}


void MultiBody::afficher()
{
  afficherCorps();
  afficherLiaisons();
}

void MultiBody::afficherCorps()
{
  for (unsigned int i=0; i<listeCorps.size(); i++) {
    cout << "corps "<< i << " : \n";
    listeCorps[i].Display();
    for (unsigned int j=0; j<liaisons[i].size(); j++) {
      cout << "    lie a corps " << liaisons[i][j].corps << " par liaison " 
	   << liaisons[i][j].liaison << " (label " << listeLiaisons[liaisons[i][j].liaison].label <<")\n";
    }
    cout << "\n";
  }
  cout << "\n";
}

void MultiBody::afficherLiaisons(void) {
  for (unsigned int i=0; i<listeLiaisons.size(); i++) {
    cout << "Name: "<< listeLiaisons[i].Name 
	 << " JointID in VRML " 
	 << listeLiaisons[i].IDinVRML << " " ;
    cout << "liaison de type " << listeLiaisons[i].type 
	 << "  label "<< listeLiaisons[i].label 
	 << "  liant le corps " 
	 << listeLiaisons[i].indexCorps1 
	 << " au corps " << listeLiaisons[i].indexCorps2 << "\n";
    cout << "translationStatique : " << endl;
    for(int j=0;j<3;j++)
      cout << listeLiaisons[i].translationStatique[j] << " ";
    cout << endl;
    if (listeLiaisons[i].type > 0) {
      cout << "    axe : " << endl;
      for(int j=0;j<3;j++)
	cout << listeLiaisons[i].listeTransformation[0].axe[j] << " ";
      cout << endl;
    }
  }
  cout << "\n";
}

void MultiBody::parserVRML(string path, string nom, const char* option)
{
  //-----------------------------------------
  // Declaration des variables

  MAL_S3_VECTOR(lxaxis,double);
  MAL_S3_VECTOR(lyaxis,double);
  MAL_S3_VECTOR(lzaxis,double);
  MAL_S3_VECTOR(lnull,double);
  lxaxis[0]=1.0;lxaxis[1]=0.0;lxaxis[2]=0.0;
  lyaxis[0]=0.0;lyaxis[1]=1.0;lyaxis[2]=0.0;
  lzaxis[0]=0.0;lzaxis[1]=0.0;lzaxis[2]=1.0;
  lnull[0]=0.0;lnull[1]=0.0;lnull[2]=0.0;
  FILE *fichier;
  int cptCorps = 0;
  int profondeur,jointID;
  bool childDescription = false;
  double cm[3], lmasse, mi[9];
  //  double r[4], tr[3];
  Body *dernierCorps = NULL;
  string nomWRML = path;
  nomWRML += nom;

  masse = 0;
  //  Body * aCC=0;
  vector<Body> corpsCourant;// = new Body [PROFONDEUR_MAX];
  corpsCourant.resize(PROFONDEUR_MAX);

  // Fin declaration des variables
  //-----------------------------------------

  //-----------------------------------------
  // Ouverture du fichier
  fichier = fopen(nomWRML.c_str(), "r");
  if (fichier == NULL)
    {
      cout << "File" << nom << " not find." << endl;
      return;
    }
  //  cout << "Fichier " << nom << " trouve." << endl;

  // Fin ouverture du fichier
  //-----------------------------------------

  // Deplacement jusqu'au debut de la description
  if (!look_for(fichier,"DEF")) {
    return;
  }

  // Creation du corps de reference
  corpsCourant[0].setLabel(cptCorps++);
  ajouterCorps(corpsCourant[0]);
  profondeur = 0;
  dernierCorps = &listeCorps[listeCorps.size()-1];
  internalLink liaisonCourante;

  char BufferDEFNAME[1024];
  static int counter=0;
  do {
    switch (nextKeyWord(fichier)) {
    case CROCHET_OUVRANT :
      //cout << "[";
      profondeur++;
      break;
    case CROCHET_FERMANT :
      //cout << "]";
      profondeur--;
      break;
    case DEF :
      bzero(BufferDEFNAME,1024);
      fscanf(fichier," %s",BufferDEFNAME);
      break;

    case CHILDREN :
      counter++;
      //      cout << "Children "<<counter << endl;
      if (childDescription) {	//On arrive a l'url du fichier VRML du corps courant


	if(look_for(fichier,"url"))	//a modifier eventuellement
	  {
#if 0	    	
	    char nomWRL[50];

	    //string path = "C:/user/Adrien/Code/HRP2 United Workspace/Geometrie/";
	    fscanf(fichier, "%s", nomWRL);
	    int l = (int)(strlen(nomWRL));
	    nomWRL[l-4] = 'b';
	    nomWRL[l-3] = 'o';
	    nomWRL[l-2] = 'd';
	    
	    basic_string <char> nomBOD(nomWRL, 1, l-2);
	    geometrieCorps g = lireBOD(path, nomBOD, option);
	    if (g.liste && g.couleur) {
	      dernierCorps->setNombreObjets(g.nb);
	      dernierCorps->setCouleurs(g.couleur);
	      dernierCorps->setLabelGLList(g.liste);
	    }

	    else {
	      corpsCourant[profondeur-1].setNombreObjets(0);
	    }
#endif
	    childDescription = !childDescription;
	  }
	else
	  {
	    cout << " Anomalie : balise url manquante" << endl;
	    return;
	  }

	if(look_for(fichier,"]"))	//a modifier eventuellement
	  {
					
	  }
	else
	  {
	    cout << " Anomalie : ] manquant" << endl;
	    return;
	  }
      }
      else {	//nouveau corps, qu'on lie a son pere
	counter++;
	//	cout << "newChildren "<<counter << endl;
	// Centres de masses
	if (look_for(fichier,"DEF"))
	  {
	    bzero(BufferDEFNAME,1024);
	    fscanf(fichier," %s",BufferDEFNAME);
	  }
	else
	  return;
	if (look_for(fichier,"centerOfMass"))
	  fscanf(fichier," %lf %lf %lf",&cm[0],&cm[1],&cm[2]);
	else
	  return;
	// Masses
	if (look_for(fichier,"mass"))
	  fscanf(fichier,"%lf",&lmasse);
	else
	  return;
	// Moments d'inertie
	if (look_for(fichier,"momentsOfInertia"))
	  fscanf(fichier," [%lf %lf %lf %lf %lf %lf %lf %lf %lf]",&mi[0],&mi[1],&mi[2],&mi[3],&mi[4],&mi[5],&mi[6],&mi[7],&mi[8]);
	else
	  return;
	if (profondeur < PROFONDEUR_MAX)
	  {
	    //	    corpsCourant[profondeur] = Body(masse, MAL_VECTOR(,double)(cm[0], cm[1], cm[2]), mi);
	    corpsCourant[profondeur].setLabel(cptCorps++);
	    corpsCourant[profondeur].setName(BufferDEFNAME);
	    corpsCourant[profondeur].setInertie(mi);
	    corpsCourant[profondeur].setMasse(lmasse);
	    masse += lmasse;
	    corpsCourant[profondeur].setPositionCoM(cm);
	    if (profondeur!=0)
	      corpsCourant[profondeur].setLabelMother(corpsCourant[profondeur-1].getLabel());
	    
	    //	    corpsCourant[profondeur].Display();
	    ajouterCorps(corpsCourant[profondeur]);
	    dernierCorps = &listeCorps[listeCorps.size()-1];
	    ajouterLiaison(corpsCourant[profondeur-1], corpsCourant[profondeur], liaisonCourante);
	    profondeur++;
	    childDescription = !childDescription;

	  }
	else
	  {
	    cout << "Profondeur Max depassee" << endl;
	    return;
	  }
      }
      break;
    case JOINT :
      liaisonCourante.type = typeOfJoint(fichier);
      liaisonCourante.label = cptLiaison++;
      liaisonCourante.Name = BufferDEFNAME;
      liaisonCourante.listeTransformation.clear();
      liaisonCourante.IDinVRML = -1;

      for (int k=0; k<((liaisonCourante.type==1)?3:1); k++) {
	switch (nextJointKeyWord(fichier)) {
	case AXE_X :
	  {
	    transfo at(ROTATION, lxaxis, 0, NULL);
	    liaisonCourante.listeTransformation.push_back(at);
	  }
	  break;
	case AXE_Y :
	  {
	    transfo at(ROTATION, lyaxis, 0, NULL);
	    liaisonCourante.listeTransformation.push_back(at);
	  }
	  break;
	case AXE_Z :
	  {
	    transfo at(ROTATION, lzaxis, 0, NULL);
	    liaisonCourante.listeTransformation.push_back(at);
	  }
	  break;
	case JOINT_TRANSLATION :
	  {
	    fscanf(fichier," %lf %lf %lf",
		   &liaisonCourante.translationStatique[0],
		   &liaisonCourante.translationStatique[1],
		   &liaisonCourante.translationStatique[2]);
	  }
	  break;
	case JOINT_ROTATION :
	  {
	    fscanf(fichier," %lf %lf %lf %lf",
		   &liaisonCourante.axeRotationStatique[0],
		   &liaisonCourante.axeRotationStatique[1],
		   &liaisonCourante.axeRotationStatique[2],
		   &liaisonCourante.angleRotationStatique);
	  }
	  break;
	case JOINT_ID :
	  {
	    fscanf(fichier,"%d",&jointID);
		      
	    liaisonCourante.IDinVRML=jointID;
	  }
	  break;
	}
      }
      if (liaisonCourante.type == FREE_JOINT) 
	{	//liaison FREE
	  transfo transDyn1(TRANSLATION, lxaxis, 0, NULL);
	  transfo transDyn2(TRANSLATION, lyaxis, 0, NULL);
	  transfo transDyn3(TRANSLATION, lzaxis, 0, NULL);
	  transfo transDyn4(ROTATION_LIBRE, lnull, 0, new float[16]);
	  transDyn4.rotation()[0] = 1;	transDyn4.rotation()[1] = 0;
	  transDyn4.rotation()[2] = 0;	transDyn4.rotation()[3] = 0;
	  transDyn4.rotation()[4] = 0;	transDyn4.rotation()[5] = 1;
	  transDyn4.rotation()[6] = 0;	transDyn4.rotation()[7] = 0;
	  transDyn4.rotation()[8] = 0;	transDyn4.rotation()[9] = 0;
	  transDyn4.rotation()[10] = 1;	transDyn4.rotation()[11] = 0;
	  transDyn4.rotation()[12] = 0;	transDyn4.rotation()[13] = 0;
	  transDyn4.rotation()[14] = 0;	transDyn4.rotation()[15] = 1;
	  liaisonCourante.listeTransformation.push_back(transDyn1);
	  liaisonCourante.listeTransformation.push_back(transDyn2);
	  liaisonCourante.listeTransformation.push_back(transDyn3);
	  liaisonCourante.listeTransformation.push_back(transDyn4);
	}
      
      break;
    case -1:
      cout << "Anomalie -1" << endl;
      break;
    case -2:
      fclose(fichier);
      profondeur = -4;
      break;
    }
  } while (profondeur != -4);
  //  cout << "Masse du robot "<< masse << endl;
}



vector<float*> MultiBody::getListeVariableTransformation(void)
{
  vector<float*> v = vector<float*>();
  for (unsigned int i=0; i<listeLiaisons.size(); i++) {
    for (unsigned int j=0; j<listeLiaisons[i].listeTransformation.size(); j++) {
      if (listeLiaisons[i].listeTransformation[j].type == ROTATION_LIBRE) {
	v.push_back(listeLiaisons[i].listeTransformation[j].rotation());
      }
      else {
	v.push_back(&(listeLiaisons[i].listeTransformation[j].quantite));
      }
      cout << "liaison " << i << " : " << (v.size()-1) << endl;
    }
  }
  return v;
}

int MultiBody::NbOfLinks()
{
  return listeLiaisons.size();
}

double MultiBody::getMasse()
{
  return masse;
}
