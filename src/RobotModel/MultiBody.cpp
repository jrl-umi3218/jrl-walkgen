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

void MultiBody::ajouterLiaison(Body &corps1, Body &corps2, internalLink & l)
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
  internalLink l = {cptLiaison++,
		    Joint(Joint::FIX_JOINT,axeRotationStat,angleRotationStat,translationStat), index1, index2};
  listeLiaisons.push_back(l);		//ajout de la liaison a la liste
  //creation de l'appariement corps2, liaison
  appariement a2 = {index2, listeLiaisons.size()-1};
  liaisons[index1].push_back(a2);	//et ajout
  //creation de l'appariement corps1, liaison
  appariement a1 = {index1, listeLiaisons.size()-1};
  liaisons[index2].push_back(a1);	//et ajout
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
    for (unsigned int j=0; j<listeLiaisons[i].listeJoints.size(); j++) {
    listeLiaisons[i].listeJoints[j].axe = -listeLiaisons[i].listeJoints[j].axe;
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
    cout << "Name: "<< listeLiaisons[i].aJoint.getName()
	 << " JointID in VRML " 
	 << listeLiaisons[i].aJoint.getIDinVRML() << " " ;
    cout << "liaison de type " << listeLiaisons[i].aJoint.type()
	 << "  label "<< listeLiaisons[i].label 
	 << "  liant le corps " 
	 << listeLiaisons[i].indexCorps1 
	 << " au corps " << listeLiaisons[i].indexCorps2 << "\n";
    cout << "translationStatique : " << endl;
    MAL_S3_VECTOR(,double) aStaticTranslation;
    listeLiaisons[i].aJoint.getStaticTranslation(aStaticTranslation);
    cout << aStaticTranslation << endl;
    if (listeLiaisons[i].aJoint.type() > 0) {
      cout << "    axe : " << endl;
      cout << listeLiaisons[i].aJoint.axe();
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
  MAL_S3_VECTOR(,double) dummy;
  internalLink CurrentLink={ 0, Joint(Joint::FIX_JOINT,dummy,0.0), 0,0};

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
	      dernierCorps->setNbObjets(g.nb);
	      dernierCorps->setCouleurs(g.couleur);
	      dernierCorps->setLabelGLList(g.liste);
	    }

	    else {
	      corpsCourant[profondeur-1].setNbObjets(0);
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
	  fscanf(fichier," [%lf %lf %lf %lf %lf %lf %lf %lf %lf]",
		 &mi[0],&mi[1],&mi[2],&mi[3],&mi[4],&mi[5],&mi[6],&mi[7],&mi[8]);
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
	    ajouterLiaison(corpsCourant[profondeur-1], corpsCourant[profondeur], CurrentLink);
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
      
      CurrentLink.aJoint.type(typeOfJoint(fichier));
      CurrentLink.label = cptLiaison++;

      for (int k=0; k<((CurrentLink.aJoint.type()==1)?3:1); k++) {
	switch (nextJointKeyWord(fichier)) {
	case AXE_X :
	  {
	    CurrentLink.aJoint.type(Joint::REVOLUTE_JOINT);
	    CurrentLink.aJoint.axe(lxaxis);
	  }
	  break;
	case AXE_Y :
	  {
	    CurrentLink.aJoint.type(Joint::REVOLUTE_JOINT);
	    CurrentLink.aJoint.axe(lyaxis);
	  }
	  break;
	case AXE_Z :
	  {
	    CurrentLink.aJoint.type(Joint::REVOLUTE_JOINT);
	    CurrentLink.aJoint.axe(lzaxis);
	  }
	  break;
	case JOINT_TRANSLATION :
	  {
	    MAL_S3_VECTOR(,double) aVec;
	    fscanf(fichier," %lf %lf %lf",&aVec(0),&aVec(1), &aVec(2));
	    CurrentLink.aJoint.setStaticTranslation(aVec);
	  }
	  break;
	case JOINT_ROTATION :
	  {
	    MAL_S3_VECTOR(,double) AnAxis;
	    double aQuantity;
	    fscanf(fichier," %lf %lf %lf %lf", & AnAxis(0),
		   &AnAxis(1), &AnAxis(2), &aQuantity);
	    CurrentLink.aJoint.axe(AnAxis);
	    CurrentLink.aJoint.quantity(aQuantity);
	  }
	  break;
	case JOINT_ID :
	  {
	    fscanf(fichier,"%d",&jointID);
		      
	    CurrentLink.aJoint.setIDinVRML(jointID);
	  }
	  break;
	}
      }
      if (CurrentLink.aJoint.type() == Joint::FREE_JOINT) 
	{	//liaison FREE
	  
	  Joint transDyn4(Joint::FREE_JOINT, lnull, 0);
	  CurrentLink.aJoint= transDyn4;
	}
      
      {
	string sas(BufferDEFNAME);
	CurrentLink.aJoint.setName(sas);
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
  //  afficherCorps();
  //  afficherLiaisons();
  //  cout << "Masse du robot "<< masse << endl;
}




int MultiBody::NbOfLinks() const
{
  return listeLiaisons.size();
}

int MultiBody::NbOfJoints() const
{
  return listeLiaisons.size();
}

double MultiBody::getMasse()
{
  return masse;
}
