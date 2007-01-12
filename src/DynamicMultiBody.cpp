/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 

   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.

   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   
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


#include <DynamicMultiBody.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "DynamicMultiBody :" << x << endl

#if 0
#define ODEBUG(x) cerr << "DynamicMultiBody :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0

#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "WalkGenJRLIntegrate: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define ODEBUG4(x,y)
#endif

using namespace PatternGeneratorJRL;

DynamicMultiBody::DynamicMultiBody()
{
  labelTheRoot=1;
  
}

DynamicMultiBody::~DynamicMultiBody()
{
  
}

void DynamicMultiBody::SpecifyTheRootLabel(int ID)
{
  labelTheRoot = ID;
  listOfBodies[ID].setLabelMother(-1);
  for(unsigned int i=0;i<liaisons[ID].size();i++)
    {
      ReLabelling(ID,liaisons[ID][i].liaison);
    }
  
  // Once finished we initialize the child and the sister.
  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      int lMother,lElderSister;
      if ((lMother=listOfBodies[i].getLabelMother()) != -1)
	{
	  if ((lElderSister=listOfBodies[lMother].child) == -1)
	    listOfBodies[lMother].child = i;					  // Mother, I am your daughter !
	  else
	    {
	      // I have an elder sister !

	      while (listOfBodies[lElderSister].sister != -1)
		lElderSister = listOfBodies[lElderSister].sister;  // I have another elder sister !

	      listOfBodies[lElderSister].sister = i;				  // I am your younger sister !
	    }
	}
    }
}

void DynamicMultiBody::UpdateBodyParametersFromJoint(int cID, int lD)
  // cID : corps identifier
  // lD : liaison destination
{

  // Update the rotation axis.
  listOfBodies[cID].a =  listeLiaisons[lD].listeTransformation[0].axe;
  // Update the translation vector
  listOfBodies[cID].b = listeLiaisons[lD].translationStatique;

  // listOfBodies[cID].R = 
}

void DynamicMultiBody::ReLabelling(int corpsCourant, int liaisonDeProvenance)
{
  switch (liaisons[corpsCourant].size()) {
  case 1 :
    if (corpsCourant==labelTheRoot)
    {
      int liaisonDestination = liaisons[corpsCourant][0].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	listOfBodies[corps2].setLabelMother(corps1);
	ReLabelling(corps2, liaisonDestination);
	if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	  ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps2;
	UpdateBodyParametersFromJoint(corps2,liaisonDestination);
      }
      else {
	listOfBodies[corps1].setLabelMother(corps2);
	ReLabelling(corps1, liaisonDestination);
	if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	  ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps1;
	UpdateBodyParametersFromJoint(corps1,liaisonDestination);
      }
    }
    break;
  case 2 :
    if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance) {
      int liaisonDestination = liaisons[corpsCourant][1].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	listOfBodies[corps2].setLabelMother(corps1);
	ReLabelling(corps2, liaisonDestination);
	if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	  ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps2;
	UpdateBodyParametersFromJoint(corps2,liaisonDestination);
      }
      else {
	listOfBodies[corps1].setLabelMother(corps2);
	ReLabelling(corps1, liaisonDestination);
	if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	  ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps1;
	UpdateBodyParametersFromJoint(corps1,liaisonDestination);
      }
    }
    else 
      {
	int liaisonDestination = liaisons[corpsCourant][0].liaison;
	int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	if (corpsCourant == corps1) 
	  {
	    listOfBodies[corps2].setLabelMother(corps1);
	    ReLabelling(corps2, liaisonDestination);
	    if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	      ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps2;
	    UpdateBodyParametersFromJoint(corps2,liaisonDestination);
	  }
	else 
	  {
	    listOfBodies[corps1].setLabelMother(corps2);
	    ReLabelling(corps1, liaisonDestination);
	    if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	      ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps1;
	    UpdateBodyParametersFromJoint(corps1,liaisonDestination);
	  }
      }
    break;
  default : //on a plus de deux liaisons
    for (unsigned int i=0; i<liaisons[corpsCourant].size(); i++) 
      {
	if (liaisons[corpsCourant][i].liaison == liaisonDeProvenance) 
	  continue;
	int liaisonDestination = liaisons[corpsCourant][i].liaison;
	int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	if (corpsCourant == corps1) 
	  {
	    listOfBodies[corps2].setLabelMother(corps1);
	    ReLabelling(corps2, liaisonDestination);
	    if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	      ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps2;
	    UpdateBodyParametersFromJoint(corps2,liaisonDestination);
	  }
	else 
	  {
	    listOfBodies[corps1].setLabelMother(corps2);
	    ReLabelling(corps1, liaisonDestination);
	    if( listeLiaisons[liaisonDestination].IDinVRML!=-1)
	      ConvertJOINTIDToBodyID[listeLiaisons[liaisonDestination].IDinVRML] = corps1;
	    UpdateBodyParametersFromJoint(corps1,liaisonDestination);
	  }
      }
    break;
  }

}

void DynamicMultiBody::ForwardVelocity(MAL_S3_VECTOR(&PosForRoot,double), 
				       MAL_S3x3_MATRIX(&OrientationForRoot,double),
				       MAL_S3_VECTOR(&v0ForRoot,double) )
{
  double norm_w, th;
 //  int NbOfNodes=1;
  int currentNode = labelTheRoot;
  int lMother=0;
  MAL_S3x3_MATRIX( Ro,double);
  MAL_S3x3_MATRIX( w_wedge,double);
  MAL_S3_VECTOR( wn,double);
  double NORME_EPSILON=10e-7;

  listOfBodies[labelTheRoot].p = PosForRoot;
  listOfBodies[labelTheRoot].v0 = v0ForRoot;
  listOfBodies[labelTheRoot].R = OrientationForRoot;
  currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;
  
  MAL_S3_VECTOR_FILL(m_P,0);
  MAL_S3_VECTOR_FILL(m_L,0);
  MAL_S3_VECTOR(lP,double);
  MAL_S3_VECTOR(lL,double);

  currentNode = listOfBodies[labelTheRoot].child;
  positionCoMPondere[0] = 0;
  positionCoMPondere[1] = 0;
  positionCoMPondere[2] = 0;
  
  MAL_S3_VECTOR(tmp,double);
  MAL_S3_VECTOR(tmp2,double);
  ODEBUG("PosForRoot: " << PosForRoot );
  ODEBUG("v0ForRoot: " << v0ForRoot );
  ODEBUG("OrientationForRoot: " << OrientationForRoot );
  do
    {

      DynamicBody aDB = listOfBodies[currentNode];

      norm_w = MAL_S3_VECTOR_NORM(aDB.a);
      lMother = aDB.getLabelMother();

      ODEBUG("CurrentBody " << listOfBodies[currentNode].getName());
      
      // ----------------------------------
      // Rodrigues formula. (p33)
      if (norm_w< NORME_EPSILON)
	{
	  MAL_S3x3_MATRIX_SET_IDENTITY(Ro);
	}
      else 
	{
	  th = norm_w * aDB.q;
 	  wn = aDB.a / norm_w;
	  w_wedge(0,0) =   0.0;w_wedge(0,1)= -wn[2]; w_wedge(0,2)=  wn[1]; // Cross product
	  w_wedge(1,0) = wn[2];w_wedge(1,1)=    0.0; w_wedge(1,2)= -wn[0];
	  w_wedge(2,0) =-wn[1];w_wedge(2,1)=  wn[0]; w_wedge(2,2)=    0.0;
	  
	  ODEBUG("w_wedge : " << w_wedge);
	  ODEBUG("aDB.a :" << aDB.a );
	  ODEBUG("norm_w:" << norm_w);

	  double ct = cos(th); double lct= (1-ct);
	  double st = sin(th);
	  Ro(0,0) = ct + wn[0]*wn[0]* lct;  
	  Ro(0,1) = wn[0]*wn[1]*lct-wn[2]*st; 
	  Ro(0,2) = wn[1] * st+wn[0]*wn[2]*lct;
	  Ro(1,0) = wn[2]*st +wn[0]*wn[1]*lct; 
	  Ro(1,1) = ct + wn[1]*wn[1]*lct;    
	  Ro(1,2) = -wn[0]*st+wn[1]*wn[2]*lct;
	  Ro(2,0) = -wn[1]*st+wn[0]*wn[2]*lct; 
	  Ro(2,1) = wn[0]*st + wn[1]*wn[2]*lct; 
	  Ro(2,2) = ct + wn[2]*wn[2]*lct;
	}
      
      ODEBUG("Ro:" << endl << Ro );
      ODEBUG("MR:" << listOfBodies[lMother].R );
      ODEBUG("b: " << aDB.b);
      ODEBUG("Mp: " << listOfBodies[lMother].p);
      // End Rodrigues formula
      //-------------------------------

      // Position and orientation in reference frame
      listOfBodies[currentNode].p = MAL_S3x3_RET_A_by_B(listOfBodies[lMother].R , aDB.b )
	+ listOfBodies[lMother].p;
      MAL_S3x3_C_eq_A_by_B(listOfBodies[currentNode].R ,listOfBodies[lMother].R , Ro);

      ODEBUG("q: "<< aDB.q );
      ODEBUG("p: " << listOfBodies[currentNode].p[0] << " " 
	     << listOfBodies[currentNode].p[1] << " " 
	     << listOfBodies[currentNode].p[2] << " " );

      // Computes the angular velocity. 
      tmp = listOfBodies[currentNode].a * listOfBodies[currentNode].dq;
      tmp = MAL_S3x3_RET_A_by_B(listOfBodies[lMother].R,tmp);

      listOfBodies[currentNode].w  = listOfBodies[lMother].w  + tmp;

      ODEBUG("w: " << listOfBodies[currentNode].w );

      // Computes the linear velocity.
      MAL_S3x3_C_eq_A_by_B(tmp,listOfBodies[lMother].R,
		      listOfBodies[currentNode].b);

      MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[lMother].w , tmp);

      listOfBodies[currentNode].v0 = listOfBodies[lMother].v0 + tmp2;
      ODEBUG("v0: " 
	      << listOfBodies[currentNode].v0[0] << " " 
	      << listOfBodies[currentNode].v0[1] << " " 
	      << listOfBodies[currentNode].v0[2] << " " );
	
      // Computes also the center of mass in the reference frame.

      MAL_S3_VECTOR( cl , double);
      MAL_S3x3_C_eq_A_by_B(cl,listOfBodies[currentNode].R, listOfBodies[currentNode].c);
      MAL_S3_VECTOR(lw_c,double);
      lw_c = cl + listOfBodies[currentNode].p;
      positionCoMPondere +=  lw_c * listOfBodies[currentNode].getMasse();
      ODEBUG("w_c: " << lw_c[0] << " " << lw_c[1] << " " << lw_c[2]);

      // Computes momentum matrix P.
      tmp2 = cl;
      ODEBUG("w: " << listOfBodies[currentNode].w );
      MAL_S3_VECTOR_CROSS_PRODUCT(tmp, tmp2 , listOfBodies[currentNode].w);
      ODEBUG("cl^w: " << tmp);
      ODEBUG("masse: " << listOfBodies[currentNode].getMasse());
      ODEBUG("v0: " << listOfBodies[currentNode].v0 );
      lP=  (listOfBodies[currentNode].v0 + 
	    tmp )* listOfBodies[currentNode].getMasse();
      listOfBodies[currentNode].P = lP;
      ODEBUG("P: " << lP );
      m_P += lP;
      
      // Computes angular momentum matrix L
      MAL_S3x3_MATRIX( Rt,double);
      Rt = listOfBodies[currentNode].R;
      Rt = MAL_S3x3_RET_TRANSPOSE(Rt);

      MAL_S3_VECTOR(tmp3,double);
      MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,lw_c,lP);

      MAL_S3x3_C_eq_A_by_B(tmp2,Rt , listOfBodies[currentNode].w);
      MAL_S3x3_C_eq_A_by_B(tmp, listOfBodies[currentNode].getInertie(),tmp2);
      MAL_S3x3_C_eq_A_by_B(tmp2, listOfBodies[currentNode].R,tmp);
      lL = tmp3 + tmp2; 
      ODEBUG("L: " << lL);
      
      listOfBodies[currentNode].L = lL;
      listOfBodies[currentNode].w_c = lw_c;
      m_L+= lL;
      

      // TO DO if necessary : cross velocity.
      
      int step=0;
      int NextNode=0;
      do{
       
	if (step==0)
	  {
	    NextNode = listOfBodies[currentNode].child;
	    step++;
	  }
	else if(step==1)
	  {
	    NextNode = listOfBodies[currentNode].sister;
	    step++;
	  }
	else if (step==2)
	  {
	    NextNode = listOfBodies[currentNode].getLabelMother();
	    if (NextNode>=0)
	      {
		currentNode = NextNode;
		NextNode = listOfBodies[currentNode].sister;
	      }
	    else 
	      NextNode=labelTheRoot;
	  }

	  
      } 
      while (NextNode==-1);
      currentNode = NextNode;
      
    }
  while(currentNode!=labelTheRoot);

  // Compute the skew matrix related to the weighted CoM.
  MAL_S3_VECTOR(,double) lpComP = positionCoMPondere/masse;

  SkewCoM(0,0) =         0; SkewCoM(0,1) = - lpComP[2]; SkewCoM(0,2) = lpComP[1];
  SkewCoM(1,0) = lpComP[2]; SkewCoM(1,1) =           0; SkewCoM(1,2) =-lpComP[0];
  SkewCoM(2,0) =-lpComP[1]; SkewCoM(2,1) =   lpComP[0]; SkewCoM(2,2) =         0;
}


void DynamicMultiBody::InertiaMatricesforRMCFirstStep()
{

  double norm_w;
  //  int NbOfNodes=1;
  int currentNode = labelTheRoot;
  int lMother=0;
  MAL_S3x3_MATRIX(Ro,double);
  MAL_S3x3_MATRIX(w_wedge,double);
  
  currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;

  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      listOfBodies[i].setExplored(0);
      listOfBodies[i].m_tildem=0.0;
      listOfBodies[i].m_tildem_sister=0.0;
      MAL_S3_VECTOR_FILL(listOfBodies[i].m_tildec,0.0);
      MAL_S3_VECTOR_FILL(listOfBodies[i].m_tildec_sister,0.0);
      
    }
  // cout << "labelTheRoot" << labelTheRoot<< endl;
  //  cout << "Current Node while starting: " << currentNode;
  //  cout << "Mother of the current Node" << listOfBodies[currentNode].getLabelMother() << endl;
  DynamicBody aDB;
  do
    {

      aDB = listOfBodies[currentNode];
      norm_w = MAL_S3_VECTOR_NORM(aDB.a);
      lMother = aDB.getLabelMother();
      

      int NextNode=0;
      int lContinue = 1;
      bool Ec=false, Es=false;
      int Ic=-1, Is=-1;

      do{

	Ec=false; Es=false;
	// Test if the node should be compute.
	Ic = aDB.child;
	Is = aDB.sister;
	if (!aDB.getExplored())
	  {
	    // Leaf
	    if ((Ic==-1) && (Is==-1))
	      {
		Es=Ec=true;
		lContinue =0;
	      }
	    else 
	      {
		
		if (Ic!=-1)
		  {
		    //cout << "IC: " << Ic << " " << (int) listOfBodies[Ic].getExplored() << endl;
		    if (listOfBodies[Ic].getExplored())
		      Ec = true;
		  }
		else Ec = true;
		
		if (Is!=-1)
		  {
		    //cout << "IS: " << Is << " " << (int) listOfBodies[Is].getExplored() << endl;
		    if (listOfBodies[Is].getExplored())
		      Es =true;
		  }
		else Es =true;
		
		if (Es && Ec)
		  lContinue = 0;
	      }
	  }

	// Depth first exploration
	if (lContinue)
	  {
	    if ((Ic!=-1) && (!Ec))
	      NextNode = aDB.child;
	    else 
	      NextNode = aDB.sister;
	    currentNode = NextNode;

	  } 
	aDB = listOfBodies[currentNode];
	//	cout << aDB.getName()<< " " << currentNode << endl;
	//	cout << aDB.sister << endl;
      }
      while (lContinue);

      double ltotaltildem = aDB.getMasse();      
      /*      cout << "FirstStep:currentNode : " << currentNode << " " << listOfBodies[currentNode].getLabelMother() 
	   << " " << aDB.getName() << endl;
      cout << "ltotaltiledm " << ltotaltildem << " " << listOfBodies[currentNode].getName()<< endl;
      cout << "Ic " << Ic << " Is :" <<Is << " Ec :" << (int)Ec << " Es :" << (int)Es << endl;
      */

      // Compute tilde m
      aDB.m_tildem = ltotaltildem;
      
      if ((Ec) && (Ic!=-1))
	{
	  aDB.m_tildem += listOfBodies[Ic].m_tildem +
	    listOfBodies[Ic].m_tildem_sister;
	}
      
      if ((Es) && (Is!=-1))
	{
	  aDB.m_tildem_sister += listOfBodies[Is].m_tildem +
	    listOfBodies[Is].m_tildem_sister;
	}
	
      //      cout << "Up here" << endl;
      // Compute Tilde CoM (Eq 24 on Kajita IROS 2003 p1647)
      MAL_S3_VECTOR(ltildec,double);
      ltildec =   aDB.w_c *ltotaltildem ;
      
      // Compute Tilde CoM for this Node.
      if ((Ec) && (Ic!=-1))
	{
	  
	  double lIctildem = listOfBodies[Ic].m_tildem;
	  ltotaltildem += lIctildem;
	  double lIctildem_sister = listOfBodies[Ic].m_tildem_sister;
	  ltotaltildem += lIctildem_sister;

	  ltildec += (listOfBodies[Ic].m_tildec * lIctildem) +
	    (listOfBodies[Ic].m_tildec_sister * lIctildem_sister);
	  /*
	  cout << "Ic " << Ic << " Ec " << Ec << endl;
	  cout << "listOfBodies[Ic].m_tildec " << listOfBodies[Ic].m_tildec << endl;
	  cout << "listOfBodies[Ic].m_tildec_sister " << listOfBodies[Ic].m_tildec_sister << endl;
	  cout << "ltotaltildem "<< ltotaltildem <<endl;
	  */
	}
      aDB.m_tildec = ltildec * (1.0/ltotaltildem);

      //      cout << "Up here 2" << endl;      
      // Compute TildeCom for the tree having the sister as root.
      if ((Es) && (Is!=-1))
	{
	  double l2totaltildem = 0.0;
	  double lIctildem = listOfBodies[Is].m_tildem;
	  l2totaltildem += lIctildem;
	  double lIctildem_sister = listOfBodies[Is].m_tildem_sister;
	  l2totaltildem += lIctildem_sister;

	  aDB.m_tildec_sister += 
	    (listOfBodies[Is].m_tildec * lIctildem +
	    listOfBodies[Is].m_tildec_sister * lIctildem_sister)*(1.0/l2totaltildem) ;
	}
      
      //      cout << "Up here 3" << endl;
      // Setting variables for the depth first exploration
      aDB.setExplored(1);
      //cout << "Node : " << currentNode << " Name " << aDB.getName() << endl; 
      //cout << "Tilde m: " << aDB.m_tildem << endl;
      //cout << "Tilde c: " << aDB.m_tildec << endl;
      
      listOfBodies[currentNode] = aDB;
      currentNode = aDB.getLabelMother();
      //      cout << "currentNode :" << currentNode << endl;
      
    }
  while(currentNode!=-1);
  //  cout << "Masse du corps: " << listOfBodies[0].m_tildem << endl;
}

MAL_S3x3_MATRIX(,double) DynamicMultiBody::D(MAL_S3_VECTOR(,double) &r)
{
  MAL_S3x3_MATRIX( res,double);

  res(0,0) = r[0]*r[0];
  res(0,1) = r[0]*r[1];
  res(0,2) = r[0]*r[2];
  
  res(1,0) = r[1]*r[0];
  res(1,1) = r[1]*r[1];
  res(1,2) = r[1]*r[2];

  res(2,0) = r[2]*r[0];
  res(2,1) = r[2]*r[1];
  res(2,2) = r[2]*r[2];
  
  return res;
}

void DynamicMultiBody::InertiaMatricesforRMCSecondStep()
{

  double norm_w;
  //  int NbOfNodes=1;
  int currentNode = labelTheRoot;
  int lMother=0;
  MAL_S3x3_MATRIX( Ro,double);
  MAL_S3x3_MATRIX(w_wedge,double);
  
  currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;

  currentNode = listOfBodies[labelTheRoot].child;
  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      listOfBodies[i].setExplored(0);
    }
  do
    {

      DynamicBody aDB = listOfBodies[currentNode];

      norm_w = MAL_S3_VECTOR_NORM(aDB.a);
      lMother = aDB.getLabelMother();

      
      int NextNode=0;
      int lContinue = 1;
      bool Ec=false, Es=false;
      int Ic=-1, Is=-1;

      do{

	Ec=false; Es=false;
	// Test if the node should be compute.
	Ic = aDB.child;
	Is = aDB.sister;
	//cout << "Ic : "<< Ic << " Is:" << Is << endl;
	if (!aDB.getExplored())
	  {
	    // Leaf
	    if ((Ic==-1) && (Is==-1))
	      {
		Es=Ec=true;
		lContinue =0;
	      }
	    else 
	      {
		
		if (Ic!=-1)
		  {
		    //cout << "IC: " << Ic << " " << (int) listOfBodies[Ic].getExplored() << endl;
		    if (listOfBodies[Ic].getExplored())
		      Ec = true;
		  }
		else Ec = true;
		
		if (Is!=-1)
		  {
		    //cout << "IS: " << Is << " " << (int) listOfBodies[Is].getExplored() << endl;
		    if (listOfBodies[Is].getExplored())
		      Es =true;
		  }
		else Es =true;
		
		if (Es && Ec)
		  lContinue = 0;
	      }
	  }

	// Depth first exploration
	if (lContinue)
	  {
	    if ((Ic!=-1) && (!Ec))
	      NextNode = aDB.child;
	    else 
	      NextNode = aDB.sister;
	    currentNode = NextNode;

	  } 

	aDB = listOfBodies[currentNode];
      }
      while (lContinue);
      
      //cout << "Body evaluated " << aDB.getName() <<endl;
      // Compute Tilde inertia matrix.
      MAL_S3x3_MATRIX(tmp,double);
      MAL_S3x3_MATRIX(tmp2,double);
      tmp = aDB.R;
      tmp = MAL_S3x3_RET_TRANSPOSE(tmp);
      MAL_S3x3_C_eq_A_by_B(tmp2,aDB.R,aDB.getInertie());
      tmp = MAL_S3x3_RET_A_by_B(tmp2,tmp);
      
      MAL_S3_VECTOR(diff_vec,double);
      diff_vec = aDB.w_c - aDB.m_tildec; 

      MAL_S3x3_MATRIX(diff_mat,double);
      diff_mat = D(diff_vec) * aDB.getMasse();
      diff_mat = tmp;
      
      aDB.m_tildeI = diff_mat;
      if ((Ec) && (Ic!=-1))
	{
	  MAL_S3_VECTOR(diff_vec,double);
	  diff_vec = listOfBodies[Ic].w_c - aDB.m_tildec; 
	  MAL_S3x3_MATRIX(tmp2,double);
	  tmp2 = D(diff_vec);

	  /*
	  aDB.m_tildeI +=  listOfBodies[Ic].m_tildeI +
	    tmp2 * listOfBodies[Ic].m_tildem + 
	    listOfBodies[Ic].m_tildeI_sister +  
	    listOfBodies[Ic].m_Dsister * listOfBodies[Ic].m_tildem_sister; */
	  aDB.m_tildeI +=  listOfBodies[Ic].m_tildeI;
	  aDB.m_tildeI +=  tmp2 * listOfBodies[Ic].m_tildem;
	  aDB.m_tildeI +=  tmp2 * listOfBodies[Ic].m_tildeI_sister;
	  aDB.m_tildeI +=  listOfBodies[Ic].m_Dsister * listOfBodies[Ic].m_tildem_sister;
	    
	}

      if ((Es) && (Is!=-1))
	{
	  int lMother = aDB.getLabelMother();
	  MAL_S3_VECTOR(diff_vec,double);
	  diff_vec = listOfBodies[Is].w_c - listOfBodies[lMother].m_tildec; 
	  MAL_S3x3_MATRIX(tmp2,double);
	  tmp2 = D(diff_vec);
	  
	  aDB.m_Dsister = tmp2;
	  aDB.m_tildeI_sister =  listOfBodies[Is].m_tildeI 
	    + ( tmp2 * listOfBodies[Is].m_tildem )
	    + listOfBodies[Is].m_tildeI_sister 
	    + (listOfBodies[Is].m_Dsister*  listOfBodies[Is].m_tildem_sister );

	}


      // Compute the two inertia matrices following Kajita 2003 IROS, p1647
      
      MAL_S3_VECTOR_CROSS_PRODUCT(aDB.m_RMC_m ,
				  aDB.a,
				  ( (aDB.m_tildec - aDB.p) * 
				    aDB.m_tildem)); // Eq 18

      MAL_S3_VECTOR(h0,double);
      MAL_S3_VECTOR_CROSS_PRODUCT(h0,
				  aDB.m_tildec,
				  aDB.m_RMC_m);  
      h0= h0 + MAL_S3x3_RET_A_by_B(aDB.m_tildeI, aDB.a); // Eq 19

      aDB.m_RMC_h = h0 - MAL_S3x3_RET_A_by_B(SkewCoM,aDB.m_RMC_m); // Eq 21
      
      //cout << "Mtilde :" << aDB.m_RMC_m << " " << aDB.m_RMC_h <<endl;
      // Computation of the last 
      // Setting variables for the depth first exploration
      aDB.setExplored(1);
      /*      cout << "Node : " << currentNode << " Name " << aDB.getName() << endl;
      cout << "RMC m: " << aDB.m_RMC_m << endl;
      cout << "RMC h: " << aDB.m_RMC_h << endl;
      */
      listOfBodies[currentNode] = aDB;
      currentNode = aDB.getLabelMother();
    }
  while(currentNode!=-1);
  
  //  cout << "Masse du corps: " << listOfBodies[0].m_tildem << endl;
}

void DynamicMultiBody::ForwardDynamics(int corpsCourant, int liaisonDeProvenance)
{
  //on fait les transfo necessaire
  if (listeLiaisons[liaisonDeProvenance].indexCorps1 == corpsCourant) 
    {
      empilerTransformationsLiaisonInverse(liaisonDeProvenance);
    }
  else 
    {
      empilerTransformationsLiaisonDirecte(liaisonDeProvenance);
    }
  
  
  if (liaisonDeProvenance == 0) { //a revoir
    masse = 0;
    MAL_S3_VECTOR_FILL(positionCoMPondere,0);
  }
  
  //  double matrice[16];
  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);
  //  positionCoMPondere += (matrice*(listeCorps[corpsCourant].getPositionCoM()))*listeCorps[corpsCourant].getMasse();
  masse += listeCorps[corpsCourant].getMasse();
  //listeCorps[corpsCourant].dessiner(alpha);
  
  switch (liaisons[corpsCourant].size()) {
  case 1 :
    break;
  case 2 :
    if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance) {
      int liaisonDestination = liaisons[corpsCourant][1].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	ForwardDynamics(corps2, liaisonDestination);
      }
      else {
	ForwardDynamics(corps1, liaisonDestination);
      }
    }
    else 
      {
	int liaisonDestination = liaisons[corpsCourant][0].liaison;
	int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	if (corpsCourant == corps1) 
	  {
	    ForwardDynamics(corps2, liaisonDestination);
	  }
	else 
	  {
	    ForwardDynamics(corps1, liaisonDestination);
	  }
      }
    break;
  default : //on a plus de deux liaisons
    for (unsigned int i=0; i<liaisons[corpsCourant].size(); i++) 
      {
	if (liaisons[corpsCourant][i].liaison == liaisonDeProvenance) 
	  continue;
	int liaisonDestination = liaisons[corpsCourant][i].liaison;
	int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	if (corpsCourant == corps1) 
	  {
	    ForwardDynamics(corps2, liaisonDestination);
	  }
	else 
	  {
	    ForwardDynamics(corps1, liaisonDestination);
	  }
      }
    break;
  }
}

inline void DynamicMultiBody::empilerTransformationsLiaisonDirecte(int liaison)
{
  //MAL_VECTOR_DIM(tr,double,3) = listeLiaisons[liaison].translationStatique;
  //  MAL_VECTOR_DIM(r,double,3) = listeLiaisons[liaison].axeRotationStatique;
  //  glTranslated(tr[0], tr[1], tr[2]);
  //  glRotated(listeLiaisons[liaison].angleRotationStatique, r[0], r[1], r[2]);
  for (unsigned int i=0; i<listeLiaisons[liaison].listeTransformation.size(); i++) {
    transfo t = listeLiaisons[liaison].listeTransformation[i];
    switch (t.type) {
    case ROTATION :
      //			cout << "done" << endl;
      //      glRotated(t.quantite, t.axe[0], t.axe[1], t.axe[2]);
      break;
    case TRANSLATION :
      //      glTranslated(t.quantite*t.axe[0], t.quantite*t.axe[1], t.quantite*t.axe[2]);
      break;
    case ROTATION_LIBRE :
      //      glMultMatrixf(t.rotation());
      break;
    default :
      cout << "attention, transformation non prise en charge !" << endl;
    }
  }
}

inline void DynamicMultiBody::empilerTransformationsLiaisonInverse(int liaison)
{
  //MAL_VECTOR_DIM(tr,double,3) = -listeLiaisons[liaison].translationStatique;
  //  MAL_VECTOR_DIM(r,double,3) = listeLiaisons[liaison].axeRotationStatique;
  for (unsigned int i=0; i<listeLiaisons[liaison].listeTransformation.size(); i++) {
    transfo t = listeLiaisons[liaison].listeTransformation[listeLiaisons[liaison].listeTransformation.size()-i-1];
    switch (t.type) {
    case ROTATION :
      //      glRotated(-t.quantite, t.axe[0], t.axe[1], t.axe[2]);
      break;
    case TRANSLATION :
      //      glTranslated(-t.quantite*t.axe[0], -t.quantite*t.axe[1], -t.quantite*t.axe[2]);
      break;
    case ROTATION_LIBRE :
      {
	//  matrix4f m = (((matrix4f(t.rotation())).transpose())
	//  .inverse_tranformation()).transpose();
	//	glMultMatrixf(m.m);
      }
      break;
    default :
      cout << "attention, transformation non prise en charge !" << endl;
    }
  }
  //  glRotated(-listeLiaisons[liaison].angleRotationStatique, r[0], r[1], r[2]);
  //  glTranslated(tr[0], tr[1], tr[2]);
}

void DynamicMultiBody::parserVRML(string path, string nom, 
				  const char *option)
{
  listOfBodies.clear();
  MultiBody::parserVRML(path, nom, option);
  listOfBodies.resize(listeCorps.size());
  for(unsigned int i=0;i<listeCorps.size();i++)
    listOfBodies[i] = listeCorps[i];

  ConvertJOINTIDToBodyID.resize(listOfBodies.size());
  SpecifyTheRootLabel(0);
}

void DynamicMultiBody::calculerMatriceTransformationEntre(int corps1, int corps2, float *matrice)
{
  vector<int> v = trouverCheminEntre(corps1, corps2);
  if (v.size() == 0) {
    matrice = NULL;
    return;
  }

  //  glPushMatrix();
  //  glLoadIdentity();

  int corpsCourant = corps1;
  for (unsigned int i=0; i<v.size(); i++) {
    cout << "i = " << i << endl;
    //il s'agit de savoir si on lit la liaison dans le bon sens :
    //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
    if (corpsCourant == listeLiaisons[v[i]].indexCorps1) {	
      // on est dans le bon sens
      empilerTransformationsLiaisonDirecte(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps2;
    }
    else {
      // il faut "lire" la liaison a l'envers
      empilerTransformationsLiaisonInverse(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps1;
    }
  }

  //  glGetFloatv(GL_MODELVIEW_MATRIX, matrice);
  for (int i=0; i<16; i++) {
    cout << matrice[i] << "  ";
  }
  cout << endl;
  //  glPopMatrix();
  return;
	
}

void DynamicMultiBody::calculerMatriceTransformationEntre(int corps1, int corps2, double *matrice)
{

  //	cout << "entree calculerMatrice" << endl;

  vector<int> v = trouverCheminEntre(corps1, corps2);
  if (v.size() == 0) {
    matrice = NULL;
    cout << "pas de chemin entre " << corps1 << " et " << corps2 << endl;
    return;
  }

  //  glPushMatrix();
  // glLoadIdentity();

  int corpsCourant = corps1;
  for (unsigned int i=0; i<v.size(); i++) {
    //		cout << "i = " << i << endl;
    //il s'agit de savoir si on lit la liaison dans le bon sens :
    //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
    if (corpsCourant == listeLiaisons[v[i]].indexCorps1) {	
      // on est dans le bon sens
      empilerTransformationsLiaisonDirecte(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps2;
    }
    else {
      // il faut "lire" la liaison a l'envers
      empilerTransformationsLiaisonInverse(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps1;
    }

    /*		glGetDoublev(GL_MODELVIEW_MATRIX, matrice);
		for (int i=0; i<16; i++) {
		cout << matrice[i] << "  ";
		}
		cout << endl;*/
  }



  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);
  /*	for (int i=0; i<16; i++) {
	cout << matrice[i] << "  ";
	}
	cout << endl;*/
  //  glPopMatrix();
  return;
	
}

vector<int> DynamicMultiBody::trouverCheminEntre(int corps1, int corps2)
{
  if (corps1 < 0 || (unsigned int)corps1 >= listeCorps.size() || corps2 < 0 || (unsigned int)corps2 >= listeCorps.size()) {
    cout << "Indice hors limites lors de l'appel de trouverCheminEntre(" << corps1 << ", " << corps2 << ")" <<endl;
    return vector<int>();
  }
  for (unsigned int i=0; i<liaisons[corps1].size(); i++) {
    vector<int> v = vector<int>();
    int c1 = listeLiaisons[liaisons[corps1][i].liaison].indexCorps1;
    int c2 = listeLiaisons[liaisons[corps1][i].liaison].indexCorps2;
    trouverCheminEntreAux((c1==corps1)?c2:c1, corps2, liaisons[corps1][i].liaison, v);
    if (v.size() !=0 ) {
      return v;
    }
  }

  cout << "aucun chemin trouve entre le corps " << corps1 << " et le corps " << corps2 << endl;
  return vector<int>();
}

void DynamicMultiBody::trouverCheminEntreAux(int corpsCourant, int corpsVise, int liaisonDeProvenance, vector<int> &chemin)
{
  if (corpsCourant == corpsVise) {
    chemin.push_back(liaisonDeProvenance);
    return;
  }

  chemin.push_back(liaisonDeProvenance);

  switch (liaisons[corpsCourant].size()) {
  case 1 :
    break;
  case 2 :
    if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance) {
      int liaisonDestination = liaisons[corpsCourant][1].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
      }
      else {
	trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
      }
    }
    else {
      int liaisonDestination = liaisons[corpsCourant][0].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
      }
      else {
	trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
      }
    }
    break;
  default : //on a plus de deux liaisons
    for (unsigned int i=0; i<liaisons[corpsCourant].size(); i++) {
      if (liaisons[corpsCourant][i].liaison == liaisonDeProvenance) 
	continue;
      int liaisonDestination = liaisons[corpsCourant][i].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
      }
      else {
	trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
      }
    }
    break;
  }

  if (chemin.size() != 0) {
    if (listeLiaisons[chemin[chemin.size()-1]].indexCorps1 == corpsVise || listeLiaisons[chemin[chemin.size()-1]].indexCorps2 == corpsVise) {
      return;
    }
  }

  chemin.pop_back();

}

void DynamicMultiBody::changerCorpsInitial(int nouveauCorps)
{

  if (nouveauCorps <= 0 || (unsigned int)nouveauCorps >= listeCorps.size()) {
    cout << "Impossible de choisir corps " << nouveauCorps << " comme nouveau corps initial" <<endl;
    return;
  }
  int corpsInitActuel = listeLiaisons[0].indexCorps2;
  //on enleve la liaison 0 de corpsInitActuel
  for (vector<appariement>::iterator it = liaisons[corpsInitActuel].begin(); 
       it!=liaisons[corpsInitActuel].end(); it++) {
    if ((*it).liaison ==0) {
      liaisons[corpsInitActuel].erase(it);
      it--;
    }
  }
  //on change l'appariement du corps 0
  liaisons[0][0].corps = nouveauCorps;
  //on ajoute l'appariement pour nouveauCorps
  appariement a = {0, 0};
  liaisons[nouveauCorps].push_back(a);
  //on change indexCorps2 pour la liaison 0
  listeLiaisons[0].indexCorps2 = nouveauCorps;
  //on change les transformations de la liaison 0
  float matrice[16];
  calculerMatriceTransformationEntre(corpsInitActuel, nouveauCorps, matrice);
  //  ml(0,0)=0.0;

  MAL_MATRIX_DIM(m1,double,4,4);

  float *lprot = listeLiaisons[0].listeTransformation[3].rotation();
  for(unsigned int li=0;li<4;li++)
    for(unsigned int lj=0;lj<4;lj++)
      m1(li,lj)= lprot[li*4+lj];

  m1(4,0) = listeLiaisons[0].listeTransformation[0].quantite;
  m1(4,1) = listeLiaisons[0].listeTransformation[1].quantite;
  m1(4,2) = listeLiaisons[0].listeTransformation[2].quantite;

  MAL_MATRIX(m2,double);
  MAL_MATRIX_RESIZE(m2,4,4);
  for(unsigned int li=0;li<4;li++)
    for(unsigned int lj=0;lj<4;lj++)
      m2(li,lj)= matrice[li*4+lj];
  
  MAL_MATRIX_DIM(matf,double,4,4);
  MAL_C_eq_A_by_B(matf,m2,m1);
  listeLiaisons[0].listeTransformation[0].quantite = matf(4,0);
  listeLiaisons[0].listeTransformation[1].quantite = matf(4,1);
  listeLiaisons[0].listeTransformation[2].quantite = matf(4,2);

  for (int i=0; i<3; i++) 
    {
      for(int j=0;j<4;j++)
	listeLiaisons[0].listeTransformation[3].rotation()[i] = matf(i,j);
    }
  listeLiaisons[0].listeTransformation[3].rotation()[12] = 0;
  listeLiaisons[0].listeTransformation[3].rotation()[13] = 0;
  listeLiaisons[0].listeTransformation[3].rotation()[14] = 0;
}

// TODO: Remove any reference to OpenGL and makes this function right.
int DynamicMultiBody::ComputeJacobian(int corps1, int corps2, 
				      MAL_S3_VECTOR(,double) coordLocales, 
				      double *jacobienne[6])
{
  vector<int> chemin = trouverCheminEntre(corps1, corps2);
  MAL_VECTOR(ve,double);
  double translationInit[3];
  double axe[3],angle=0.0;
  int l = chemin.size();
  if (l == 0) {
    jacobienne = NULL;
    return 1;
  }

  double *matrice = new double[16];

  //  glPushMatrix();
  //  glLoadIdentity();
  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);

  int corpsCourant = corps1;
  for (int i=0; i<l; i++) {
    /*		for (int j=0; j<16; j++) {
		cout << matrice[j] << "  ";
		}*/
    //		cout << endl;
    //		cout << "i = " << i << endl;
    //il s'agit de savoir si on lit la liaison dans le bon sens :
    //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
    if (corpsCourant == listeLiaisons[chemin[i]].indexCorps1) {
      // on est dans le bon sens
      for (unsigned int j=0; j<listeLiaisons[chemin[i]].listeTransformation.size(); j++) {
	//				cout << "bon sens" << endl;
	transfo t = listeLiaisons[chemin[i]].listeTransformation[j];
	switch (t.type) {
	case ROTATION :
	  //					cout << "rotation : " << t.quantite << endl;
	  //ve = matrice*(t.axe);
	  jacobienne[3][i] = ve[0]-matrice[12];
	  jacobienne[4][i] = ve[1]-matrice[13];
	  jacobienne[5][i] = ve[2]-matrice[14];
	  break;
	case ROTATION_LIBRE :
	  //	  Matrix2AxeAngle(t.rotation, axe, angle);
	  for (int j=0; j<16; j++) {
	    cout << t.rotation()[j] << "  ";
	  }
	  cout << endl;
	  jacobienne[3][i] = axe[0];
	  jacobienne[4][i] = axe[1];
	  jacobienne[5][i] = axe[2];
	  cout << axe[0] << " " << axe[1] << " " << axe[3] << "  angle : " << angle << endl;
	  break;
	case TRANSLATION :
	  //A faire
	  break;
	default :
	  cout << "attention, transformations non prises en charge lors du calcul de la jacobienne !" << endl;
	}
      }
      empilerTransformationsLiaisonDirecte(chemin[i]);
      corpsCourant = listeLiaisons[chemin[i]].indexCorps2;
    }
    else {
      // il faut "lire" la liaison a l'envers
      for (unsigned int j=0; j<listeLiaisons[chemin[i]].listeTransformation.size(); j++) {
	//				cout << "sens inverse" << endl;
	transfo t = listeLiaisons[chemin[i]].listeTransformation[j];
	switch (t.type) {
	case ROTATION :
	  //	  ve = matrice*(t.axe);
	  jacobienne[3][i] = -ve[0]+matrice[12];//rajouter +matrice[12] ?
	  jacobienne[4][i] = -ve[1]+matrice[13];
	  jacobienne[5][i] = -ve[2]+matrice[14];
	  break;
	case ROTATION_LIBRE :
	  //	  Matrix2AxeAngle(t.rotation, axe, angle);
	  jacobienne[3][i] = -axe[0];
	  jacobienne[4][i] = -axe[1];
	  jacobienne[5][i] = -axe[2];
	  break;
	case TRANSLATION :
	  //A faire
	  break;
	default :
	  cout << "attention, transformations non prises en charge lors du calcul de la jacobienne !" << endl;
	}
      }
      empilerTransformationsLiaisonInverse(chemin[i]);
      corpsCourant = listeLiaisons[chemin[i]].indexCorps1;
    }
    //    glGetDoublev(GL_MODELVIEW_MATRIX, matrice);

	
    //recuperation des vecteurs
    if (i == 0) {
      /*				cout <<  "translation init" << endl;
					cout << (translationInit[0] = matrice[12]) << endl;
					cout << (translationInit[1] = matrice[13]) << endl;
					cout << (translationInit[2] = matrice[14]) << endl;*/
      translationInit[0] = matrice[12];
      translationInit[1] = matrice[13];
      translationInit[2] = matrice[14];
    }
    else {
      jacobienne[0][i-1] = matrice[12] - translationInit[0];
      jacobienne[1][i-1] = matrice[13] - translationInit[1];
      jacobienne[2][i-1] = matrice[14] - translationInit[2];
    }

    /*
      MAL_VECTOR_DIM(translationOriente,double,3)1(matrice[12],matrice[13],matrice[14]);
      MAL_VECTOR_DIM(translationOriente,double,3)2 = matrice*translationOriente1 - translationOriente1;

      jacobienne[0][i] = translationOriente1[0];
      jacobienne[1][i] = translationOriente1[1];
      jacobienne[2][i] = translationOriente1[2];
    */
  }
  //glPopMatrix();
  /*	cout << endl;
	for (int j=0; j<16; j++) {
	cout << matrice[j] << "  ";
	}
	cout << endl;
	cout << endl;*/
  //a present, il faut recalculer les vecteurs : on a en effet les trois premieres composantes de chaque 
  //colonnes de la jacobienne qui correspondent aux vecteurs depuis l'origine jusqu'a un joint, alors qu'on
  //cherche a avoir le vecteur de ce joint a l'extremite
  //MAL_VECTOR_DIM(point,double,3)(0,0,0);
  MAL_S3_VECTOR(point,double);
  point = coordLocales;
  //  point = matrice*point;
  for(int i=0;i<3;i++)
    cout << point[i] << " " ;
  cout << endl;
  point[0] -= translationInit[0];
  point[1] -= translationInit[1];
  point[2] -= translationInit[2];
  for(int i=0;i<3;i++)
    cout << point[i] << " " ;
  cout << endl;

  /*
    cout << endl;
    cout << "*-*-*" << endl;
    for (int i=0; i<6; i++) {
    for (int j=0; j<l; j++) {
    if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
    cout << " " << 0 << "  ";
    }
    else {
    cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
    }
    }
    cout << endl;
    }
    cout << endl;
  */

  for (int i=l-1; i>0; i--) {
    jacobienne[0][i] = point[0] - jacobienne[0][i-1];
    jacobienne[1][i] = point[1] - jacobienne[1][i-1];
    jacobienne[2][i] = point[2] - jacobienne[2][i-1];
  }
  jacobienne[0][0] = point[0];
  jacobienne[1][0] = point[1];
  jacobienne[2][0] = point[2];

  cout << endl;
  cout << "***" << endl;
  /*	for (int i=0; i<6; i++) {
	for (int j=0; j<l; j++) {
	if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
	cout << " " << 0 << "  ";
	}
	else {
	cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
	}
	}
	cout << endl;
	}
	cout << endl;
  */
  //on calcul les produits vectoriels (j[3],j[4],j[5])*(j[0],j[1],j[2])
  double pvx, pvy, pvz;
  for (int i=0; i<l; i++) {
    pvx = jacobienne[2][i]*jacobienne[4][i] - jacobienne[1][i]*jacobienne[5][i];
    pvy = jacobienne[0][i]*jacobienne[5][i] - jacobienne[2][i]*jacobienne[3][i];
    pvz = jacobienne[1][i]*jacobienne[3][i] - jacobienne[0][i]*jacobienne[4][i];
    jacobienne[0][i] = pvx;
    jacobienne[1][i] = pvy;
    jacobienne[2][i] = pvz;
  }


  for (int i=0; i<6; i++) {
    for (int j=0; j<l; j++) {
      if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
	cout << " " << 0 << "  ";
      }
      else {
	cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
      }
    }
    cout << endl;
  }

  return 0;
}

void DynamicMultiBody::ComputeJacobianWithPath(vector<int> aPath,MAL_MATRIX(&J,double))
{
  int last=aPath.back();
  //  cout << "last "<< last <<endl;
  MAL_S3_VECTOR(target,double);
  target = listOfBodies[ConvertJOINTIDToBodyID[last]].p;
  //  cout << "target " << endl
  //       <<target << endl;
  for (unsigned int i=0;i<aPath.size();i++)
    {
      int li = aPath[i];
      //cout << "li " << li << endl;
      DynamicBody *aDB = &listOfBodies[ConvertJOINTIDToBodyID[li]];

      //  cout << aDB->getName() << endl;
      /* cout << "R:" << endl << aDB->R<< endl <<"a: " << aDB->a << endl
	 << "p:" << endl; */

      MAL_S3_VECTOR(wa,double);
      MAL_S3x3_C_eq_A_by_B(wa , aDB->R, aDB->a);
      MAL_S3_VECTOR(cwa,double);
      MAL_S3_VECTOR_CROSS_PRODUCT(cwa ,wa, (target - aDB->p));
      for(int j=0;j<3;j++)
	J(j,i) = cwa[j];
      for(int j=0;j<3;j++)
	J(j+3,i) = wa[j];
      /*
      cout << "wa: "<<endl
	   << wa << endl
	   << "cwa: " <<endl
	   << cwa << endl
	   << "target - p " <<endl
	   << target- aDB->p << endl;
      */
      
    }
  //  cout << "J " << endl << J<< endl;

}

MAL_S3_VECTOR(,double) 
  DynamicMultiBody::getPositionPointDansRepere(MAL_S3_VECTOR(,double) point, 
					       int corpsDuPoint, int corpsDuRepere)
{
  //	cout << "entree getPosition" << endl;
  double matrice[16];
  calculerMatriceTransformationEntre(corpsDuRepere, corpsDuPoint, matrice);
  //  return (matrice*point);
  return point;
}

double DynamicMultiBody::Getq(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].q;
  return 0.0;
}

void DynamicMultiBody::Setq(int JointID,double q)
{
 if ((JointID>=0) &&
     ((unsigned int)JointID<listOfBodies.size()))
   {  
     listOfBodies[ConvertJOINTIDToBodyID[JointID]].q= q;
   }
}

void DynamicMultiBody::Setdq(int JointID,double dq)
{
 if ((JointID>=0) &&
     ((unsigned int)JointID<listOfBodies.size()))
   {  
     listOfBodies[ConvertJOINTIDToBodyID[JointID]].dq= dq;
   }
}

void DynamicMultiBody::Setv(int JointID, MAL_S3_VECTOR(,double) v0)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertJOINTIDToBodyID[JointID]].v0 = v0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getv(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].v0;
  
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetvBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID].v0;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetwBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID].w;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getw(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].w;
 
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

void DynamicMultiBody::Setw(int JointID, MAL_S3_VECTOR(,double) w)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertJOINTIDToBodyID[JointID]].w = w;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getp(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].p;
  return empty;
}

void DynamicMultiBody::Setp(int JointID, MAL_S3_VECTOR(,double) apos)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertJOINTIDToBodyID[JointID]].p = apos;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::getPositionCoM(void)
{
  return (positionCoMPondere/masse);
}

void DynamicMultiBody::GetPandL(MAL_S3_VECTOR(,double) &aP, MAL_S3_VECTOR(,double) &aL)
{
  aP = m_P;
  aL = m_L;
}

void DynamicMultiBody::CalculateZMP(double &px, double &py,
				    MAL_S3_VECTOR(,double) dP, MAL_S3_VECTOR(,double) dL, double zmpz)
{
  double g= 9.80665;
  
  px = ( g * positionCoMPondere[0] +
	 zmpz * dP[0] - dL[1])/(masse * g + dP[2]);
  py = ( g * positionCoMPondere[1] +
	 zmpz * dP[1] + dL[0])/(masse * g + dP[2]);

  /*  cout << "Masse :"<< masse << " g:" << g 
      << dP << " " << dL << " " << endl; */
  //  cout << "px :"<< px<< " py: " << py<< endl;

}

string DynamicMultiBody::GetName(int JointID)
{
  string empty;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].getName();
  return empty;
    
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetL(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].L;
  return empty;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetP(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].P;
  return empty;
}

double DynamicMultiBody::Getdq(int JointID)
{
  double empty=0.0;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertJOINTIDToBodyID[JointID]].dq;
  return empty;
}

void DynamicMultiBody::BuildSplittedInertialMatrices(  vector<int> LeftLeg, vector<int> RightLeg,
						       int WaistIndex, vector<int> FreeJoints)
{
  MAL_MATRIX(aJacobian,double);
  MAL_MATRIX(IaJacobian,double);
  MAL_MATRIX(MHlegi,double);
  // Build M*Fi and H*Fi

  MAL_MATRIX_RESIZE(aJacobian,6,LeftLeg.size());

  ComputeJacobianWithPath(LeftLeg,aJacobian);


  //  cout << "Jacobian Left Foot"<< aJacobian << endl;
  //  cout << "Inverse Jacobian"<< Inverse(aJacobian) << endl;
  MAL_MATRIX_RESIZE(MHlegi,6,LeftLeg.size());
  int lindex;
  for(unsigned int i=0;i<LeftLeg.size();i++)
    {
      lindex =ConvertJOINTIDToBodyID[LeftLeg[i]];
      //cout << listOfBodies[lindex].getName() <<endl;
      // M 
      MHlegi(0,i) = listOfBodies[lindex].m_RMC_m[0];
      MHlegi(1,i) = listOfBodies[lindex].m_RMC_m[1];
      MHlegi(2,i) = listOfBodies[lindex].m_RMC_m[2];
      // H
      MHlegi(3,i) = listOfBodies[lindex].m_RMC_h[0];
      MHlegi(4,i) = listOfBodies[lindex].m_RMC_h[1];
      MHlegi(5,i) = listOfBodies[lindex].m_RMC_h[2];
      
    }
  //  cout << "MH for LeftFoot"<< MHlegi << endl;
  MAL_INVERSE(aJacobian, m_ILeftJacobian,double);

  MAL_C_eq_A_by_B(m_MHStarLeftFoot, MHlegi, m_ILeftJacobian);

  MAL_MATRIX_RESIZE(aJacobian,6,RightLeg.size());
  ComputeJacobianWithPath(RightLeg,aJacobian);
  //  cout << "Jacobian Right Foot"<< aJacobian << endl;
  //  cout << "Inverse Jacobian"<< Inverse(aJacobian) << endl;

  MAL_MATRIX_RESIZE(MHlegi,6,RightLeg.size());
  for(unsigned int i=0;i<RightLeg.size();i++)
    {
      lindex = ConvertJOINTIDToBodyID[RightLeg[i]];
      //      cout << listOfBodies[lindex].getName() <<endl;
      // M 
      MHlegi(0,i) = listOfBodies[lindex].m_RMC_m[0];
      MHlegi(1,i) = listOfBodies[lindex].m_RMC_m[1];
      MHlegi(2,i) = listOfBodies[lindex].m_RMC_m[2];
      // H
      MHlegi(3,i) = listOfBodies[lindex].m_RMC_h[0];
      MHlegi(4,i) = listOfBodies[lindex].m_RMC_h[1];
      MHlegi(5,i) = listOfBodies[lindex].m_RMC_h[2];
      
    }
  //  cout << "MH for RightFoot"<< MHlegi << endl;
  //  cout << "MH for LeftFoot"<< MHlegi << endl;

  MAL_MATRIX(mIRightJacobian,double);
  MAL_INVERSE(aJacobian, mIRightJacobian,double);
  MAL_C_eq_A_by_B( m_MHStarRightFoot, MHlegi, m_IRightJacobian);

  //  cout << " m_MHStarLeftFoot" << m_MHStarLeftFoot << endl;
  //cout << " m_MHStarRightFoot" << m_MHStarRightFoot << endl;

  // Create M*B

  // Create the variables of equation 10 et 11 given pages 1645
  MAL_S3_VECTOR(rbCoM,double);
  rbCoM = positionCoMPondere/masse - listOfBodies[WaistIndex].p;
  
  //  cout << "rbCoM : " << endl << rbCoM << " "
  //       << positionCoMPondere/masse << endl <<  listOfBodies[WaistIndex].p << endl;
  MAL_MATRIX_DIM(FirstTerm,double,6,6);
  
  FirstTerm(0,0) =   FirstTerm(1,1) =  FirstTerm(2,2) =  masse;
  
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      FirstTerm(i+3,j+3) = listOfBodies[labelTheRoot].m_tildeI(i,j);
  
  FirstTerm(0,3) = 0;              FirstTerm(0,4) = masse* rbCoM[2]; FirstTerm(0,5) = -masse * rbCoM[1];
  FirstTerm(1,3) = -masse*rbCoM[2]; FirstTerm(1,4) =              0; FirstTerm(1,5) =  masse * rbCoM[0];
  FirstTerm(2,3) =  masse*rbCoM[1]; FirstTerm(2,4) =-masse* rbCoM[0]; FirstTerm(2,5) =                0;

  MAL_MATRIX_DIM(SecondTerm,double,6,6);
  MAL_MATRIX_DIM(ThirdTerm,double,6,6);

  MAL_S3_VECTOR(rbfi,double);
  rbfi = listOfBodies[ConvertJOINTIDToBodyID[LeftLeg.back()]].p - listOfBodies[WaistIndex].p;
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      if (i==j)
	SecondTerm(i,j) = 1.0;
      else 
	SecondTerm(i,j) = 0.0;

  SecondTerm(0,3) =       0; SecondTerm(0,4) = -rbfi[2]; SecondTerm(0,5) = rbfi[1];
  SecondTerm(1,3) =  rbfi[2]; SecondTerm(1,4) =       0; SecondTerm(1,5) =-rbfi[0];
  SecondTerm(2,3) = -rbfi[1]; SecondTerm(2,4) =  rbfi[0]; SecondTerm(2,5) =       0;
  // To compute the leg joint velocity... hello ramzi ! .. did you find me ?
  m_ERBFI_Left = SecondTerm;
  
  MAL_C_eq_A_by_B(ThirdTerm,m_MHStarLeftFoot,SecondTerm);
  ThirdTerm = -1.0 * ThirdTerm;


  rbfi = listOfBodies[ConvertJOINTIDToBodyID[RightLeg.back()]].p - listOfBodies[WaistIndex].p;
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      if (i==j)
	SecondTerm(i,j) = 1.0;
      else 
	SecondTerm(i,j) = 0.0;

  SecondTerm(0,3) =       0; SecondTerm(0,4) = -rbfi[2]; SecondTerm(0,5) = rbfi[1];
  SecondTerm(1,3) =  rbfi[2]; SecondTerm(1,4) =       0; SecondTerm(1,5) =-rbfi[0];
  SecondTerm(2,3) = -rbfi[1]; SecondTerm(2,4) =  rbfi[0]; SecondTerm(2,5) =       0;
  // To compute the leg joint velocity... hello ramzi ! .. did you find me (again)?
  m_ERBFI_Right = SecondTerm;

  MAL_MATRIX(tmp3rdTerm,double);
  MAL_C_eq_A_by_B(tmp3rdTerm, m_MHStarRightFoot,SecondTerm);
  ThirdTerm = ThirdTerm - tmp3rdTerm;

  ODEBUG3("FirstTerm " << endl << FirstTerm);
  ODEBUG3("ThirdTerm " << endl << ThirdTerm);
  m_MHStarB = FirstTerm + ThirdTerm;

  /*  cout << "MHStarB" <<endl 
      << m_MHStarB << endl;
  */
  MAL_MATRIX_RESIZE(m_MHFree,6,FreeJoints.size());

  for(unsigned int i=0;i<FreeJoints.size();i++)
    {
      // M 
      m_MHFree(0,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_m[0];
      m_MHFree(1,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_m[1];
      m_MHFree(2,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_m[2];
      // H
      m_MHFree(3,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_h[0];
      m_MHFree(4,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_h[1];
      m_MHFree(5,i) = listOfBodies[ConvertJOINTIDToBodyID[FreeJoints[i]]].m_RMC_h[2];
      
    }

  //  cout << "MHFree " <<endl
  //    << m_MHFree << endl;
}

void DynamicMultiBody::BuildLinearSystemForRMC(MAL_MATRIX( &PLref, double),
					       MAL_MATRIX(&XiLeftFootRef,double),
					       MAL_MATRIX(&XiRightFootRef,double),
					       int NbOfFreeJoints,
					       MAL_MATRIX(&S,double),
					       MAL_MATRIX(&XiBdThetaFreeRef,double),
					       MAL_MATRIX(&XiBdThetaFree,double),
					       MAL_MATRIX(&LeftLegVelocity,double),
					       MAL_MATRIX(&RightLegVelocity,double))
{
  // Building y (eq 10 p 1646, Kajita IROS 2003)
  MAL_MATRIX_DIM(TmpVector,double, 6,1);
  MAL_MATRIX_DIM(Tmp2,double,6,1);
  MAL_MATRIX_DIM(Tmp3,double,6,1);
  MAL_MATRIX_DIM(Tmp4,double,6,1);
  
  MAL_MATRIX_CLEAR(TmpVector);

  //  cout << "PLref:" <<endl << PLref << endl;
  Tmp2 = PLref; 

  //  cout << "Tmp2 1: " << Tmp2 << endl;
  
  Tmp2 -= MAL_RET_A_by_B(m_MHStarLeftFoot,XiLeftFootRef);
  //  cout << "Tmp2 2: " << Tmp2 << endl;
  Tmp2 -= MAL_RET_A_by_B(m_MHStarRightFoot, XiRightFootRef);
  //  cout << "Tmp2 3: " << Tmp2 << endl;

  MAL_MATRIX_RESIZE(TmpVector, 
		    MAL_MATRIX_NB_ROWS(Tmp2),
		    MAL_MATRIX_NB_COLS(Tmp2));

  for(unsigned int i=0;i<6;i++)
    TmpVector(i,0) = Tmp2(i,0);

  //  cout << TmpVector<< endl;

  MAL_MATRIX(y,double);
  y = MAL_RET_A_by_B(S , TmpVector);
 
  //  cout << "y " << endl << y << endl;
  cout << "m_MHStarB " << endl
       << m_MHStarB << endl;
  cout << "m_MHFree " << endl
       << m_MHFree << endl;

  MAL_MATRIX_DIM(TmpMatrix,double,6, 6+ NbOfFreeJoints);

  for(unsigned int i=0;i<6;i++)
    {
      for(unsigned int j=0;j<6;j++)
	{
	  TmpMatrix(i,j) = m_MHStarB(i,j);
	}
      for(unsigned int j=0;j<(unsigned int)NbOfFreeJoints;j++)
	{
	  TmpMatrix(i,j+5) = m_MHFree(i,j);
	}
    }

  MAL_MATRIX(A,double);

  A = MAL_RET_A_by_B(S , TmpMatrix);
  
  cout << "S " << endl << S << endl;
  cout << "TmpMatrix " << endl << TmpMatrix << endl;
  cout << "A " << endl << A << endl;
  
  MAL_MATRIX(PinvA,double);
  MAL_PSEUDOINVERSE(A,PinvA,double);

  //  cout << "y" << endl << y << endl;
  //  cout << "PinvA: " << endl << PinvA<< endl;
  
  MAL_MATRIX(KernA,double);
  MAL_MATRIX_DIM(E,double,
		 MAL_MATRIX_NB_ROWS(PinvA),
		 MAL_MATRIX_NB_COLS(A));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(E);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(E);j++)
      if (i==j)
	E(i,j)=1.0;
      else
	E(i,j)=0.0;

  KernA = E - MAL_RET_A_by_B(PinvA , A);
  
  MAL_MATRIX_DIM(lXiBdThetaFree,double,
		 MAL_MATRIX_NB_ROWS(XiBdThetaFreeRef),
		 MAL_MATRIX_NB_COLS(XiBdThetaFreeRef));
  
    //cout << "KernA" << endl << KernA.Rows() << " " << KernA.Columns() << endl;
  //  cout << "KernA * lXiBdThetaFreeRef" << endl << KernA* lXiBdThetaFreeRef << endl;
  lXiBdThetaFree = 
    MAL_RET_A_by_B(PinvA , y)
    + MAL_RET_A_by_B(KernA , XiBdThetaFreeRef);

  MAL_MATRIX_DIM(XiBd,double,6,1);
  MAL_MATRIX_RESIZE(XiBdThetaFree,
		    MAL_MATRIX_NB_ROWS(lXiBdThetaFree),
		    MAL_MATRIX_NB_COLS(lXiBdThetaFree));

  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(lXiBdThetaFree);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(lXiBdThetaFree);j++)
      {
	XiBdThetaFree(i,j)=lXiBdThetaFree(i,j);
	if ((i<6) && (j==0))
	  XiBd(i,j) = lXiBdThetaFree(i,j);
	  
      }

  // Computes the new leg joints velocities. (eq 3) 
  LeftLegVelocity = MAL_RET_A_by_B(m_ERBFI_Left , XiBd);
  LeftLegVelocity = MAL_RET_A_by_B(m_ILeftJacobian , XiLeftFootRef)
    - MAL_RET_A_by_B(m_ILeftJacobian ,LeftLegVelocity);

  RightLegVelocity = MAL_RET_A_by_B(m_ERBFI_Right , XiBd);
  RightLegVelocity = MAL_RET_A_by_B(m_IRightJacobian , XiRightFootRef) 
    - MAL_RET_A_by_B(m_IRightJacobian ,RightLegVelocity);

}

void DynamicMultiBody::SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))  
    listOfBodies[BodyID].R = R;
}
