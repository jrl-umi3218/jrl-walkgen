#include <iostream>
#include <math.h>
#include "HRP2DisplayGL.h"

HRP2DisplayGL::HRP2DisplayGL(QWidget *parent, const char *name) : QGLWidget(parent,name)
{
  m_corpsDeDepart =1 ;
  move(10,10);
  setBaseSize(QSize(640,480));
  m_TranslateX  = 0.156863;
  m_TranslateY  = -0.470588;
  m_TranslateZ  = -3.21;
  m_fRotX = 109;
  m_fRotY = 180;
  m_fRotZ = 87;
  m_alpha = 0.1;
  m_RightGripperType  = 1;
  m_LeftGripperType  = 1;
  double lWaistPosture[16]= { 1.0, 0.0, 0.0, 0.0,
			      0.0, 1.0, 0.0, 0.0,
			      0.0, 0.0, 1.0, 0.0,
			      0.0, 0.0, 0.705, 1.0};
  for(int i=0;i<16;i++)
    m_WaistPosture[i] = lWaistPosture[i];


  // Right Leg
  for(int i=0;i<6;i++)
    m_RightLeg[i] = 0.0;

  // Left Leg 
  for(int i=0;i<6;i++)
    m_lleg[i] = 0.0;
 
  // Chest  
  for(int i=0;i<2;i++)
    m_chest[i] = 0.0;

  // Head 
  for(int i=0;i<2;i++)
    m_head[i] = 0.0;

  // Right arm
  for(int i=0;i<7;i++)
    m_rarm[i] = 0.0;

  // Left arm
  for(int i=0;i<7;i++)
    m_larm[i] = 0.0;

}

HRP2DisplayGL::~HRP2DisplayGL()
{
}


void HRP2DisplayGL::resizeGL(int width, int height)
{ 

  GLfloat w = (float) width / (float) height;
  GLfloat h = 1.0;

  glViewport(0, 0, width, height);
  resize(width,height);
}

void HRP2DisplayGL::SetWaistPosture(double * aPosture)
{
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      m_WaistPosture[i*4+j] = aPosture[j*4+i];

}

void HRP2DisplayGL::SetMotorsAndWaistPosture(double * Motors,double * WaistPosture)
{
  SetMotors(Motors);
  SetWaistPosture(WaistPosture);
}

void HRP2DisplayGL::SetMotors(double * Motors)
{
  //  std::cout << "SetMotors" << std::endl;
  // Follow the order from bodyinfo.h
  int GlobalIndex=0;

  // Right Leg
  for(int i=0;i<6;i++)
    m_RightLeg[i] = Motors[GlobalIndex++]*180.0/M_PI;

  // Left Leg 
  for(int i=0;i<6;i++)
    m_lleg[i] = Motors[GlobalIndex++]*180.0/M_PI;
 
  // Chest  
  for(int i=0;i<2;i++)
    m_chest[i] = Motors[GlobalIndex++]*180.0/M_PI;

  // Head 
  for(int i=0;i<2;i++)
    m_head[i] = Motors[GlobalIndex++]*180.0/M_PI;

  // Right arm
  for(int i=0;i<7;i++)
    m_rarm[i] = Motors[GlobalIndex++]*180.0/M_PI;

  // Left arm
  for(int i=0;i<7;i++)
    m_larm[i] = Motors[GlobalIndex++]*180.0/M_PI;

  updateGL();
}

void HRP2DisplayGL::initializeGL(void)
{
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  // Initialise light source
  initLighting();
 
  // Other initialisations
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glLineWidth(1.0);
  glPointSize(1.0);
  glEnable(GL_POINT_SMOOTH);
  
  glEnable(GL_COLOR_MATERIAL);

  //  if (settings.f_back_faces)
  glEnable(GL_CULL_FACE);
  //  else
  //    glDisable(GL_CULL_FACE);
  
  // Enable Smooth Shading
  //glShadeModel(GL_SMOOTH);
  // Really Nice Perspective Calculations
  //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  
  // Initialise the modelview transformation
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  //  gluLookAt(2.0, 10.0, 2.0, 0.0, 0.0, 0.75, 0, 0, 1);
  glLoadIdentity();
  
  glShadeModel(GL_SMOOTH);

  BuildPlane();
  // Texture stuff
  //  initTexture();
  //cout << "GLMesh::initializeGL(void): end"<<endl;

  LoadOpenGLFiles();
}

void HRP2DisplayGL::LoadOpenGLFiles()
{
  // Body
  m_BODYa= new HRP2DisplayNS::Body("./HRP2Model/BODYa.ASE","r");
  m_BODYb= new HRP2DisplayNS::Body("./HRP2Model/BODYb.ASE","r");
  m_BODYc= new HRP2DisplayNS::Body("./HRP2Model/BODYc.ASE","r");

  // Jambe droite
  m_RLEGLINK0= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK0.ASE","r");
  m_RLEGLINK1= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK1.ASE","r");
  m_RLEGLINK2= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK2.ASE","r");
  m_RLEGLINK3= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK3.ASE","r");
  m_RLEGLINK4= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK4.ASE","r");
  m_RLEGLINK5a= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK5a.ASE","r");
  m_RLEGLINK5b= new HRP2DisplayNS::Body("./HRP2Model/RLEG_LINK5b.ASE","r");

  // Jambe gauche
  m_LLEGLINK0= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK0.ASE","r");
  m_LLEGLINK1= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK1.ASE","r");
  m_LLEGLINK2= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK2.ASE","r");
  m_LLEGLINK3= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK3.ASE","r");
  m_LLEGLINK4= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK4.ASE","r");
  m_LLEGLINK5a= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK5a.ASE","r");
  m_LLEGLINK5b= new HRP2DisplayNS::Body("./HRP2Model/LLEG_LINK5b.ASE","r");

  // Torse
  m_CHESTLINK0= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK0.ASE","r");
  m_CHESTLINK1a= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK1a.ASE","r");
  m_CHESTLINK1b= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK1b.ASE","r");
  m_CHESTLINK1c= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK1c.ASE","r");
  m_CHESTLINK1d= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK1d.ASE","r");
  m_CHESTLINK1e= new HRP2DisplayNS::Body("./HRP2Model/CHEST_LINK1e.ASE","r");

	// Tete
  m_HEADLINK0= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK0.ASE","r");
  m_HEADLINK1a= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK1a.ASE","r");
  m_HEADLINK1b= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK1b.ASE","r");
  m_HEADLINK1c= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK1c.ASE","r");
  m_HEADLINK1d= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK1d.ASE","r");
  m_HEADLINK1e= new HRP2DisplayNS::Body("./HRP2Model/HEAD_LINK1e.ASE","r");

	// Bras droit
  m_RARMLINK0= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK0.ASE","r");
  m_RARMLINK1= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK1.ASE","r");
  m_RARMLINK2a= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK2a.ASE","r");
  m_RARMLINK2b= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK2b.ASE","r");
  m_RARMLINK3a= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK3a.ASE","r");
  m_RARMLINK3b= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK3b.ASE","r");
  m_RARMLINK4= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK4.ASE","r");
 
  // Main droite
  m_RH0= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK5a.ASE", "r");
  m_RH1= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK5b.ASE", "r");
  m_RH2= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK6a.ASE", "r");
  m_RH3= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK6b.ASE", "r");

  m_RG0= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK5.ASE", "r");
  m_RG1= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK5_LOWERa.ASE", "r");
  m_RG2= new HRP2DisplayNS::Body("./HRP2Model//RARM_LINK5_LOWERb.ASE", "r");
  m_RG3= new HRP2DisplayNS::Body("./HRP2Model/RARM_LINK6.ASE",  "r");
  m_RG4= new HRP2DisplayNS::Body("./HRP2Model/RHAND_LINK0.ASE", "r");
  m_RG5= new HRP2DisplayNS::Body("./HRP2Model/RHAND_LINK1.ASE", "r");
  m_RG6= new HRP2DisplayNS::Body("./HRP2Model/RHAND_LINK2.ASE", "r");
  m_RG7= new HRP2DisplayNS::Body("./HRP2Model/RHAND_LINK3.ASE", "r");
  m_RG8= new HRP2DisplayNS::Body("./HRP2Model/RHAND_LINK4.ASE", "r");
	

  // Bras Gauche
  m_LARMLINK0= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK0.ASE","r");
  m_LARMLINK1= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK1.ASE","r");
  m_LARMLINK2a= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK2a.ASE","r");
  m_LARMLINK2b= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK2b.ASE","r");
  m_LARMLINK3a= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK3a.ASE","r");
  m_LARMLINK3b= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK3b.ASE","r");
  m_LARMLINK4= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK4.ASE","r");

  // Main Gauche
  // HumanLike hand
  m_LH0= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK5a.ASE", "r");
  m_LH1= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK5b.ASE", "r");
  m_LH2= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK6a.ASE", "r");
  m_LH3= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK6b.ASE", "r");

  // GripperLike hand
  m_LG0= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK5.ASE", "r");
  m_LG1= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK5_LOWERa.ASE", "r");
  m_LG2= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK5_LOWERb.ASE", "r");
  m_LG3= new HRP2DisplayNS::Body("./HRP2Model/LARM_LINK6.ASE", "r");
  m_LG4= new HRP2DisplayNS::Body("./HRP2Model/LHAND_LINK0.ASE", "r");
  m_LG5= new HRP2DisplayNS::Body("./HRP2Model/LHAND_LINK1.ASE", "r");
  m_LG6= new HRP2DisplayNS::Body("./HRP2Model/LHAND_LINK2.ASE", "r");
  m_LG7= new HRP2DisplayNS::Body("./HRP2Model/LHAND_LINK3.ASE", "r");
  m_LG8= new HRP2DisplayNS::Body("./HRP2Model/LHAND_LINK4.ASE", "r");


  // Fin chargement des objets graphiques
  //-------------------------------------

}
void HRP2DisplayGL::paintGL()
{

  glClearColor(.4f, .4f, .4f, 1.0f);	// Couleur du BackGround
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // save matrix

  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  gluPerspective(90.0,((double)width())/((double)height()),1.0, 500.0); 

#if 0
  float O_xyAspect=1.0, O_Left=-1, O_Right=1, O_Bottom=-1, O_Top=1,
    O_zNear=0.1, O_zFar=10.0;

  glOrtho(O_xyAspect*O_Left, O_xyAspect*O_Right, O_Bottom, O_Top, O_zNear, O_zFar);
#else
  float P_Focal=60.0, P_xyAspect=1.0, P_zNear=0.01, P_zFar=20.0;
  P_xyAspect=(double)width()/(double)height();
  gluPerspective(P_Focal, P_xyAspect, P_zNear, P_zFar);
#endif

  glMatrixMode(GL_MODELVIEW);
  //  glPushMatrix();
  glLoadIdentity();


  /*
  cout << m_TranslateX << " "
       << m_TranslateY << " "
       << m_TranslateZ << " "
       << m_fRotX << " "
       << m_fRotY << " "
       << m_fRotZ << endl;
  */
  glTranslatef(m_TranslateX, m_TranslateY, m_TranslateZ);
  glRotatef(m_fRotX, 1.0, 0.0, 0.0);
  glRotatef(m_fRotY, 0.0, 1.0, 0.0);
  glRotatef(m_fRotZ, 0.0, 0.0, 1.0);


  glEnable(GL_LIGHTING);
#if 0  
  double lMotors[30];
  static unsigned int lIndex=0;
  for(int i=0;i<30;i++)
    {
      lMotors[i] = (double)lIndex;
    }
  SetMotors(lMotors);
  lIndex++;
  if (lIndex==360)
    lIndex=0;
#endif	  
  HRP2_Rendering(1.0);

  glPopMatrix();
  glDisable(GL_LIGHTING);

 
  glBegin(GL_LINES);

  glViewport(0, 0, width(), height());
  
  GLfloat MATVIEW_Center[3] = {0.0,0.0,0.0};
  /* Display scale. */
  glColor3f(0.8,0.8,0.8);
  for(float i=-12*0.250;i<=12*0.250;i+=0.250)
    {
      glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]-3,-MATVIEW_Center[2]);
      glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]+3, -MATVIEW_Center[2]);
    }
  
  for(float i=-12*0.250;i<=12*0.250;i+=0.250)
    {
      glVertex3d(-MATVIEW_Center[0]-3,-MATVIEW_Center[1]+i,-MATVIEW_Center[2]);
      glVertex3d(-MATVIEW_Center[0]+3,-MATVIEW_Center[1]+i,-MATVIEW_Center[2]);
      
    }

  
  glEnd();

#if 0
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_POLYGON_SMOOTH);

  glColor4f(1.0,0.0,0.0,m_alpha);
  glCallList(10123);

  glColor4f(0.25,0.55,0.85,m_alpha);
  glCallList(10124);

  glColor4f(0.0,0.0,0.8,m_alpha);
  glCallList(10125);
  
  glDisable(GL_BLEND);
#endif
  //  glPopMatrix();
  glFlush();

}


void HRP2DisplayGL::initLighting(void)
{
  // light from a light source
  const GLfloat diffuseLight[] = {0.4, 0.4, 0.4, 1.0};
  // light from no particulat light source
  const GLfloat ambientLight[] = {0.1, 0.1, 0.1, 1.0};
  // light positions for 4 lights
  const GLfloat lightPositions[4][4] = {{ 1.0,  1.0,  0.0, 0.0},
					{-1.0, -1.0,  0.0, 0.0},
					{-0.1, -0.1,  1.0, 0.0},
					{ 0.1,  0.1, -1.0, 0.0}};

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);

  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT0, GL_POSITION, lightPositions[0]);
  glEnable(GL_LIGHT0);

  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT1, GL_POSITION, lightPositions[1]);
  glEnable(GL_LIGHT1);

  glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT2, GL_POSITION, lightPositions[2]);
  glEnable(GL_LIGHT2);

  glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT3, GL_POSITION, lightPositions[3]);
  glEnable(GL_LIGHT3);

  /*
  if (settings.f_lighting)
    glEnable(GL_LIGHTING);
  else
  glDisable(GL_LIGHTING);
  */
}


void HRP2DisplayGL::SlideTranslateXChanged(float X)
{
  m_TranslateX = X;
  updateGL();
}

void HRP2DisplayGL::SlideTranslateYChanged(float Y)
{
  m_TranslateY = Y;
  updateGL();
}

void HRP2DisplayGL::SlideTranslateZChanged(float Z)
{
  m_TranslateZ = Z;
  updateGL();
}

void HRP2DisplayGL::SlideRotXChanged(float RotX)
{
  m_fRotX = RotX;
  updateGL();
}

void HRP2DisplayGL::SlideRotYChanged(float RotY)
{
  m_fRotY = RotY;
  updateGL();
}

void HRP2DisplayGL::SlideRotZChanged(float RotZ)
{
  m_fRotZ = RotZ;
  updateGL();
}

void HRP2DisplayGL::setAlpha(float an_alpha)
{
  m_alpha = an_alpha; 
  updateGL();
}

void HRP2DisplayGL::BuildPlane()
{
  GLfloat MATVIEW_Center[3] = {0.0,0.0,0.0};
  glNewList(10123,GL_COMPILE);

  MATVIEW_Center[2]=-1.47;
  float di = 0.250;
  for(float i=-12*di;i<12*di;i+=di)
    {
      for(float j=-12*di;j<12*di;j+=di)
	{
	  glBegin(GL_QUADS);
	  glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]+j,-MATVIEW_Center[2]);
	  glVertex3d(-MATVIEW_Center[0]+i+di,-MATVIEW_Center[1]+j, -MATVIEW_Center[2]);
	  glVertex3d(-MATVIEW_Center[0]+i+di,-MATVIEW_Center[1]+j+di,-MATVIEW_Center[2]);
	  glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]+j+di,-MATVIEW_Center[2]);
	  glEnd();
	}
    }
  glEndList();
  
  glNewList(10124,GL_COMPILE);
  GLUquadricObj * sphere = gluNewQuadric();

  // Camera parameters sphere.
  for(float i=-12*di;i<12*di;i+=di)
    {
      for(float j=-12*di;j<12*di;j+=di)
	{
	  
	  if (sphere)
	    {
	      glPushMatrix();
	      glTranslatef(i+di/2,j+di/2,1.47);
	      gluSphere(sphere, 0.1, 20, 20); 
	      glPopMatrix();
	    }
	}
    }
  gluDeleteQuadric(sphere);
  glEndList();

  glNewList(10125,GL_COMPILE);
  glBegin(GL_LINES);
  /* Display scale. */
  for(float i=-12*0.250;i<=12*0.250;i+=0.250)
    {
      glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]-3,-MATVIEW_Center[2]);
      glVertex3d(-MATVIEW_Center[0]+i,-MATVIEW_Center[1]+3, -MATVIEW_Center[2]);
    }
  
  for(float i=-12*0.250;i<=12*0.250;i+=0.250)
    {
      glVertex3d(-MATVIEW_Center[0]-3,-MATVIEW_Center[1]+i,-MATVIEW_Center[2]);
      glVertex3d(-MATVIEW_Center[0]+3,-MATVIEW_Center[1]+i,-MATVIEW_Center[2]);
      
    }
  
  glEnd();
  glEndList();
}

void HRP2DisplayGL::HRP2_Rendering(double alpha)
{
  glEnable(GL_COLOR_MATERIAL);
  glMaterialf(GL_FRONT, GL_SHININESS, 100);

  glPushMatrix();

  glMultMatrixd(m_WaistPosture);
  // Dessin du Body - 100
  //if (Draw_HRP2Body)
    {
      // Dessin de Bodyb - gris anthracite
      glColor4f(0.2f, 0.2f, 0.2f, alpha);
      glCallList(101);
	  
      // Dessin de Bodyc - bleu
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(102);
	  
      // Dessin de Bodya - gris clair
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(100);
    }
      
  // Dessin de la jambe droite - 200
  //if (Draw_HRP2RLeg)
    {
      glPushMatrix();	// Push RLeg
	  
      // Dessin de RLEG_LINK0 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0, -0.060, 0);
      glRotated(m_RightLeg[0], 0, 0, 1);
      glCallList(200);
	  
      // Dessin de M_RLEG_LINK1 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_RightLeg[1], 1, 0, 0);
      glCallList(201);
	  
      // Dessin de M_RLEG_LINK2 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_RightLeg[2],0,1,0);
      glCallList(202);
	  
      // Dessin de M_RLEG_LINK3 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0,0,-0.300);
      glRotated(m_RightLeg[3],0,1,0);
      glCallList(203);

      // Dessin de M_RLEG_LINK4 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0,-0.035,-0.300);
      glRotated(m_RightLeg[4],0,1,0);
      glCallList(204);
	  
      glRotated(m_RightLeg[5],1,0,0);
	  
      // Dessin de M_RIGHTLEG_LINK5a - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(2051);
	  
      // Dessin de M_RLEG_LINK5b - gris fonce
      glColor4f(0.1f, 0.1f, 0.1f, alpha);
      glCallList(2052);
	  
      glPopMatrix();	// Pop RLeg
    }
      
  // Dessin de la jambe gauche - 300
  //if (Draw_HRP2LLeg)
    {
      glPushMatrix();	// Push LLeg
	  
      // Dessin de LLEG_LINK0 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0,0.060,0);
      glRotated(m_lleg[0],0,0,1);
	  
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(300);
	  
      // Dessin de M_LLEG_LINK1 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_lleg[1],1,0,0);
      glCallList(301);
	  
      // Dessin de M_LLEG_LINK2 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_lleg[2],0,1,0);
      glCallList(302);
	  
      // Dessin de M_LLEG_LINK3 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0,0,-0.300);
      glRotated(m_lleg[3],0,1,0);
      glCallList(303);
	  
      // Dessin de M_LLEG_LINK4 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0,0.035,-0.300);
      glRotated(m_lleg[4],0,1,0);
      glCallList(304);
	  
      glRotated(m_lleg[5],1,0,0);
	  
      // Dessin de M_LLEG_LINK5a - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(3051);
	  
      // Dessin de M_LLEG_LINK5b - gris fonce
      glColor4f(0.1f, 0.1f, 0.1f, alpha);
      glCallList(3052);
	  
      glPopMatrix();	// Pop M_Lleg
    }
      
  // Dessin du torse - 400
      
  glTranslated(0.032, 0, 0.3507);
  glRotated(m_chest[0], 0, 0, 1);

  //if (Draw_HRP2Chest)
    {
      // Dessin de m_chest_LINK0
      glColor4f(0.2f, 0.2f, 0.2f, alpha);
      glCallList(400);
    }
      
  glRotated(m_chest[1], 0, 1, 0);
      
  //if (Draw_HRP2Chest)
    {
      // Dessin de m_chest_LINK1a - gris metallise
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(4011);
	  
      // Dessin de m_chest_LINK1b - bleu
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(4012);
	  
      // Dessin de m_chest_LINK1c - jaune
      glColor4f(0.9f, 0.9f, 0.0f, alpha);
      glCallList(4013);
	  
      // Dessin de m_chest_LINK1e - noir
      glColor4f(0.05f, 0.05f, 0.05f, alpha);
      glCallList(4015);
	  
      // Dessin de CHEST_LINK1d - gris anthracite
      glColor4f(0.2f, 0.2f, 0.2f, alpha);
      glCallList(4014);
    }
      
  // Dessin de la tete - 500
  //if (Draw_HRP2Head)
    {
      glPushMatrix();	// Push Head
	  
      glTranslated(-0.007, 0.0, 0.2973);
      glRotated(m_head[0], 0.0, 0.0, 1.0);
      glCallList(500);
	  
      // Dessin de HEADJ_LINK1a - aluminium
      glRotated(m_head[1], 0.0, 1.0, 0.0);
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(5011);
	  
      // Dessin de HEAD_LINK1b - bleu
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(5012);
	  
      // Dessin de HEAD_LINK1c - noir
      glColor4f(0.05f, 0.05f, 0.05f, alpha);
      glCallList(5013);
	  
      // Dessin de HEAD_LINK1d - gris clair
      glColor4f(0.2f, 0.2f, 0.2f, alpha);
      glCallList(5014);
	  
      // Dessin de HEAD_LINK1e - jaune
      glColor4f(0.9f, 0.9f, 0.0f, alpha);
      glCallList(5015);
	  
	  

      glPopMatrix();	// Pop Head
    }

  // Dessin du bras droit - 600
  //if (Draw_HRP2RHand)
    {
      glPushMatrix();	// Push RHand

      // Dessin de RARM_LINK0 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0.008, -0.250, 0.181);
      glRotated(m_rarm[0], 0.0, 1.0, 0.0);

      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(600);

      // Dessin de RARM_LINK1 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_rarm[1], 1.0, 0.0, 0.0);
      glCallList(601);

      // Dessin de RARM_LINK2a - bleu
      glRotated(m_rarm[2],0.0,0.0,1.0);
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(6021);

      // Dessin de RARM_LINK2b - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(6022);

      // Dessin de RARM_LINK3a - bleu
      glTranslated(0.0, 0.0, -0.250);
      glRotated(m_rarm[3], 0.0, 1.0, 0.0);
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(6031);

      // Dessin de RARM_LINK3b - jaune
      glColor4f(0.9f, 0.9f, 0.0f, alpha);
      glCallList(6032);

      // Dessin de RARM_LINK4 - aluminium
      glTranslated(0.0, 0.0, -0.250);
      glRotated(m_rarm[4],0.0,0.0,1.0);
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(604);

      glRotated(m_rarm[5], 0.0, 1.0, 0.0);

      switch(m_RightGripperType)
	{
	case 0:
	  glPushMatrix();  // Push 1
			
	  // Dessin de LARM_LINK5a - bleu
	  glColor4f(0.0f, 0.0f, 0.6f, alpha);
	  glCallList(6041);

	  // Dessin de LARM_LINK5b - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(6042);

	  glPopMatrix();	// Pop 1

	  // Dessin de LARM_LINK6a - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0.0219, 0.0, -0.095);
	  glRotated(m_rarm[6], 0.0, 1.0, 0.0);
	  glCallList(6043);

	  // Dessin de LARM_LINK6b - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(6044);
	  break;
	case 1:
	  // Dessin de RARM_LINK5 - bleu
	  glColor4f(0.0f, 0.0f, 0.6f, alpha);
	  glCallList(6051);
	  glCallList(6052);
	
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(6053);
			
	  glPushMatrix();	// Push 1			
	  // Dessin de RARM_LINK6 - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0.0, 0.020, -0.095);
	  glRotated(m_rarm[6], 1, 0, 0);
	  glCallList(60601);
			
	  // Dessin de RHAND_LINK0 - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0, 0, -0.06225397);
	  glRotated(-m_rarm[6], 1, 0, 0);

	  glColor4f(1.0f, 1.0f, 1.0f, alpha);
	  glCallList(6061);
		
	  // Dessin de RHAND_LINK1 - aluminium
	  glTranslated(0, -0.016, -0.014);
	  glRotated(m_rarm[6], 1, 0, 0);
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(6062);
	  glPopMatrix();	// Pop 1
			
	  // Dessin de RHAND_LINK2 - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0.0, -0.020, -0.095);
	  glRotated(-m_rarm[6], 1, 0, 0);
	  glCallList(60701);
			
	  // Dessin de RHAND_LINK3 - aluminium
	  glTranslated(0.0, 0.0, -0.062225397);
	  glRotated(m_rarm[6], 1, 0, 0);
	  glColor4f(1.0f, 1.0f, 1.0f, alpha);
	  glCallList(6063);

	  // Dessin de RHAND_LINK4 - aluminium
	  glTranslated(0.0, 0.016, -0.014);
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glRotated(-m_rarm[6], 1, 0, 0);
	  glCallList(6064);
	}

      glPopMatrix();	// Pop RHand		
    }

  // Dessin du bras gauche - 700
  //if (Draw_HRP2LHand)
    {
      glPushMatrix(); // Push LHand

      // Dessin de M_LARM_LINK0 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glTranslated(0.008, 0.250, 0.181);    
      glRotated(m_larm[0], 0.0, 1.0, 0.0);
        
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(700);

      // Dessin de M_LARM_LINK1 - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glRotated(m_larm[1], 1.0, 0.0, 0.0);
      glCallList(701);

      // Dessin de M_LARM_LINK2a - bleu
      glRotated(m_larm[2],0.0,0.0,1.0);
		
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(7021);

      // Dessin de M_LARM_LINK2b - aluminium
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(7022);

      // Dessin de M_LARM_LINK3a - bleu
      glTranslated(0.0, 0.0, -0.250);
      glRotated(m_larm[3], 0.0, 1.0, 0.0);
		
      glColor4f(0.0f, 0.0f, 0.6f, alpha);
      glCallList(7031);

      // Dessin de M_LARM_LINK3b - jaune
		
      glColor4f(0.9f, 0.9f, 0.0f, alpha);
      glCallList(7032);

      // Dessin de M_LARM_LINK4 - aluminium
      glTranslated(0.0, 0.0, -0.250);
      glRotated(m_larm[4], 0.0, 0.0, 1.0);
        
      glColor4f(0.7f, 0.7f, 0.7f, alpha);
      glCallList(704);

      glRotated(m_larm[5], 0.0, 1.0, 0.0);

      switch(m_LeftGripperType)
	{
	case 0:
	  glPushMatrix();  // Push 1
			
	  // Dessin de M_LARM_LINK5a - bleu
	  glColor4f(0.0f, 0.0f, 0.6f, alpha);
	  glCallList(7051);

	  // Dessin de M_LARM_LINK5b - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(7052);

	  glPopMatrix();	// Pop 1

	  // Dessin de M_LARM_LINK6a - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0.0219, 0.0, -0.095);
	  glRotated(m_larm[6], 0.0, 1.0, 0.0);
	  glCallList(7061);

	  // Dessin de M_LARM_LINK6b - aluminium
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(7062);
	  break;
	case 1:			
	  // Dessin de M_LARM_LINK5a - bleu
	  glColor4f(0.0f, 0.0f, 0.6f, alpha);
	  glCallList(7151);
	  glCallList(7152);

	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(7153);
			
	  glPushMatrix();
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glTranslated(0.0, 0.020, -0.095);
	  glRotated(m_larm[6], 1.0, 0.0, 0.0);
	  glCallList(60602);

	  glTranslated(0.0, 0.0, -0.06225397);
	  glRotated(-m_larm[6], 1, 0, 0);
	  glColor4f(1.0f, 1.0f, 1.0f, alpha);
	  glCallList(7154);

	  glTranslated(0.0, -0.016, -0.014);
	  glRotated(m_larm[6], 1.0, 0.0, 0.0);
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(7155);
	  glPopMatrix();

	  glTranslated(0.0, -0.020, -0.095);
	  glRotated(-m_larm[6], 1.0, 0.0, 0.0);
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(60702);

	  glTranslated(0.0, 0.0, -0.06225397);
	  glRotated(m_larm[6], 1, 0, 0);
	  glColor4f(1.0f, 1.0f, 1.0f, alpha);
	  glCallList(7156);

	  glTranslated(0.0, 0.016, -0.014);
	  glRotated(-m_larm[6], 1.0, 0.0, 0.0);
	  glColor4f(0.7f, 0.7f, 0.7f, alpha);
	  glCallList(7157);
	}
      glPopMatrix(); // Pop LHand
    }
	
  glPopMatrix();	// Pop HRP2
  glDisable(GL_COLOR_MATERIAL);
}
