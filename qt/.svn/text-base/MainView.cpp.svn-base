//
//    File: mview.cc
//
//    (C) 2000 Helmut Cantzler
//
//    Licensed under the terms of the Lesser General Public License.
//
#include <iostream>
#include <fstream>
#include <qspinbox.h> 
#include <qslider.h>
#include <qtoolbar.h>
#include <qtoolbutton.h>
#include <qpopupmenu.h>
#include <qmenubar.h>
#include <qkeycode.h>
#include <qfile.h>
#include <qfiledialog.h>
#include <qprogressdialog.h>
#include <qstatusbar.h>
#include <qmessagebox.h>
#include <qapplication.h>
#include <qaccel.h>
#include <qpixmap.h>
#include <qimage.h>
#include <qhbox.h>
#include <qtooltip.h>
#include <qcolordialog.h>
#include <qdesktopwidget.h>

#include <GL/glut.h>
#include "MainView.h"
#include <VNL/matrix.h>

//#include "viewpointdialog.h"
//#include "coordinates.h"

#include <getopt.h>


#include "HRP2Display.h"
#include "MotorValuesEdit.h"


QProgressDialog *pd;
QLabel *label_mesh, *label_features, *label_fps;

int update_progress(int pos)
{
  static int q=0;

  if (q % 100 == 0)
    pd->setProgress(pos);
  q++;

  return pd->wasCancelled();
}

void set_total(int size)
{
  pd->setTotalSteps(size);
}

MainView::MainView(int argc, char **argv)
    : QMainWindow( 0, "Main Window for PG")
{
  //// Set OpenGL options
  glutInit(&argc,argv);


  m_Workspace = new QWorkspace(this, "Workspace");

  //// Create the main and the mesh widget

  /*  QHBox *q = new QHBox(m_Workspace);
  q->setMargin(10);
  q->setSpacing(10);
  */

  setCentralWidget(m_Workspace);
  
  resize(1024,840);
  HRP2Display *m_HD = new HRP2Display(m_Workspace,"HRP2");

  QPopupMenu *file = new QPopupMenu(this);
  menuBar()->insertItem("&File",file);
  file->insertItem("&Open File", this, SLOT(load_file()), Key_O);
  file->insertItem("&Quit", qApp, SLOT( quit() ), Key_Escape);

  // Pattern Generator Menu  
  QPopupMenu *PGModes = new QPopupMenu(this);
  menuBar()->insertItem("&PG",PGModes);
  PGModes->insertItem("To ZMP stack",this, SLOT(ToZMPStack()), Key_Z);
  PGModes->insertItem("Initial Position",this, SLOT(InitialPosition()));

  PGModes->insertItem("Start On Line",this, SLOT(StartOnLine()), Key_S);
  PGModes->insertItem("Stop On Line",this, SLOT(StopOnLine()), Key_T);

  std::istringstream strm("../src/PreviewControlParameters.ini ../../../../../OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl");
  m_PGI =  new PatternGeneratorJRL::PatternGeneratorInterface(strm);

  // Status bar.
  // Display speed of display.
  label_fps = new QLabel(" FPS: 0.0 ", statusBar(), "FPS Label");
  statusBar()->addWidget(label_fps, 0, TRUE);
  
  m_timer = new QTimer(this,"Timer for Main Window" );
  connect(m_timer,SIGNAL(timeout()), 
	  this,SLOT(timerEvent())); 
  m_HD->show();

  connect(this,SIGNAL(sendMotors(double *)),
	  m_HD->m_HRP2GL, SLOT(SetMotors(double *)));

  connect(this,SIGNAL(sendWaistPosture(double *)),
	  m_HD->m_HRP2GL, SLOT(SetWaistPosture(double *)));

  connect(this,SIGNAL(sendMotorsAndWaistPosture(double *,double *)),
	  m_HD->m_HRP2GL, SLOT(SetMotorsAndWaistPosture(double *,double *)));
  
  InitialPosition("HalfSitting.init");

  // Connect to the joystick.
  m_Joystick = new Joystick();
}

MainView::~MainView()
{
}

void MainView::clear_views()
{
}

void MainView::save_view()
{
}

void MainView::set_view()
{
}

void MainView::reset_view()
{
}

void MainView::generate_view()
{

}

void MainView::LoadCameraParameters()
{
  
}

void MainView::load_view(int id)
{

}

void MainView::GenerateImageFromViews()
{
  
}

void MainView::setTexture()
{

}

void MainView::setSolid()
{

}

void MainView::setFrontlines()
{
}

void MainView::setWire()
{

}

void MainView::setPoints()
{

}

void MainView::setFeatures()
{
}

void MainView::togglePick()
{
}

void MainView::toggleTextureMapping()
{
}

void MainView::toggleLighting()
{
}

void MainView::toggleShapeColors()
{
}

void MainView::toggleSurfaceNormals()
{

}

void MainView::toggleCutBackFaces()
{
}

void MainView::toggleBilinearFiltering()
{

}

void MainView::toggleFeatures()
{
}

void MainView::toggleAspectRatio()
{
}

void MainView::setBackgroundColor()
{

}

void MainView::negateNormals()
{

}

void MainView::removeDoublePoints()
{
}

void MainView::ComputeAPostureFromEulerAngles(PatternGeneratorJRL::COMPosition &aPosition,
					      double *Posture)
{
  // Compute Waist Posture.
  double c,s,co,so;
  c = cos(aPosition.theta*M_PI/180.0);
  s = sin(aPosition.theta*M_PI/180.0);
  
  co = cos(aPosition.omega*M_PI/180.0);
  so = sin(aPosition.omega*M_PI/180.0);
  
  // COM Orientation
  Posture[0] = c*co;Posture[1]  = -s; Posture[2] = c*so;Posture[3]  = aPosition.x[0];
  Posture[4] = s*co;Posture[5]  =  c; Posture[6] = s*so;Posture[7]  = aPosition.y[0];
  Posture[8] =  -so;Posture[9]  =  0; Posture[10]= co  ;Posture[11] = aPosition.z[0];
  Posture[12]=  0.0;Posture[13] =0.0; Posture[14]= 0.0 ;Posture[15] = 1.0;

}

void MainView::timerEvent()
{
  QString s;
  label_fps->setText(s);

  if (m_PGI!=0)
    {
      VNL::Matrix<double> qr(6,1,0.0),ql(6,1,0.0),
	UpperBodyAngles(28,1,0.0),
	ZMPTarget(3,1,0.0);
      PatternGeneratorJRL::COMPosition CurrentWaistPosition;

      if(m_PGI->RunOneStepOfTheControlLoop(qr,ql,UpperBodyAngles,ZMPTarget,CurrentWaistPosition))
	{
	  double lMotors[30];

	  int GlobalIndex = 0;

	  // Right Leg
	  for(int i=0;i<6;i++)
	    lMotors[GlobalIndex++] = qr(i,0);
	  
	  // Left Leg 
	  for(int i=0;i<6;i++)
	    lMotors[GlobalIndex++] = ql(i,0);
	  
	  // UpperBody
	  for(int i=0;i<18;i++)
	    lMotors[GlobalIndex++] = UpperBodyAngles(i,0);
	  
	  double WaistPosture[16];
	  //ComputeAPostureFromEulerAngles(CurrentWaistPosition,WaistPosture);
	  VNL::Matrix<double> lWaistAbsPos;
	  m_PGI->getWaistPositionMatrix(lWaistAbsPos);

	  for(int i=0;i<4;i++)
	    for(int j=0;j<4;j++)
	      WaistPosture[i*4+j] = lWaistAbsPos(i,j);
	  
	  cout << "Position: " << WaistPosture[3] << " " << WaistPosture[7] << " " << WaistPosture[11] << endl;
	  emit sendMotorsAndWaistPosture(lMotors,WaistPosture);
	}
      else
	{

	  // Stop the timer.
	  m_timer->stop();
	}

    }

  if (m_Joystick!=0)
    {
      double X=0.0,Y=0.0,Theta=0.0;
      m_Joystick->status2(X,Y,Theta);
      m_PGI->AddOnLineStep(X,Y,Theta);
    }

}

void MainView::takeShot(int item)
{
}

void MainView::save_as(int item)
{
}

void MainView::load_file()
{
  QString s = QFileDialog::getOpenFileName(".","*.txt",this, "Open File Dialog","Choose a file");
  ifstream aif;
  aif.open(s,ifstream::in);
  if (aif.is_open())
    {
      while(!aif.eof())
	{
	  char Buffer[2048];
	  aif.getline(Buffer,2048);
	  istringstream strm(Buffer);
	  if (m_PGI!=0)
	    {
	      m_PGI->ParseCmd(strm);
	    }
	}

      m_timer->start(5,FALSE);
      
    } 
}

void MainView::InitialPosition()
{
  QString s = QFileDialog::getOpenFileName(".","*.init",this, "Open File Dialog","Choose a file");
  InitialPosition(s);
}

void MainView::InitialPosition(QString s )
{
  ifstream aif;
  aif.open(s,ifstream::in);
  if (aif.is_open())
    {
      double lMotors[30];
      unsigned lIndex=0;
      string ls;

      // Load the values from the file.
      while(!aif.eof())
	{
	  if (lIndex<30)
	    aif >> lMotors[lIndex];
	  else
	    aif >> ls;
	  lIndex++;
	}

      // Conversion in radians.
      double degtorad = M_PI/180.0;
      VNL::Matrix<double> InitialPosition(40,1,0.0),CurrentPosition(40,1,0.0);
  
      for(int i=0;i<30;i++)
	{
	  lMotors[i] *=degtorad;
	  
	  InitialPosition[i][0] = lMotors[i];

	  //std::cout << i << " : " << lMotors[i] << std::endl;
	}
      m_PGI->SetCurrentJointValues(InitialPosition);

      // Find the associate Waist Position.
      VNL::Vector<double> lStartingCOMPosition(3);
      VNL::Vector<double> lStartingWaistPosition(3);
      PatternGeneratorJRL::FootAbsolutePosition InitLeftAbsPos,
	InitRightAbsPos;

      m_PGI->EvaluateStartingCOM(InitialPosition,
				 lStartingCOMPosition,
				 lStartingWaistPosition,
				 InitLeftAbsPos,
				 InitRightAbsPos);

      double WaistPosture[16]={1.0, 0.0, 0.0, 0.0,
			       0.0, 1.0, 0.0, 0.0,
			       0.0, 0.0, 1.0, 0.0,
			       0.0, 0.0, 0.0, 1.0};

      WaistPosture[3] = lStartingWaistPosition(0);
      WaistPosture[7] = lStartingWaistPosition(1);
      WaistPosture[11] = lStartingWaistPosition(2);

      emit sendMotorsAndWaistPosture(lMotors,WaistPosture);
    } 
}

void MainView::ToZMPStack()
{
}


void MainView::StopOnLine()
{
  if (m_PGI!=0)
    {
      m_PGI->StopOnLineStepSequencing();
    }
}

void MainView::StartOnLine()
{
  if (m_PGI!=0)
    {
      m_PGI->StartOnLineStepSequencing();
      m_timer->start(5,FALSE);
    }
}

void MainView::set_status()
{ 
}

void MainView::SimulateProjection()
{
}

void MainView::HRP2Model()
{

  HRP2Display * aHRP2 = new HRP2Display(m_Workspace,"HRP2 Model");
  aHRP2->show();
}

void MainView::about()
{
  QMessageBox::about(this, "Mesh Viewer 0.2.2",
   "\nCopyright (c) 2001-2003 Helmut Cantzler\n\n"
   "All files in this package can be freely distributed and\n"
   "used according to the terms of the Lesser General Public License.\n\n"
   "For more information visit the MeshViewer website at:\n"
   "http://mview.sourceforge.net/\n");
}

void MainView::aboutQt()
{
  QMessageBox::aboutQt(this, "Mesh Viewer");
}

void MainView::MotorValues()
{
  MotorValuesEdit * m_MVE;
  m_MVE = new MotorValuesEdit(m_Workspace, "Motor Values Editor");
  m_MVE->show();
}

