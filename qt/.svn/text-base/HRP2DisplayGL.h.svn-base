//
// File : HRP2DisplayGL.h
// (c) 2005 Olivier Stasse JRL, CNRS/AIST
// 
#ifndef _HRP2_DISPLAY_GL_H_
#define _HRP2_DISPLAY_GL_H_

#include <qgl.h>
#include <Body.h>
#include <vector>


class HRP2DisplayGL : public QGLWidget
{
  Q_OBJECT
 public:
  HRP2DisplayGL(
		QWidget *parent, 
		const char *name);
  ~HRP2DisplayGL();
  
  void SlideTranslateXChanged(float X);
  void SlideTranslateYChanged(float Y);
  void SlideTranslateZChanged(float Z);
  void SlideRotXChanged(float RotX);
  void SlideRotYChanged(float RotY);
  void SlideRotZChanged(float RotZ);
  void setAlpha(float anAlpha);
  void BuildPlane();
  void LoadOpenGLFiles();
  void HRP2_Rendering(double alpha);

  // Legs
  double m_RightLeg[6];
  double m_lleg[6];

  // Arms
  double m_rarm[7];
  double m_larm[7];

  // Chest
  double m_chest[2];
  double m_head[2];

  public slots:
  void SetMotors(double *Motors);
  void SetWaistPosture(double * aPosture);  
  void SetMotorsAndWaistPosture(double * Motors,double * WaistPosture);

 protected:

  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  void initLighting(void);

 protected:
    // Angles for the robot.


  Body  * m_BODYa,
    * m_BODYb,
    * m_BODYc,

    * m_RLEGLINK0,
    * m_RLEGLINK1,
    * m_RLEGLINK2,
    * m_RLEGLINK3,
    * m_RLEGLINK4,
    * m_RLEGLINK5a,
    * m_RLEGLINK5b,
    

    * m_LLEGLINK0,
    * m_LLEGLINK1,
    * m_LLEGLINK2,
    * m_LLEGLINK3,
    * m_LLEGLINK4,
    * m_LLEGLINK5a,
    * m_LLEGLINK5b,

    * m_CHESTLINK0,
    * m_CHESTLINK1a,
    * m_CHESTLINK1b,
    * m_CHESTLINK1c,
    * m_CHESTLINK1d,
    * m_CHESTLINK1e,
    
    * m_HEADLINK0,
    * m_HEADLINK1a,
    * m_HEADLINK1b,
    * m_HEADLINK1c,
    * m_HEADLINK1d,
    * m_HEADLINK1e,

    * m_RARMLINK0,
    * m_RARMLINK1,
    * m_RARMLINK2a,
    * m_RARMLINK2b,
    * m_RARMLINK3a,
    * m_RARMLINK3b,
    * m_RARMLINK4,

    * m_RH0,
    * m_RH1,
    * m_RH2,
    * m_RH3,

    * m_RG0,
    * m_RG1,
    * m_RG2,
    * m_RG3,
    * m_RG4,
    * m_RG5,
    * m_RG6,
    * m_RG7,
    * m_RG8,

    * m_LARMLINK0,
    * m_LARMLINK1,
    * m_LARMLINK2a,
    * m_LARMLINK2b,
    * m_LARMLINK3a,
    * m_LARMLINK3b,
    * m_LARMLINK4,

    * m_LH0,
    * m_LH1,
    * m_LH2,
    * m_LH3,

    * m_LG0,
    * m_LG1,
    * m_LG2,
    * m_LG3,
    * m_LG4,
    * m_LG5,
    * m_LG6,
    * m_LG7,
    * m_LG8;


  int m_corpsDeDepart;
  float m_alpha;
  float m_TranslateX;
  float m_TranslateY;
  float m_TranslateZ;
  float m_fRotX;
  float m_fRotY;
  float m_fRotZ;


  int m_RightGripperType;
  int m_LeftGripperType;

  double m_WaistPosture[16]; // This matrix follows the OpenGL format
};
#endif /* _HRP2_DISPLAY_GL_H_ */
