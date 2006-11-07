// 
// File : HRP2Display.h
// (c) 2005 Olivier Stasse JRL, CNRS/AIST
// 

#ifndef _HRP2DISPLAY_H_
#define _HRP2DISPLAY_H_

#include <qwidget.h>
#include <Body.h>
#include "HRP2DisplayGL.h"

class HRP2Display: public QWidget
{
  Q_OBJECT

public: 

  HRP2Display(QWidget *parent, const char *name);
  ~HRP2Display();

  void paintEvent(QPaintEvent *event);

  // Reference to the OpenGL window.
  HRP2DisplayGL *m_HRP2GL;

 protected:


 
  // Sliders


  public slots:
  
  void SlideTranslateXChanged(int X);
  void SlideTranslateYChanged(int Y);
  void SlideTranslateZChanged(int Z);
  void SlideRotXChanged(int X);
  void SlideRotYChanged(int Y);
  void SlideRotZChanged(int Y);
  void SetAlpha(int an_alpha);

};
#endif
