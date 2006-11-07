#include <qhbox.h>
#include <qvbox.h>
#include <qslider.h>
#include "HRP2Display.h"

HRP2Display::HRP2Display(QWidget *parent, const char *name) : 
  QWidget(parent, name)
{
  QHBox *q = new QHBox(this);
  
  resize(640+250,480+20);
  q->resize(640+220,480+10);
  q->setSpacing(10);
  q->setMargin(10);

  m_HRP2GL = new HRP2DisplayGL(q,"HRP2 Model");
  m_HRP2GL->show();
  m_HRP2GL->updateGL();

  QSlider *as = new QSlider(Qt::Vertical,q,"TranslateX");
  as->setMinValue(-100);  as->setMaxValue(100);as->setSteps(5,10);
  as->setValue(0);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideTranslateXChanged(int)));

  as = new QSlider(Qt::Vertical,q,"TranslateY");
  as->setMinValue(-100);  as->setMaxValue(100);as->setSteps(5,10);
  as->setValue(0);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideTranslateYChanged(int)));

  as = new QSlider(Qt::Vertical,q,"TranslateZ");
  as->setMinValue(-100);  as->setMaxValue(100);as->setSteps(5,10);
  as->setValue(-41);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideTranslateZChanged(int)));


  as = new QSlider(Qt::Vertical,q,"RotateX");
  as->setMinValue(0);  as->setMaxValue(360);as->setSteps(5,10);
  as->setValue(180);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideRotXChanged(int)));

  as = new QSlider(Qt::Vertical,q,"RotateY");
  as->setMinValue(0);  as->setMaxValue(360);as->setSteps(5,10);
  as->setValue(180);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideRotYChanged(int)));

  as = new QSlider(Qt::Vertical,q,"RotateZ");
  as->setMinValue(0);  as->setMaxValue(360);as->setSteps(5,10);
  as->setValue(180);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SlideRotZChanged(int)));

  as = new QSlider(Qt::Vertical,q,"Alpha");
  as->setMinValue(0);  as->setMaxValue(100);as->setSteps(5,10);
  as->setValue(10);
  connect(as,SIGNAL(valueChanged(int)),this,SLOT(SetAlpha(int)));

}

HRP2Display::~HRP2Display()
{

}

void HRP2Display::paintEvent(QPaintEvent *event)
{
  m_HRP2GL->updateGL();
}

void HRP2Display::SetAlpha(int anAlpha)
{
  m_HRP2GL->setAlpha(anAlpha/100.0);
}

void HRP2Display::SlideTranslateXChanged(int X)
{
  m_HRP2GL->SlideTranslateXChanged((float)X/12.75);
}

void HRP2Display::SlideTranslateYChanged(int Y)
{
  m_HRP2GL->SlideTranslateYChanged((float)Y/12.75);
}

void HRP2Display::SlideTranslateZChanged(int Z)
{
  m_HRP2GL->SlideTranslateZChanged((float)Z/12.75);
}

void HRP2Display::SlideRotXChanged(int X)
{
  m_HRP2GL->SlideRotXChanged((float)X);
}

void HRP2Display::SlideRotYChanged(int Y)
{
  m_HRP2GL->SlideRotYChanged((float)Y);
}

void HRP2Display::SlideRotZChanged(int Z)
{
  m_HRP2GL->SlideRotZChanged((float)Z);
}

