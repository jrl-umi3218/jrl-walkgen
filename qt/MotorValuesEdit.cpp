#include <iostream>


using namespace std;

#include "MotorValuesEdit.h"
using namespace HRP2DisplayNS;
#include <qpushbutton.h>
#include <qvbox.h>
#include <qhbox.h>
#include <qlabel.h>
#include <qscrollview.h>


MotorValuesEdit::MotorValuesEdit(QWidget *parent,
				 const char *name,
				 HRP2DisplayNS::MultiBody *aMB)
  :QWidget(parent,name)
{
  m_Table = 0;
  // Create the appropriate number of SpinBox.
  m_VBox = new QVBox(this);
  SetMultiBody(aMB);

  QHBox * aHBox =  new QHBox(m_VBox);

  QPushButton * OkButton = new QPushButton("Ok",aHBox);
  connect(OkButton,SIGNAL(clicked()), this, SLOT(close()));
  
  aHBox->move(width()/2 - OkButton->width()/2, height()- OkButton->height()-10);
  resize(200,400);
}


MotorValuesEdit::~MotorValuesEdit()
{
}

void MotorValuesEdit::SetMultiBody(HRP2DisplayNS::MultiBody *aMB)
{
  vector<float*> lienSpinner;			//pointeur sur les variables de transformations

  m_MB = aMB;

  cout << "MotorValueEdit::SetMultiBody: "<< m_MB << endl;
  if (m_MB==0)
    return;
  
  cout << "MotorValueEdit::SetMultiBody: "<< m_MB->liaisons.size() << endl;

  m_Table = new QTable(m_MB->liaisons.size()-4,2,m_VBox);

  for(int i=0,k=0;i<m_MB->liaisons.size();i++)
    {
      string CorpsName ;
      QString Value;

      if ((CorpsName!="HRP2") &&
	  (CorpsName!="WAIST"))
	{
	  cout <<lienSpinner[k+4]<< endl;
	  m_Table->setText(k,0, CorpsName);
	  Value = QString("%1").arg(*(lienSpinner[k+4]));
	  m_Table->setText(k,1, Value);
	  //	  cout << m_MB->listeCorps[i].getName() << " "<<endl;
	  k++;
	  //m_MB->listeCorps[i].afficherNombreObjets();
	}
    }
 
}

void MotorValuesEdit::SynchronizeValue()
{
  
}

void MotorValuesEdit::resizeEvent(QResizeEvent *aQR)
{
  QSize asize = aQR->size();

  if ((asize.width()<200) || (asize.height()<400))
    QWidget::resize(200,400);

  m_VBox->resize(asize.width()-10,
		 asize.height()-10);
  if (m_Table!=0)
    m_Table->resize(asize.width()-10,
		    asize.height()-50);
}


