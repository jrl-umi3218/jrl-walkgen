//
// File: MotorValuesEdit.h
// 
// Edit the motor values.
//
// (c) 2005 Olivier Stasse, JRL CNRS/AIST

#ifndef _MOTOR_VALUES_EDIT_H_
#define _MOTOR_VALUES_EDIT_H_

#include "MultiBody.h"
#include <qwidget.h>
#include <qtable.h>
#include <qvbox.h>


class MotorValuesEdit: public QWidget
{
  Q_OBJECT

   public:
  MotorValuesEdit(QWidget * parent, const char * name, MultiBody *aMB=0);

  ~MotorValuesEdit();


 public:
  //Specify the multibody  to use 
  void SetMultiBody(MultiBody *aMB);

  public slots:

  // Modify the fields value to reflect the MultiBody 
  // true values.
  void SynchronizeValue();

  signals:
  // The values of the MultiBody have been modified by this dialog 
  // widget.
  void ValuesModified();

 protected:

  void resizeEvent(QResizeEvent *aRE);

  // Reference to the robot's multibody.
  MultiBody *m_MB;

  // Table;
  QTable * m_Table;

  // Vertical box.
  QVBox * m_VBox;
};
#endif /* _MOTOR_VALUES_EDIT_H_ */
