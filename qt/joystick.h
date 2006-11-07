#ifndef JOYSTICK_H
#define JOYSTICK_H 1

#include <linux/joystick.h>
#include <deque>

class Joystick
{
 public:
  
  Joystick();
  ~Joystick();

  int create();
  void finish();
  int status(double &X, double &Y, double &Theta);
  int status2(double &X, double &Y, double &Theta);
  void print();

 protected:

  // File descriptor on the device.
  int m_fd;
  
  // Structure for data related to the joystick.
  struct JS_DATA_TYPE m_js;
  
  // Name of the joystick
  char *m_name;

  // Stack of values.
  std::deque<double> m_X;
  std::deque<double> m_Y;
  std::deque<double> m_Theta;  
  double m_Mean[3];
};

#endif

