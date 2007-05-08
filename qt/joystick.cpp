#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "joystick.h"

#define JOYSTICK_DEVFS "/dev/input/js0" /* devfs */
#define JOYSTICK "/dev/js0"


Joystick::Joystick()
{
  char c_num_axis;
  char c_num_buttons;
  unsigned long ver = 0;
  unsigned int namesize = 100;

  m_name = NULL;

  if (!(m_fd = open(JOYSTICK_DEVFS, O_RDONLY | O_NONBLOCK)))
    m_fd = open(JOYSTICK, O_RDONLY | O_NONBLOCK);

  if (m_fd) {
    if (!ioctl(m_fd,JSIOCGVERSION,&ver))
      printf("JSIOCGVERSION %d\n", ver);
    if (ioctl(m_fd,JSIOCGAXES,&c_num_axis))
      c_num_axis = 0;
    if (ioctl(m_fd,JSIOCGBUTTONS,&c_num_buttons))
      c_num_buttons = 0;
    m_name = new char[namesize];
    sprintf(m_name,"%dbt %dax ", c_num_buttons, c_num_axis);
    if (ioctl(m_fd,JSIOCGNAME(namesize-strlen(m_name)),m_name+strlen(m_name))>=0)
      printf("ioctl(JSIOCGNAME) %s\n", m_name);
  }

  m_Mean[0] = 0.0;
  m_Mean[1] = 0.0;
  m_Mean[2] = 0.0;
}

Joystick::~Joystick()
{
  if (m_name)
    free(m_name);
  close(m_fd);
}

int Joystick::status(double &X, double &Y, double &Theta)
{
  int JOYSTICK_MAX_POS = 32767;

  int status;
  struct js_event jse;
  
  double lx=0.0;
  double ly=0.0;
  double ltheta=0.0;

  
  if (m_X.size()>0)
    {
      lx = m_X.back();
      ly = m_Y.back();
      ltheta = m_Theta.back();
    }
  
  while(  (status = read(m_fd, &jse, sizeof(struct js_event)))>0)
    {
      //  if (status != JS_RETURN) 
      //    return 0;
      
      int axis_number=-1;
      int axis_value;
      switch (jse.type & ~JS_EVENT_INIT)
	{
	case JS_EVENT_AXIS:
	  axis_number = jse.number;
	  axis_value = jse.value;
	  break;
	};
      //      std::cout << "axis: " << axis_number << " " << axis_value << std::endl;
          
      switch(axis_number)
	{
	case 0:
	  ly = (double)axis_value/(double)JOYSTICK_MAX_POS;
	  break;
	case 1:
	  lx = (double)axis_value/(double)JOYSTICK_MAX_POS;
	  break;
	case 3:
	  ltheta = (double)axis_value/(double)JOYSTICK_MAX_POS;
	  break;
	  
	};
    };
  //  std::cout << "lx : " << lx << " ly : " << ly << " theta : " << ltheta << std::endl;

  m_X.push_back(lx);
  m_Y.push_back(ly);
  m_Theta.push_back(ltheta);
  m_Mean[0] += lx;
  m_Mean[1] += ly;
  m_Mean[2] += ltheta;

  if (m_X.size()==161)
    {
      m_Mean[0] -= m_X[0];
      m_Mean[1] -= m_Y[0];
      m_Mean[2] -= m_Theta[0];

      m_Y.pop_front();
      m_X.pop_front();
      m_Theta.pop_front();
    }
  double nbels = (double)m_X.size();
  X = -(m_Mean[0]/nbels) * 0.1;
  Y = -(m_Mean[1]/nbels) * 0.05;
  Theta = -(m_Mean[2]/nbels) * M_PI;
  return 1;
}

int Joystick::status2(double &X, double &Y, double &Theta)
{

  int status;
  struct JS_DATA_TYPE js;

  status = read(m_fd, &js, JS_RETURN);

  //  if (status != JS_RETURN) 
  //    return 0;

  unsigned char lx=js.y;
  unsigned char ly=js.x;
  unsigned char ltheta=128;

  m_X.push_back(lx);
  m_Y.push_back(ly);
  m_Theta.push_back(ltheta);
  m_Mean[0] += lx;
  m_Mean[1] += ly;
  m_Mean[2] += ltheta;

  if (m_X.size()==161)
    {
      m_Mean[0] -= m_X[0];
      m_Mean[1] -= m_Y[0];
      m_Mean[2] -= m_Theta[0];

      m_Y.pop_front();
      m_X.pop_front();
      m_Theta.pop_front();
    }

  double nbels = (double)m_X.size();
  X = -(m_Mean[0]/nbels-128.0)* (0.1/128.0);
  Y = -(m_Mean[1]/nbels-128.0)* (0.05/128.0);
  Theta = -(m_Mean[2]/nbels-128.0)* 360;
  return 1;
}

void Joystick::print()
{
  printf("btn 0: %s  btn 1: %s  X: %4d  Y: %4d\r",
	 (m_js.buttons & 1) ? "on " : "off", (m_js.buttons & 2) ? "on " : "off",
	 m_js.x, m_js.y);
  fflush(stdout);
}

