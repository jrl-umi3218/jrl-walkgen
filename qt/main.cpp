//
//    File: main.cc
//
//    (C) 2000 Helmut Cantzler
//
//    Licensed under the terms of the Lesser General Public License.
//

#include <qapplication.h>
#include <qgl.h>

#include "MainView.h"

int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  if (!QGLFormat::hasOpenGL())
    {
      qWarning("This QT libary has no OpenGL support. Rebuilt your libary with OpenGL support or download a proper rpm. Exiting.");
      return -1;
    }


  MainView m(argc, argv);

  a.setMainWidget(&m);
  m.show();
  return a.exec();
}
