/** \mainpage 
\section sec_intro Introduction

This library implements the Walking Pattern Generator as described by Kajita
in his IROS 2003 paper \ref Kajita2003 . It is build upon the preview control of the 3D inverted pendulum.
Compare to the commercial version shipped with the HRP-2,
new functionnalities were added. Those functionnalities are dynamical stepping-over
and upper-body motion.


\section sec_approach Approach implemented

The immediate available reference for this code is \ref Kajita2003 .
But numerous details were given in Kajita's book \ref Kajita2005 .

\section sec_organization Organization of the code

The code relative to the Walking Pattern Generator library is divided into three parts:
\li the library itself,
\li a plugin which can be used inside the OpenHRP architecture and accessed
through CORBA,
\li several examples with a console interface,
\li a Qt GUI.

\image html archi.png "Architecture of module HumanoidPathPlanner: the module works with KineoWorks objects (devices for robots and bodies for obstacles). It is aimed at implementing path planning functions for Humanoid robots."
\image latex archi.pdf "Architecture of module HumanoidPathPlanner: the module works with KineoWorks objects (devices for robots and bodies for obstacles). It is aimed at implementing path planning functions for Humanoid robots."

\subsection subsec_WalkGenJRL_library The library libWalkGenJRL.so

Library libWalkGenJRL.so is aimed at implementing a walking pattern generator for humanoid robots 
having 6 DOFs per legs, and for which a VRML file is available.

\subsubsection subsubsec_WalkGenJRL_plugin Plugin 

The plugin allows to use the walking pattern generator inside the OpenHRP architecture.
As it is also a CORBA server, it can be used also by other components of the robotic application.

\section sec_changes Changes between v1.0 and v2.0
The v2.0 library has the following new functionnalities:
\li It takes properly into account the upper body position inside the pattern generator.
\li Based on experimental data, the height of the ZMP can be changed also the Ricatti
system is not modified.
\li Bjorn Verrelst's stepping over is now fully integrated.
\li It is possible to specify way-points for the upper-body while walking.


\section References
\anchor Kajita2003,
Shuuji Kajita and Fumio Kanehiro and Kenji Kaneko and Kiyoshi Fujiwara and Kensuke Harada and Kazuhito Yokoi and Hirohisa Hirukawa, "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point",
International Conference on Robotics And Automation, 2003, Taipei Taiwan 

\anchor Kajita2005,
Shuuji Kajita, Omsha, Humanoid Robot ,
2005,(In Japanese) ISBN4-274-20058-2


\defgroup pgjrl JRL Walking Pattern Generator Library (WalkGenJRL)
This library is intended to implement walking pattern generator algorithms for humanoid robots.
This group is defined into subgroups:

\defgroup previewcontrol Preview Control
\ingroup pgjrl
This group implements the preview control algorithm as presented by Kajita in \ref Kajita2003.

\defgroup forwardynamics Forward Dynamics
\ingroup pgjrl
This group implements a forward dynamic algorithm in order to compute basic properties
of a robot described in the OpenHRP format (specialized VRML).

\defgroup pginterfaces  Interfaces to WalkGenJRL
This group shows how to interface the WalkGenJRL library to three kinds of applications:
\li An OpenHRP plugin, to run in real-time inside an HRP-2 humanoid robot,
\li A console program, for quick but yet complete testing of the pattern generator,
\li An OpenGL interface.


*/
