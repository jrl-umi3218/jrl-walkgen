/*
 * Copyright 2011, 
 *
 * Olivier Stasse,
 * 
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */
/** \page Installing Installing


\section Dependencies
This package depends upon the following packages:
<ul>
  <li> System Packages: </li>
  <ul>
    <li> boost (tested on 1.40 and 1.43)</li>
    <li> pkgconfig </li>
    <li> cmake </li>
  </ul>
  <li> JRL packages: </li>
  <ul>
   <li> jrl-mal</li>
   <li> abstract-robot-dynamics</li>
   <li> jrl-dynamics </li>
  </ul>
</ul>

\section Compiling

<ol>
<li> Uncompress the source package<br>
tar zxvf jrl-walkgen.tgz </li>
<li> Create a build directory and go inside it: <br>
mkdir build<br></li>
<li> Setup the installation with :<br>
cmake -DCMAKE_INSTALL_PREFIX=/your/prefix -DCMAKE_BUILD_TYPE=RELEASE ..<br></li>
<li> Compile with: <br>
make <br></li>
<li> The installation is realized with: <br>
make install <br></li>
<li> The documentation can be generated with: <br>
make doc <br></li>
<li> A set of unitary tests can be launched with: <br>
make test <br></li>
</ol>

*/
