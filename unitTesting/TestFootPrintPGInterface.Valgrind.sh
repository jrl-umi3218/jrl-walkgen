#
# Copyright 2006, 2007, 2008, 2009, 2010, 
#
# Olivier Stasse
#
# JRL, CNRS/AIST
#
# This file is part of walkGenJrl.
# walkGenJrl is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# walkGenJrl is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
#
#  Research carried out within the scope of the 
#  Joint Japanese-French Robotics Laboratory (JRL)
#
#!/bin/bash
export HRP2JRLDIR=${HOME}/src/OpenHRP/Controller/IOserver/robot/HRP2JRL
export ETCDIR=${HRP2JRLDIR}/etc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PWD}/../src/
echo $LD_LIBRARY_PATH
valgrind --leak-check=full --error-limit=no -v --gen-suppressions=yes  ./TestFootPrintPGInterface ${ETCDIR}/PreviewControlParameters.ini ${HRP2JRLDIR}/model/ HRP2JRLmain.wrl ${ETCDIR}/HRP2Specificities.xml ${ETCDIR}/HRP2LinkJointRank.xml &> output_valgrind.dat
