#!/bin/bash
export HRP2JRLDIR=${HOME}/src/OpenHRP/Controller/IOserver/robot/HRP2JRL
export ETCDIR=${HRP2JRLDIR}/etc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PWD}/../src/
echo $LD_LIBRARY_PATH
valgrind --leak-check=full --error-limit=no -v --gen-suppressions=yes  ./TestFootPrintPGInterface ${ETCDIR}/PreviewControlParameters.ini ${HRP2JRLDIR}/model/ HRP2JRLmain.wrl ${ETCDIR}/HRP2Specificities.xml ${ETCDIR}/HRP2LinkJointRank.xml &> output_valgrind.dat
