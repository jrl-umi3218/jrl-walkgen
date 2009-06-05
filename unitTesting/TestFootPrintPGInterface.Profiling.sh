#!/bin/bash
export HRP2JRLDIR=${HOME}/src/OpenHRP/Controller/IOserver/robot/HRP2JRL
export ETCDIR=${HRP2JRLDIR}/etc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PWD}/../src/
echo $LD_LIBRARY_PATH
valgrind --tool=cachegrind --branch-sim=yes ./TestFootPrintPGInterface  &> output_valgrind.dat
