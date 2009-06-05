#!/bin/bash
export HRP2JRLDIR=${HOME}/src/OpenHRP/Controller/IOserver/robot/HRP2JRL
export ETCDIR=${HRP2JRLDIR}/etc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PWD}/../src/
echo $LD_LIBRARY_PATH
valgrind --leak-check=full --error-limit=no -v  --gen-suppressions=all --suppressions=file.supp  --max-stackframe=57626480 ./TestFootPrintPGInterface  &> output_valgrind.dat
