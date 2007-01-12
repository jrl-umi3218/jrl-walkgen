/**
 *
 * \file MatrixAbstractLayer.h
 *
 * @ingroup linearalgebra
 * This header file intend to define an abstraction layer
 * to allow the use of diverse matrix libraries.
 * This is supposed to solve the dependency to a particular
 * matrix library.
 * If you do not like the current two solutions:
 *  - Oxford VNL
 *  - uBLAS + LAPACK 
 *
 * you can implement your own.
 *
 *
 * For small matrices it is sometimes more efficient
 * to use specific data structure. 
 * For this reason an abstract interface for 
 * small 3x3 and 4x4 matrices as well as vectors 
 * is also provided
 *
 * We however assume that the library used 
 * is based on C++ classes and has a sensible implementation
 * of matrix operators.
 * 
 * (c) 2006 , Olivier Stasse JRL-Japan, CNRS-AIST, ISRI.
 */


#ifndef _MATRIX_ABSTRACT_LAYER_H_
#define _MATRIX_ABSTRACT_LAYER_H_

#include "MatrixAbstractLayerDoc.h"

#include "MatrixAbstractLayerVNL.h"
#include "MatrixAbstractLayerBoost.h"

#include "MatrixAbstractLayerSmall.h"
#include "MatrixAbstractLayerSmallVector3Default.h"
#include "MatrixAbstractLayerSmallMatrix3x3Default.h"

#define MATRIX_VERSION 1
#endif
