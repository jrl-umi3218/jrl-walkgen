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


#ifndef __PG_MATRIX_ABSTRACT_LAYER_H_
#define __PG_MATRIX_ABSTRACT_LAYER_H_

#include "MatrixAbstractLayerDoc.h"

#include "configJRLWPG.h"

#ifdef _VNL_MATRIX_
#include "MatrixAbstractLayerVNL.h"
#elif defined _BOOST_MATRIX_
#include "MatrixAbstractLayerBoost.h"
#else
#error "You need to choose one linear algebra package. Please reconfigure with option --with-boost-sandbox=PATH_TO_BOOST_SANDBOX or --with-VNL=PATH_TO_VNL"
#endif


//#include "MatrixAbstractLayerSmall.h"
#include "MatrixAbstractLayerSmallVector3Default.h"
#include "MatrixAbstractLayerSmallMatrix3x3Default.h"

#define MATRIX_VERSION 1
#endif
