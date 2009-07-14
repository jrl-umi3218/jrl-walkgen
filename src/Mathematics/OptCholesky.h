/* \file Cholesky.h
   \brief This object provides the Cholesky decomposition of a symmetric positive-definite matrix.
   This one is especially designed for QP resolution and handle constraints.

   Copyright (c) 2009, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please look at License.txt for more information on license.
*/

#ifndef _CHOLESKY_DEC_H_
#define _CHOLESKY_DEC_H_

#include <vector>


#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

using namespace::std;

namespace PatternGeneratorJRL
{
  /*! This object provides the Cholesky decomposition of a symmetric positive-definite matrix.
   This one is especially designed for QP resolution and is designed specifically
   to grow from a set of rows extracted from a constraint matrix \$f{\bf A} \f$ of size \f$(m,\|u\|)\f$.   
   */  
  class  OptCholesky
    {
    
    public:
      /*! \brief Constructor 
	@param[in] lNbMaxOfConstraints \f$ m \f$ the number of constraints of matrix \f$ \bf A \f$
	@param[in] lCardU \f$ \|{\bf u}\| \f$ the size of vector \f$ {\bf u} \f$.
       */
      OptCholesky(unsigned int lNbMaxOfConstraints, unsigned int lCardU,
		  unsigned int mode);

      /*! \brief Destructor */
      ~OptCholesky();

      /*! \brief Specify the fixed constraint matrix \f${\bf A}\f$*/
      void SetA(double * aA,
		unsigned int lNbOfConstraints);
      
      /*! \brief Add a list of active constraints
	@param[in] lConstraints: row indexes of constraints in \f${\bf A} \f$.
	@return \f$ -i \f$ where \f$ i \f$ is the constraint for which there is a problem.
       */
      int AddActiveConstraints(vector<unsigned int> & lConstraints);

      /*! \brief Add one active constraint 
	@param[in] lConstraints: row indexes of constraints in \f${\bf A} \f$.
	@return -1 if the constraint was not added, 0 otherwise.
       */
      int AddActiveConstraint(unsigned int aConstraint);

      /*! \brief Returns the current number of rows 
       or the current number of active constraints on \f$ {\bf A} \f$.*/
      int CurrentNumberOfRows();

      /*! \brief Compute normal Cholesky decomposition on A (if possible).*/
      int ComputeNormalCholeskyOnANormal();


      /*! \brief Compute the inverse of the Cholesky decomposition.*/
      int ComputeInverseCholeskyNormal(int mode);

      /*! \brief Specify the cholesky matrix L */
      void SetL(double *aL);

      /*! \brief Specify the inverse cholesky matrix L */
      void SetiL(double *aiL);

      /*! \brief Set the algorithm to zero. */
      void SetToZero();
      
      /*! \brief Set Mode to update the cholesky matrix */
      void SetMode(unsigned int mode);

      /*! \brief Various mode to compute cholesky matrix. */
      static const unsigned int MODE_NORMAL=0;
      static const unsigned int MODE_FORTRAN=1;
      
    private:

      /*! \brief Size of the constraints, i.e. maximum number of rows for \f$ E E^{\top} \f$ */
      unsigned int m_NbMaxOfConstraints;

      /*! \brief Size of the control vector \f$ {\bf u} \f$, i.e. the number of columns for \f$ E \f$ */
      unsigned int m_CardU;

      /*! \brief Pointeur towards the initial constraint matrix \f$ {\bf A} \f$ of size \f$ (m,\|u\|) \f$. */
      double *m_A;

      /*! \brief The cholesky decomposition result: the \f$ {\bf L} \f$ matrix. */
      double *m_L;

      /*! \brief The inversecholesky decomposition result: the \f$ {\bf L} \f$ matrix. */
      double *m_iL;

      /*! \brief Mode to update the cholesky. */
      unsigned int m_UpdateMode;

      /*! \brief Number of constraints related with matrix A. */
      unsigned int m_NbOfConstraints;

      /*! \brief Set of active constraintes. The maximum size is  \f$ m \f$,
	Its size gives the size of \f$ {\bf L} \f$, and \f$ {\bf E} \f$ */
      vector<unsigned int> m_SetActiveConstraints;


      /*! \brief Update Cholesky computation */
      int UpdateCholeskyMatrixFortran();

      /*! \brief Update Cholesky computation */
      int UpdateCholeskyMatrixNormal();

      /*! \brief  Free memory. */
      void FreeMemory();

      /*! \brief Initialization of the internal variables. */
      void InitializeInternalVariables();
      
      
    };
};


#endif /* _CHOLESKY_DEC_H_ */
