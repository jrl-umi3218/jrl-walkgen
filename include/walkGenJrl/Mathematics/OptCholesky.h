/* \file Cholesky.h
   \brief This object provides the Cholesky decomposition of a symmetric positive-definite matrix.
   This one is especially designed for QP resolution and handle constraints.

   Copyright (c) 2009, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CHOLESKY_DEC_H_
#define _CHOLESKY_DEC_H_

#include <vector>
#include <walkGenJrl/walkGenJrl_API.h>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

using namespace::std;

namespace PatternGeneratorJRL
{
  /*! This object provides the Cholesky decomposition of a symmetric positive-definite matrix.
   This one is especially designed for QP resolution and is designed specifically
   to grow from a set of rows extracted from a constraint matrix \$f{\bf A} \f$ of size \f$(m,\|u\|)\f$.   
   */  
  class WALK_GEN_JRL_EXPORT OptCholesky
    {
    
    public:
      /*! \brief Constructor 
	@param[in] lNbMaxOfConstraints \f$ m \f$ the number of constraints of matrix \f$ \bf A \f$
	@param[in] lCardU \f$ \|{\bf u}\| \f$ the size of vector \f$ {\bf u} \f$.
       */
      OptCholesky(unsigned int lNbMaxOfConstraints, unsigned int lCardU);

      /*! \brief Destructor */
      ~OptCholesky();

      /*! \brief Specify the fixed constraint matrix \f${\bf A}\f$*/
      void SetA(double * aA);
      
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
      int ComputeNormalCholeskyOnA();

      /*! \brief Compute the inverse of the Cholesky decomposition.*/
      int ComputeInverseCholesky(int mode);

      /*! \brief Specify the cholesky matrix L */
      void SetL(double *aL);

      /*! \brief Specify the inverse cholesky matrix L */
      void SetiL(double *aiL);

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

      /*! \brief Set of active constraintes. The maximum size is  \f$ m \f$,
	Its size gives the size of \f$ {\bf L} \f$, and \f$ {\bf E} \f$ */
      vector<unsigned int> m_SetActiveConstraints;

      /*! \brief Update Cholesky computation */
      int UpdateCholeskyMatrix();

      /*! \brief  Free memory. */
      void FreeMemory();

      /*! \brief Initialization of the internal variables. */
      void InitializeInternalVariables();
      
      
    };
};


#endif /* _CHOLESKY_DEC_H_ */
