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
   to grow from a set of rows extracted from a constraint matrix.   
   */  
  class WALK_GEN_JRL_EXPORT OptCholesky
    {
    
    public:
      /*! Constructor */
      OptCholesky();

      /*! Destructor */
      ~OptCholesky();

      
    private:

      /*! \brief Size of the constraints, i.e. maximum number of rows for \f$ E \f$ */
      unsigned int m_m;

      /*! \brief Current 

      /*! \brief Size of the control vector\f$u\f$, i.e. the number of columns for \f$ E \f$ */
      unsigned int m_CardU;

      /*! \brief Pointeur towards the initial constraint matrix \f$A\f$ of size \f$(m,\|u\|)\f$. */
      double *m_A;

      /*! \brief Set of active constraintes. The maximum size is  \f$m\f$*/
      
      
    };
};


#endif /* _CHOLESKY_DEC_H_ */
