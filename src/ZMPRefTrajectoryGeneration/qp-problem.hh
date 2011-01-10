/*
 * Copyright 2010,
 *
 * Medhi    Benallegue
 * Andrei   Herdt
 * Francois Keith
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
#ifndef _QP_PROBLEM_H_
#define _QP_PROBLEM_H_

#include <jrl/mal/matrixabstractlayer.hh>
#include <Mathematics/qld.h>

namespace PatternGeneratorJRL
{

  /*! \brief Final optimization problem.
    This object stores and solves a quadratic problem with linear constraints.
  */
  struct QPProblem_s
  {
    int m, me, mmax, n, nmax, mnn;
    double *Q, *D, *DU, *DS, *XL, *XU, *X, *NewX, *U, *war;//For COM
    int *iwar;
    int iout, ifail, iprint, lwar, liwar;
    double Eps;

    const static int MATRIX_Q=0;
    const static int MATRIX_DU=1;
    const static int VECTOR_D=2;
    const static int VECTOR_DS=3;
    const static int VECTOR_XL=4;
    const static int VECTOR_XU=5;

    const static int QLD=10;
    const static int PLDP=11;

    //
    //Public methods
    //

    /// \brief Initialize by default an empty problem.
    QPProblem_s();

    /// \brief Release the memory at the end only.
    ~QPProblem_s();

    /// \brief Set the number of optimization parameters.
    inline void setNbVariables(const int & NbVariables)
    { m_NbVariables = NbVariables;};

    /// \brief Set the dimensions of the problem.
    /// This method has an internal logic to 
    /// allocate the memory. It is done only
    /// when the problem gets bigger. When it shrinks
    /// down the memory is kept to avoid overhead.
    void setDimensions(const int & NbVariables,
		       const int & NbConstraints,
		       const int & NbEqConstraints);
    
    /// \brief Reallocate array
    ///
    /// \name array
    /// \name old_size
    /// \name new_size
        template <class type>
        int resize(type * &array, const int & old_size, const int & new_size)
        { 
	  try 
	    { 
	      type * NewArray = new type[new_size];
	      initialize(NewArray, new_size, (type)0);
	      for(int i = 0; i < old_size; i++)
		{
		  NewArray[i] = array[i]; std::cout<<NewArray[i]<<" xx ";
		}
	      std::cout<<std::endl;
	      if (array!=0)
		delete [] array;
	      array = NewArray; 
	    }
	  catch (std::bad_alloc& ba)
	    {std::cerr << "bad_alloc caught: " << ba.what() << std::endl; }
	  return 0;}


    /// \brief Add a matrix to the final optimization problem in array form
    ///
    /// \param Mat Added matrix
    /// \param target Target matrix
    /// \param row First row inside the target
    /// \param col First column inside the target
    void addTerm(const MAL_MATRIX (&Mat, double), const int type,
		 const int row, const int col);
    /// \brief Add a vector to the final optimization problem in array form
    ///
    /// \param Mat Added vector
    /// \param target Target vector type
    /// \param row First row inside the target
    void addTerm(const MAL_VECTOR (&Vec, double), const int type,
		 const int row);

    /// \brief Print of disk the parameters that are passed to the solver
    void dumpSolverParameters(std::ostream & aos);

    /// \brief Dump on disk a problem.
    void dumpProblem(const char *filename);
    void dumpProblem(std::ostream &);

    /// \brief Dump on disk an array.
    ///
    /// \param type
    /// \param filename
    void dump(const int type, const char *filename);
    void dump(const int type, std::ostream &);

    /// \brief Initialize array
    ///
    /// \param array
    /// \param size
    template <class type> void initialize(type * array, const int & size, type value)
    { std::fill_n(array,size,value); }

    /// \brief Initialize whole array
    ///
    /// \param type
    void clear(const int type);

    /// \brief Initialize block of matrix-array
    ///
    /// \param type
    void clear(const int type,
	       const int & row, const int & col,
	       const int & nb_rows, const int & nb_cols);

    /// \brief Solve the problem
    void solve(const int solver);

    //
    //Protected methods
    //
  protected:

    /// The method doing the real job of releasing the memory.
    void ReleaseMemory();

    /// The method allocating the memory.
    /// Called when setting the dimensions of the problem.
    void resizeAll(const int & NbVariables, const int & NbConstraints);

    //
    //Private members
    //
  private:

    /// \brief Number of optimization parameters
    int m_NbVariables;

    /// \brief Number of optimization parameters
    int m_NbConstraints;

    /// \brief Reallocation margins
    int m_ReallocMarginVar, m_ReallocMarginConstr;

  };
  typedef struct QPProblem_s QPProblem;
}
#endif /* _QP_PROBLEM_H_ */
