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
#include <privatepgtypes.h>

namespace PatternGeneratorJRL
{

  /*! \brief Final optimization problem.
    This object stores and solves a quadratic problem with linear constraints.
  */
  struct QPProblem_s
  {

    //
    // Public types
    //
  public:

    const static int MATRIX_Q=0;
    const static int MATRIX_DU=1;
    const static int VECTOR_D=2;
    const static int VECTOR_DS=3;
    const static int VECTOR_XL=4;
    const static int VECTOR_XU=5;

    const static int QLD=10;
    const static int PLDP=11;


    /// \brief Solution
    struct solution_s
    {

      /// \brief Size of the solution array
      int NbVariables;

      /// \brief Number of constraints (lagrange multipliers)
      int NbConstraints;

      /// \name qld-output
      /// \{
      /// \brief SHOWS THE TERMINATION REASON.
      ///   IFAIL = 0 :   SUCCESSFUL RETURN.
      ///   IFAIL = 1 :   TOO MANY ITERATIONS (MORE THAN 40*(N+M)).
      ///   IFAIL = 2 :   ACCURACY INSUFFICIENT TO SATISFY CONVERGENCE
      ///                 CRITERION.
      ///   IFAIL = 5 :   LENGTH OF A WORKING ARRAY IS TOO SHORT.
      ///   IFAIL > 10 :  THE CONSTRAINTS ARE INCONSISTENT.
      int Fail;

      /// \brief OUTPUT CONTROL.
      ///   IPRINT = 0 :  NO OUTPUT OF QL0001.
      ///   IPRINT > 0 :  BRIEF OUTPUT IN ERROR CASES.
      int Print;
      /// \}

      /// \brief Solution vector
      boost_ublas::vector<double> vecSolution;

      /// \name Splitted lagrange multipliers sets
      /// \{
      /// \brief Lagrange multipliers of the constraints
      boost_ublas::vector<double> vecConstrLagr;
      /// \brief Lagrange multipliers of the lower bounds
      boost_ublas::vector<double> vecLBoundsLagr;
      /// \brief Lagrange multipliers of the upper bounds
      boost_ublas::vector<double> vecUBoundsLagr;
      /// \}

      /// \brief Resize solution containers
      void resize( int nb_variables, int nb_constraints );

      /// \name Dumping
      /// \{
      /// \brief Dump solution
      void dump( const char *filename );
      void print( std::ostream & aos);
      /// \}
    };
    typedef struct solution_s solution_t;


    //
    //Public methods
    //
  public:

    /// \brief Initialize by default an empty problem.
    QPProblem_s();

    /// \brief Release the memory at the end only.
    ~QPProblem_s();

    /// \brief Set the number of optimization parameters.
    ///
    /// \param nb_variables
    void setNbVariables( int nb_variables )
    { nbvariables_ = nb_variables;};

    /// \brief Set the number of optimization parameters.
    ///
    /// \param nb_eq_constraints
    inline void setNbEqConstraints( int nb_eq_constraints )
    { nbeqconstraints_ = nb_eq_constraints;};

    /// \brief Set the number of optimization parameters.
    ///
    /// \param nb_constraints
    inline void setNbConstraints( int nb_constraints )
    { nbconstraints_ = nb_constraints;};

    
    /// \brief Reallocate array
    ///
    /// \param[in] array
    /// \param[in] old_size
    /// \param[in] new_size
    template <class type>
    int resize( type * &array, int old_size, int new_size )
    {
      try
      {
        type * NewArray = new type[new_size];
        initialize(NewArray, new_size, (type)0);
        for(int i = 0; i < old_size; i++)
          {
            NewArray[i] = array[i];
          }
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
    /// \param type Target matrix type
    /// \param row First row inside the target
    /// \param col First column inside the target
    void addTerm(const MAL_MATRIX (&Mat, double), int type,
		 int row, int col);

    /// \brief Add a vector to the final optimization problem in array form
    ///
    /// \param Mat Added vector
    /// \param ype Target vector type
    /// \param row First row inside the target
    void addTerm(const MAL_VECTOR (&Vec, double), int type,
		 int row);

    /// \brief Dump on disk a problem.
    void dumpProblem(const char *filename);

    /// \brief Dump on disk an array.
    ///
    /// \param type
    /// \param filename
    void dump( int type, const char *filename);
    /// \}

    /// \brief Initialize set of arrays
    void clear( );

    /// \brief Initialize whole array
    ///
    /// \param[in] type
    void clear( int type );

    /// \brief Solve the problem
    ///
    /// \param[in] solver
    /// \param[out] result
    void solve( int solver, solution_t & result);

    //
    //Protected methods
    //
  protected:

    /// \brief Relese memory.
    void releaseMemory();

    /// \brief Allocate memory.
    /// Called when setting the dimensions of the problem.
    ///
    /// \param[in] nb_variables
    /// \param[in] nb_constraints
    void resizeAll();

    //
    // Private methods
    //
  private:

    /// \name Dumping functions
    /// \{
    /// \brief Print on disk the parameters that are passed to the solver
    void dumpSolverParameters(std::ostream & aos);
    /// \brief Print array
    void dump( int type, std::ostream & aos);
    /// \brief Print problem
    void dumpProblem(std::ostream &);
    /// \}
    //
    //Private types
    //
  private:

    /// \brief Handle matrices/vectors in array form
    template<class type>
    struct array_s
    {
      type * array_;

      int id_;
      int nrows_, ncols_;

      void fill( type value)
      { std::fill_n(array_, nrows_*ncols_, value); }

      void fill( type * array, int size, type value)
      { std::fill_n(array, size, value); }


      /// \brief Make a contiguous array
      ///
      /// \param[in] nrows Size of the new array
      /// \param[in] ncols Size of the new array
      /// \param[in] preserve Preserve old values
      /// \return 0
      int stick_together(type *& final_array, int nrows, int ncols)
      {
        try {
          type * NewArray = new type[nrows*ncols];
          fill(NewArray, nrows*ncols, (type)0);
            for(int i = 0; i < nrows; i++)
              for(int j = 0; j < ncols; j++)
                NewArray[i+nrows*j] = array_[i+nrows_*j];
          if (final_array!=0)
            delete [] final_array;

          final_array = NewArray;
          nrows_ = nrows;
          ncols_ = ncols;
        }
        catch (std::bad_alloc& ba)
        {std::cerr << "bad_alloc caught: " << ba.what() << std::endl; }

        return 0;
      }

      /// \brief Resize array
      ///
      /// \param[in] nrows Size of the new array
      /// \param[in] ncols Size of the new array
      /// \param[in] preserve Preserve old values
      /// \return 0
      int resize(int nrows, int ncols, bool preserve)
      {
        try {
          type * NewArray = new type[nrows*ncols];
          fill(NewArray, nrows*ncols, (type)0);
          if(preserve) {
            for(int i = 0; i < nrows_; i++)
              for(int j = 0; j < ncols_; j++)
                NewArray[i+nrows*j] = array_[i+nrows_*j]; }
          if (array_!=0)
            delete [] array_;

          array_ = NewArray;
          nrows_ = nrows;
          ncols_ = ncols;
        }
        catch (std::bad_alloc& ba)
        {std::cerr << "bad_alloc caught: " << ba.what() << std::endl; }

        return 0;
      }

      array_s():
        array_(0),id_(0),nrows_(0),ncols_(0){
      };
      ~array_s()
      {  if (array_!=0) delete [] array_;};
    };

    //
    //Private members
    //
  private:

    /// \name ql-parameters
    /// \{
    int m, me, mmax, n, nmax, mnn;
    array_s<double> Q_, Q_dense_, D_, DU_, DU_dense_, DS_, XL_, XU_, X_, U_, war_;
    array_s<int> iwar_;
    int iout, ifail, iprint, lwar, liwar;
    double Eps;
    /// \}

    /// \brief Number of optimization parameters
    int nbvariables_;

    /// \brief Total number of constraints
    int nbconstraints_;

    /// \brief Number of equality constraints
    int nbeqconstraints_;

    /// \brief Reallocation margins
    int m_ReallocMarginVar, m_ReallocMarginConstr;

//    /// \brief Primal Least square Distance Problem solver
//    Optimization::Solver::PLDPSolverHerdt * m_PLDPSolverHerdt;

    int scale_factor_;

    /// \brief Maximal number of variables
    int max_var_;
    /// \brief Maximal number of constraints
    int max_constr_;
  };
  typedef struct QPProblem_s QPProblem;
}
#endif /* _QP_PROBLEM_H_ */
