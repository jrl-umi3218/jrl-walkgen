/*
 * Copyright 2011,
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

  /// \brief Final optimization problem.
  /// Store and solve a quadratic problem with linear constraints.
  ///
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

      /// \brief Solution vector
      boost_ublas::vector<double> Solution_vec;

      /// \brief Lagrange multipliers of the constraints
      boost_ublas::vector<double> ConstrLagr_vec;
      /// \brief Lagrange multipliers of the lower bounds
      boost_ublas::vector<double> LBoundsLagr_vec;
      /// \brief Lagrange multipliers of the upper bounds
      boost_ublas::vector<double> UBoundsLagr_vec;

      /// \brief Resize solution containers
      void resize( int nb_variables, int nb_constraints );

      /// \brief Dump solution
      /// \param filename
      void dump( const char *filename );
      /// \brief Print solution
      /// \param aos
      void print( std::ostream & aos);

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
    void set_nb_variables( int NbVariables )
    { NbVariables_ = NbVariables;};

    /// \brief Set the number of equality constraints.
    ///
    /// \param nb_eq_constraints
    inline void set_nb_eq_constraints( int NbEqConstraints )
    { NbEqConstraints_ = NbEqConstraints;};

    /// \brief Set the total number of constraints.
    ///
    /// \param nb_constraints
    inline void set_nb_constraints( int NbConstraints )
    { NbConstraints_ = NbConstraints;};
    
    /// \brief Get the total number of constraints.
    ///
    /// \param nb_constraints
    inline int NbConstraints()
    { return NbConstraints_;};

    /// \brief Reallocate array
    ///
    /// \param[in] array
    /// \param[in] old_size
    /// \param[in] new_size
    template <typename type>
    int resize( type * &array, int old_size, int new_size );


    /// \brief Add a matrix to the final optimization problem in array form
    ///
    /// \param Mat Added matrix
    /// \param type Target matrix type
    /// \param row First row inside the target
    /// \param col First column inside the target
    void add_term(const MAL_MATRIX (&Mat, double), int type,
		 int row, int col);

    /// \brief Add a vector to the final optimization problem in array form
    ///
    /// \param Mat Added vector
    /// \param ype Target vector type
    /// \param row First row inside the target
    void add_term(const MAL_VECTOR (&Vec, double), int type,
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
    // Private methods
    //
  private:

    /// \brief Release memory.
    void release_memory();

    /// \brief Allocate memory.
    /// Called when setting the dimensions of the problem.
    ///
    void resize_all();

    /// \name Dumping functions
    /// \{
    /// \brief Print on disk the parameters that are passed to the solver
    void dump_solver_parameters(std::ostream & aos);
    /// \brief Print array
    void dump( int type, std::ostream & aos);
    /// \brief Print problem
    void dump_problem(std::ostream &);
    /// \}

    //
    //Private types
    //
  private:

    /// \brief Handle matrices/vectors in array form
    template<typename type>
    struct array_s
    {
      type * array_;

      int id_;
      int nrows_, ncols_;
      unsigned int memsize_;

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
      int stick_together(struct array_s<type> & final_array,
			 int nrows, int ncols)
      {
        try {
	  type * NewArray = 0;
	  if ((final_array.memsize_<nrows*ncols) ||
	      (final_array.array_==0))
	    {
	      final_array.array_ = new type[nrows*ncols];
	      final_array.memsize_ = nrows*ncols;
	    }
	  NewArray = final_array.array_;
	  
          fill(NewArray, nrows*ncols, (type)0);
          for(int i = 0; i < nrows; i++)
            for(int j = 0; j < ncols; j++)
              NewArray[i+nrows*j] = array_[i+nrows_*j];

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
	  bool b_reallocate = false;
	  type * NewArray = 0;
	  if (nrows*ncols>memsize_)
	    {
	      NewArray = new type[nrows*ncols];
	      memsize_ = nrows*ncols;
	      b_reallocate = true;
	      std::cout << "memsize_ " << memsize_ << std::endl;
	    }
	  else NewArray = array_;

	  fill(NewArray, nrows*ncols, (type)0);
	  if ((preserve) && 
	      (array_!=0) ) {
	    for(int i = 0; i < nrows_; i++)
	      for(int j = 0; j < ncols_; j++)
		NewArray[i+nrows*j] = array_[i+nrows_*j]; }

	  if ((array_!=0) && b_reallocate)
	    {
	      delete [] array_;
	    }
	  array_ = NewArray;
	  
          nrows_ = nrows;
          ncols_ = ncols;
        }
        catch (std::bad_alloc& ba)
        {std::cerr << "bad_alloc caught: " << ba.what() << std::endl; }

        return 0;
      }

      array_s():
        array_(0),id_(0),nrows_(0),ncols_(0), memsize_(0){
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
    int m_, me_, mmax_, n_, nmax_, mnn_;
    array_s<double> Q_, Q_dense_, D_, DU_, DU_dense_, DS_, XL_, XU_, X_, U_, war_;
    array_s<int> iwar_;
    int iout_, ifail_, iprint_, lwar_, liwar_;
    double Eps_;
    /// \}

    /// \brief Number of optimization parameters
    int NbVariables_;

    /// \brief Total number of constraints
    int NbConstraints_;

    /// \brief Number of equality constraints
    int NbEqConstraints_;
  };
  typedef struct QPProblem_s QPProblem;
}
#include <ZMPRefTrajectoryGeneration/qp-problem.hxx>
#endif /* _QP_PROBLEM_H_ */
