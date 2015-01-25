/** \file Bsplines.h
    \brief Bsplines object for generating trajectoire from set of Points given. */


#ifndef _BSPLINES_H_
#define _BSPLINES_H_

#include <vector>
#include <iostream>
#include <math.h>


struct Point
{
    double x;
    double y;
};

namespace PatternGeneratorJRL
{

/** Bspline class */
  class  Bsplines
    {

    public:
        /*! Constructor */
        Bsplines(int degree);

        /*! Destructor */
        ~Bsplines();

        /*! Caculate Degree of Bsplines from m_control_points and m_knot_vector*/
        void GenerateDegree();

        /*! Create a Knot Vector from m_degree and m_control_points with an algo "method" */
        //void GenerateKnotVector(std::string method);

        /*! Create a derivative Bsplines*/
        Bsplines DerivativeBsplines();

        /*!Compute Basic Function */
        std::vector<double> ComputeBasisFunction(double t);

        /*!Compute Bsplines */
        double ComputeBsplines(double t);

        /*! Set Degree */
        void SetDegree(int degree);

        /*! Set Control Points */
        void SetControlPoints(std::vector<double> &control_points) ;

        /*! Set Knot Vector */
        void SetKnotVector(std::vector<double> &knot_vector) ;

        /*! Get Degree */
        int GetDegree() const;

        /*! Get Control Points */
        std::vector<double> GetControlPoints() const;

        /*! Get Knot Vector*/
        std::vector<double> GetKnotVector() const;

        void PrintKnotVector() const;

        void PrintControlPoints() const;

        void PrintDegree() const;

    protected:

        unsigned int m_degree;

        std::vector<double> m_control_points;
        std::vector<double> m_knot_vector;
    };

   /// Bsplines used for Z trajectory of stair steps
  class BSplinesFoot : public Bsplines
  {
      public:
      /** Constructor:
       FT: Final time
       FP: Final position
       ToMP : Time of Max Position
       MP : Max Position */
      BSplinesFoot( double FT=0.0,
                    double FP=0.0,
                    std::vector<double>ToMP = std::vector<double>(),
                    std::vector<double> MP = std::vector<double>());

      /*!Compute Position at time t */
      double Compute(double t);

        /*!Compute Velocity at time t */
      double ComputeDerivative(double t);

        /*!Compute Acceleration at time t */
      double ComputeSecDerivative(double t);

      /** Detructor **/
      ~BSplinesFoot();

      /*!  Set the parameters
	  This method assumes implicitly a initial position
	  initial speed and initial acceleration equal to zero.
	  The same for final speed and final acceleration.
	  Speed at Max Position is around zero.
	  It generates knot vector and control point vector
	  according to the input
       */
      void SetParameters(double FT,
                         double IP,
                         double FP,
                         std::vector<double>ToMP,
                         std::vector<double> MP);

      void GetParameters(double &FT,
                         double &IP,
                         double &FP,
                         std::vector<double> &ToMP,
                         std::vector<double> &MP);

      std::vector<double> MP()
      {return m_MP;}

      std::vector<double> ToMP()
      {return m_ToMP;}

      double FT()
      {return m_FT;}

      double IP()
      {return m_IP;}

      double FP()
      {return m_FP;}

      private:
      double m_FT ; // final time
      double m_IP ; // Initial Position
      double m_FP ; // Final Position
      std::vector<double> m_ToMP ; // times to reach the middle (intermediate) positions
      std::vector<double> m_MP ; // middle (intermediate) positions
  };

}
#endif /* _BSPLINES_H_*/
