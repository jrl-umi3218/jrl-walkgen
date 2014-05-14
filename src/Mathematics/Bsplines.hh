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
        void GenerateKnotVector(std::string method);

        /*! Create a derivative Bsplines*/
        Bsplines DerivativeBsplines();

        /*!Compute Basic Function */
        double *ComputeBasisFunction(double t);

        /*!Compute Bsplines */
        Point ComputeBsplines(double t);

        /*! Set Degree */
        void SetDegree(int degree);

        /*! Set Control Points */
        void SetControlPoints(std::vector<Point> &control_points) ;

        /*! Set Knot Vector */
        void SetKnotVector(std::vector<double> &knot_vector) ;

        /*! Get Degree */
        int GetDegree() const;

        /*! Get Control Points */
        std::vector<Point> GetControlPoints() const;

        /*! Get Knot Vector*/
        std::vector<double> GetKnotVector() const;

        void PrintKnotVector() const;

        void PrintControlPoints() const;

        void PrintDegree() const;

    protected:

        int m_degree;

        std::vector<Point> m_control_points;

        std::vector<double> m_knot_vector;
    };

   /// Bsplines used for Z trajectory of stair steps
  class ZBsplines : public Bsplines
  {
      public:
      /** Constructor:
       FT: Final time
       FP: Final position
       ToMP : Time of Max Position
       MP : Max Position */
      ZBsplines( double FT, double FP, double ToMP, double MP);

      /*!Compute Position at time t */
      double ZComputePosition(double t);

        /*!Compute Velocity at time t */
      double ZComputeVelocity(double t);

        /*!Compute Acceleration at time t */
      double ZComputeAcc(double t);

      /** Detructor **/
      ~ZBsplines();

      /*!  Set the parameters
	  This method assumes implicitly a initial position
	  initial speed and initial acceleration equal to zero.
	  The same for final speed and final acceleration.
	  Speed at Max Position is around zero.
       */
      void SetParameters(double FT, double FP, double ToMP, double MP);

      void SetParametersWithInitPos(double IP, double FT, double FP, double ToMP, double MP);

      void GetParametersWithInitPosInitSpeed(double &FT,
						  double &FP,
						  double &InitPos,
						  double &InitSpeed);

      /*! Create a vector of Control Points with 8 Points :
      {0.0,0.0},
      {m_FT*0.05,0.0},
      {m_FT*0.1,0.0}},
      {0.85*m_ToMP,m_MP},
      {1.15*m_ToMP,m_MP},
      {0.85*m_FT,m_FP},
      {0.9*m_FT,m_FP},
      {m_FT,m_FP}*/
      void ZGenerateControlPoints(double IP,double FT, double FP, double ToMP, double MP);

      void ZGenerateKnotVector(double FT, double ToMP);

      private:
      double m_IP, m_FT, m_FP, m_ToMP, m_MP;
  };

}
#endif /* _BSPLINES_H_*/
