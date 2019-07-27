/** \file Bsplines.h
    \brief Bsplines object for generating trajectoire 
    from set of Points given. */


#ifndef _BSPLINES_H_
#define _BSPLINES_H_

#include <vector>
#include <deque>
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
    Bsplines(long int degree);

    /*! Destructor */
    ~Bsplines();

    /*! Caculate Degree of Bsplines from m_control_points and m_knot*/
    void GenerateDegree();

    /*! Create a Knot Vector from m_degree and
      m_control_points with an algo "method" */
    //void GenerateKnotVector(std::string method);

    /*! Create a derivative Bsplines*/
    Bsplines DerivativeBsplines();

    /*!Compute Basic Function and its first and second derivatives*/
    int ComputeBasisFunctions(double t);

    // computes the basis function without the derivatives
    int ComputeBasisFunctionsRecursively
    (double t, std::deque<double> &knot, unsigned int degree);
    double Nij_t(int i, int j, double t, std::deque<double> & knot) ;


    /*!Compute Bsplines */
    double ComputeBsplines(double t);

    /*! Set Degree */
    void SetDegree(long int degree);

    /*! Set Control Points */
    void SetControlPoints(std::vector<double> &control_points) ;

    /*! Set Knot Vector */
    void SetKnotVector(std::deque<double> &knot_vector) ;

    /*! Get Degree */
    long int GetDegree() const;

    /*! Get Control Points */
    std::vector<double> GetControlPoints() const;

    /*! Get Knot Vector*/
    std::deque<double> GetKnotVector() const;

    void PrintKnotVector() const;

    void PrintControlPoints() const;

    void PrintDegree() const;

  protected:

    long int m_degree;

    std::vector<double> m_control_points;
    std::vector<double> m_derivative_control_points;
    std::vector<double> m_sec_derivative_control_points;

    std::vector< std::vector<double> > m_basis_functions ;
    std::vector<double> m_basis_functions_derivative ;
    std::vector<double> m_basis_functions_sec_derivative ;

    std::deque<double> m_knot;
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
    BSplinesFoot( double FT=1.0,
                  double IP=0.0,
                  double FP=0.0,
                  std::vector<double>ToMP = std::vector<double>(),
                  std::vector<double> MP = std::vector<double>(),
                  double IS = 0.0, double IA = 0.0,
                  double FS = 0.0, double FA = 0.0);

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
                       std::vector<double> MP,
                       double IS = 0.0, double IA = 0.0,
                       double FS = 0.0, double FA = 0.0);
    void SetParametersWithoutMPAndToMP(double FT,
                                       double IP,
                                       double FP,
                                       double IS, double IA,
                                       double FS, double FA);

    /*!Compute Position at time t */
    int Compute(double t, double &x, double &dx, double &ddx);

    /*! Compute the control point position for an order 5
     * Bsplines. It also computes the control point of the derivative
     * and the second derivatice of the BSplines.
     */
    void ComputeControlPointFrom2DataPoint();
    void ComputeControlPointFrom3DataPoint();
    void ComputeControlPointFrom4DataPoint();

    void GetParameters(double &FT,
                       double &IP,
                       double &FP,
                       std::vector<double> &ToMP,
                       std::vector<double> &MP);

    std::vector<double> MP()
    {
      return m_MP;
    }

    std::vector<double> ToMP()
    {
      return m_ToMP;
    }

    double FT()
    {
      return m_FT;
    }

    void FT(double ft)
    {
      m_FT=ft;
    }

    double IP()
    {
      return m_IP;
    }

    double FP()
    {
      return m_FP;
    }

    void  SetParametersWithInitFinalPose(double FT,
                                         double IP,
                                         double FP,
                                         std::vector<double> &ToMP,
                                         std::vector<double> &MP);

  private:

    double m_FT ; // final time
    double m_IP ; // Initial Position
    double m_IS ; // Initial Speed
    double m_IA ; // Initial Acceleration
    double m_FP ; // Final Position
    double m_FS ; // Final Speed
    double m_FA ; // Final Acceleration
    std::vector<double> m_ToMP
    ; // times to reach the middle (intermediate) positions
    std::vector<double> m_MP ; // middle (intermediate) positions
  };

}
#endif /* _BSPLINES_H_*/
