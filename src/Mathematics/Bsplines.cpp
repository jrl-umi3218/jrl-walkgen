#include <iostream>
#include <vector>
#include <assert.h>

#include <Debug.hh>
#include <Mathematics/Bsplines.hh>


using namespace::std;
using namespace::PatternGeneratorJRL;


Bsplines::Bsplines(int degree)
{
    m_degree = degree;
    m_control_points.clear();
    m_knot_vector.clear();
}

Bsplines::~Bsplines()
{
}

void Bsplines::GenerateDegree()
{
    int degree = (m_knot_vector.size()-1) - (m_control_points.size()-1) -1;
    if (degree < 0)
    {
        cout << " Attention !! degree is smaller than 0 " << endl;
    }
    m_degree = (unsigned)degree ;
}

//void Bsplines::GenerateKnotVector(std::string method)
//{
//    /*Calculer set of parameters*/
//    unsigned int i,j;
//    vector<double> set_of_pam;

//        if (method == "centripetal")
//        {
//            //cout << "centripetal" << endl;
//            set_of_pam.clear();
//            set_of_pam.reserve(m_control_points.size());
//            cout << m_control_points.size() << endl;
//            double L = 0.0;
//            double D = 0.0;
//            for (i=0;i<m_control_points.size()-1;i++)
//            {
//                L += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
//                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
//            }
//            for (i=0;i<m_control_points.size();i++)
//            {
//                if (i == 0)
//                {
//                    set_of_pam.push_back(0.0);
//                }
//                else if (i == m_control_points.size()-1)
//                {
//                    set_of_pam.push_back(1.0);
//                }
//                else
//                {
//                    D += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
//                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
//                    set_of_pam.push_back(D/L);
//                }
//            }

//            m_knot_vector.clear();
//            double U = 0.0;
//            for (i=0;i<=m_degree;i++)
//            {
//                m_knot_vector.push_back(0.0);
//            }

//            if (m_control_points.size()-1>=m_degree)
//            {
//                for (j=1;j<=m_control_points.size()-1-m_degree;j++)
//                {
//                    i=j;
//                    U=0.0;
//                    while (i<=m_degree-1+j)
//                    {
//                        U +=set_of_pam[i];
//                        i++;
//                    }
//                    m_knot_vector.push_back(U/double(m_degree));
//                }
//            }

//            for (i=0;i<=m_degree;i++)
//            {
//                m_knot_vector.push_back(1.0);
//            }
//            if (m_knot_vector.size() - 1 != m_control_points.size() + m_degree)
//            {
//                cout << "Knot vector cant be created. m_control_points.size()-1>=m_degree "<< endl;
//                m_knot_vector.clear();
//            }

//        }

//        else if (method =="universal")
//        {
//            m_knot_vector.clear();
//            //cout << "universal" << endl;
//            double U=0;
//            for (i=0;i<=m_degree;i++)
//            {
//                m_knot_vector.push_back(0.0);
//            }
//            for (i=1;i<=m_control_points.size()-1-m_degree;i++)
//            {
//                U = double(i)/(m_control_points.size()-1-m_degree+1);
//                m_knot_vector.push_back(U);
//            }
//            for (i=0;i<=m_degree;i++)
//            {
//                m_knot_vector.push_back(1.0);
//            }

//        }
//}

vector<double> Bsplines::ComputeBasisFunction(double t)
{
  vector< vector<double> > m_basis_function (m_degree+1);
  unsigned int i,j,n;

  if (m_degree!= m_knot_vector.size() - m_control_points.size() -1 )
  {
      cout << "The parameters are not compatibles. Please recheck " << endl;
      return vector<double>() ;
  }
  for(j=0;j <= m_degree;j++)
  {
    n = m_knot_vector.size() - 1 - j -1;
    m_basis_function[j] = vector<double>(n+1,0.0);

    for(i=0;i <= n;i++)
    {
      if (j == 0 && m_knot_vector[i] <= t && t < m_knot_vector[i+1] || t == m_knot_vector.back() )
      {
        m_basis_function[j][i] = 1.0;
      }
      else if (j == 0)
      {
        m_basis_function[j][i] = 0.0;
      }
      else if (j != 0)
      {
        double first_term (0.0), second_term (0.0) ;

        if ( m_knot_vector[i] == m_knot_vector[i+j] )
          first_term = 0.0 ;
        else
          first_term = (t - m_knot_vector[i]) / (m_knot_vector[i+j]-m_knot_vector[i]) * m_basis_function[j-1][i] ;

        if (m_knot_vector[i+j+1] == m_knot_vector[i+1])
          second_term = 0.0 ;
        else
          second_term = (m_knot_vector[i+j+1] - t) / (m_knot_vector[i+j+1]-m_knot_vector[i+1] ) * m_basis_function[j-1][i+1] ;

        m_basis_function[j][i] = first_term + second_term  ;
      }
    }
  }
  return m_basis_function[m_degree];
}

double Bsplines::ComputeBsplines(double t)
{
    vector<double> m_basis_function = ComputeBasisFunction(t);
    double result = 0.0 ;
    if (m_degree!= m_knot_vector.size() - m_control_points.size() -1 )
    {
        cout << "The parameters are not compatibles. Please recheck " << endl;
        return result;
    }
    for (unsigned int i=0;i<m_control_points.size();i++)
    {
        result += m_basis_function[i] * m_control_points[i];
    }
    return result;
}

Bsplines Bsplines::DerivativeBsplines()
{
    if (m_degree >=1)
    {
        Bsplines dB(m_degree-1);
        std::vector<double> dB_control_points(m_control_points.size()-1);
        std::vector<double> dB_knot_vector (m_knot_vector.size()-2);


        for (unsigned int i=0 ; i<dB_control_points.size() ; ++i)
        {
            if(m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]==0.0)
            {
              cout << "Knot no differenciable : result in the zero function\n" ;
              dB_control_points[i] = 0.0 ;
            }
            else
            {
              dB_control_points[i] = ((m_control_points[i+1] - m_control_points[i])*double(m_degree) )/ (m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]);
            }
        }

        for (unsigned int i=0 ; i<dB_knot_vector.size() ; ++i)
          dB_knot_vector[i] = m_knot_vector[i+1] ;


        dB.SetKnotVector(dB_knot_vector);
        dB.SetControlPoints(dB_control_points);
        return dB;
        }
    else
        {
            std::cout << "the function cannot be derivated " << std::endl;
            return Bsplines(m_degree);
        }
}

void Bsplines::SetDegree(int degree)
{
    m_degree = degree;
}

void Bsplines::SetControlPoints(std::vector<double> &control_points)
{
    if (control_points.size()>=2)
    {
        m_control_points = control_points;
    }
    else
    {
        std::cout << "You must give at least 2 control points" << std::endl;
    }
}

void Bsplines::SetKnotVector(std::vector<double> &knot_vector)
{
    m_knot_vector = knot_vector;
}

int Bsplines::GetDegree() const
{
    return m_degree;
}

std::vector<double> Bsplines::GetControlPoints() const
{
    return m_control_points;
}

std::vector<double> Bsplines::GetKnotVector() const
{
    return m_knot_vector;
}

void Bsplines::PrintKnotVector() const
{
    std::cout << "Knot Vector: "<< std::endl;
    for (unsigned int i = 0;i<m_knot_vector.size();i++)
    {
        std::cout << m_knot_vector[i] << " , ";
    }
    std::cout <<" " <<std::endl;
}

void Bsplines::PrintControlPoints() const
{
    std::cout << "Control Points : "<< std::endl;
    for (unsigned int i = 0;i<m_control_points.size();i++)
    {
        std::cout << m_control_points[i]  << std::endl;
    }
}

void Bsplines::PrintDegree() const
{
    std::cout << "Degree: " << m_degree << std::endl;
}


// Class ZBplines heritage of class Bsplines
// create a foot trajectory of Z in function of the time t


BSplinesFoot::BSplinesFoot(double FT, double FP, vector<double> ToMP, vector<double> MP):Bsplines(5)
{
  SetParameters(FT, 0.0 , FP, ToMP, MP);
}

BSplinesFoot::~BSplinesFoot()
{

}

double BSplinesFoot::Compute(double t)
{
  double time = t/m_FT ;
  if (time <= 0.0)
    return m_IP ;
  if (time >= 1.0)
    return m_FP ;
  return ComputeBsplines(time);
}

double BSplinesFoot::ComputeDerivative(double t)
{
  if (m_degree >=1){
    double time = t/m_FT ;
    if (time <= 0.0)
      return 0.0 ;
    if (time >= 1.0)
      return 0.0 ;
    return DerivativeBsplines().ComputeBsplines(time);
  }
  else
  {
    cout << "ERROR" << endl;
    return -1;
  }
}

double BSplinesFoot::ComputeSecDerivative(double t)
{
  if (m_degree >=2)
  {
    double time = t/m_FT ;
    if (time <= 0.0)
      return 0.0 ;
    if (time >= 1.0)
      return 0.0 ;
    return DerivativeBsplines().DerivativeBsplines().ComputeBsplines(time);
  }
  else
  {
    cout << "ERROR" << endl;
    return -1;
  }
}

void  BSplinesFoot::SetParameters(double FT,
                                  double IP,
                                  double FP,
                                  vector<double>ToMP,
                                  vector<double> MP)
{
  // verify that each middle point has a reaching time parameter
  assert(ToMP.size()==MP.size());

  // save the parameters
  m_FT = FT ;
  m_IP = IP ;
  m_FP = FP ;
  m_ToMP = ToMP ;
  m_MP = MP ;


  // initialize some variables
  std::vector<double> knot;
  std::vector<double> control_points;
  knot.clear();
  control_points.clear();

  double alpha = 0.0;
  // generation of the knot vector
  switch (ToMP.size())
  {
    case 0 :
      // set the first three knots to 0.0
      // the next one to 50% of the final time
      // and the last three to the final time
      for (unsigned int i=0;i<=m_degree;i++)
        {knot.push_back(0.0);}

      for (unsigned int i=0 ; i<(m_degree-1) ; ++i)
        {knot.push_back ((double)(i+1) / (m_degree)) ;}

      for (unsigned int i =0;i<=m_degree;i++)
        {knot.push_back(1);}

      // Set the first three control point
      // to the initial pos and the last three
      // to the final pos
      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_IP);

      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_FP);
    break ;

    case 1 :
      for (unsigned int i=0 ; i<=m_degree ; ++i)
        {knot.push_back(0.0);}

      for (unsigned int i=1 ; i <=3 ; ++i)
        {knot.push_back((double)i/3.0*m_ToMP[0]/m_FT);}

      for (unsigned int i=0 ; i <3 ; ++i)
        {knot.push_back((m_ToMP[0]+(double)i/3*(m_FT-m_ToMP[0]))/m_FT);}

      for (unsigned int i =0 ; i<=m_degree ; ++i)
        {knot.push_back(1);}

      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_IP);

      control_points.push_back(m_MP[0]);
      control_points.push_back(m_MP[0]);

      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_FP);

    break ;

    case 2 :
      for (unsigned int i=0 ; i<=m_degree ; ++i)
        {knot.push_back(0.0);}

      for (unsigned int i=1 ; i <=2 ; ++i)
        {knot.push_back((double)i/2.0*m_ToMP[0]/m_FT);}

      for (unsigned int i=0 ; i <2 ; ++i)
        {knot.push_back( (m_ToMP[0]+m_ToMP[1])*0.5 /m_FT);}

      for (unsigned int i=0 ; i <2 ; ++i)
        {knot.push_back((m_ToMP[1]+(double)i/2*(m_FT-m_ToMP[1]))/m_FT);}

      for (unsigned int i=0 ; i<=m_degree ; ++i)
        {knot.push_back(1);}

      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_IP);

      control_points.push_back(m_MP[0]);
      control_points.push_back(m_MP[1]);

      for(unsigned int i=0 ; i<m_degree ; ++i)
        control_points.push_back(m_FP);

    break ;
  }// end switch case

  SetKnotVector(knot);
  SetControlPoints(control_points);

  return ;
}


void BSplinesFoot::GetParameters(double &FT,
				 double &IP,
				 double &FP,
				 vector<double> &ToMP,
				 vector<double> &MP)
{
    FT = m_FT ;
    FP = m_FP ;
    IP = m_IP ;
    ToMP = m_ToMP ;
    MP = m_MP ;
}
