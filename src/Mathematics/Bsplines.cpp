#include <iostream>
#include <vector>

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
    m_degree = (m_knot_vector.size()-1) - (m_control_points.size()-1) -1;
    if (m_degree < 0)
    {
        cout << " Attention !! degree is smaller than 0 " << endl;
    }
}

void Bsplines::GenerateKnotVector(std::string method)
{
    /*Calculer set of parameters*/
    int i,j;
    vector<double> set_of_pam;

        if (method == "centripetal")
        {
            //cout << "centripetal" << endl;
            set_of_pam.clear();
            set_of_pam.reserve(m_control_points.size());
            cout << m_control_points.size() << endl;
            double L = 0.0;
            double D = 0.0;
            for (i=0;i<m_control_points.size()-1;i++)
            {
                L += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
            }
            for (i=0;i<m_control_points.size();i++)
            {
                if (i == 0)
                {
                    set_of_pam.push_back(0.0);
                }
                else if (i == m_control_points.size()-1)
                {
                    set_of_pam.push_back(1.0);
                }
                else
                {
                    D += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
                    set_of_pam.push_back(D/L);
                }
            }

            m_knot_vector.clear();
            double U = 0.0;
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(0.0);
            }

            if (m_control_points.size()-1>=m_degree)
            {
                for (j=1;j<=m_control_points.size()-1-m_degree;j++)
                {
                    i=j;
                    U=0.0;
                    while (i<=m_degree-1+j)
                    {
                        U +=set_of_pam[i];
                        i++;
                    }
                    m_knot_vector.push_back(U/double(m_degree));
                }
            }

            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(1.0);
            }
            if (m_knot_vector.size() - 1 != m_control_points.size() + m_degree)
            {
                cout << "Knot vector cant be created. m_control_points.size()-1>=m_degree "<< endl;
                m_knot_vector.clear();
            }

        }

        else if (method =="universal")
        {
            m_knot_vector.clear();
            //cout << "universal" << endl;
            double U=0;
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(0.0);
            }
            for (i=1;i<=m_control_points.size()-1-m_degree;i++)
            {
                U = double(i)/(m_control_points.size()-1-m_degree+1);
                m_knot_vector.push_back(U);
            }
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(1.0);
            }

        }
}

double *Bsplines::ComputeBasisFunction(double t)
{
    double **m_basis_function;
    m_basis_function = new double* [m_degree+1];
    int i,j,n;

    if (m_degree!= m_knot_vector.size() - m_control_points.size() -1 )
    {
        cout << "The parameters are not compatibles. Please recheck " << endl;
        return NULL;
    }
        for(j=0;j <= m_degree;j++)
        {
            n = m_knot_vector.size() - 1 -j -1;
            m_basis_function[j] = new double[n+1];
            //cout << "order  " ; cout << j << endl;
            //cout << "n control point  " ; cout << n << endl;

            for(i=0;i <= n;i++)
            {
               if (j == 0 && m_knot_vector[i]<= t && t< m_knot_vector[i+1])
                {
                    m_basis_function[j][i] = 1.0;
                    //cout << i << " "<< j << " "<<m_basis_function[j][i] << endl;
                }

                else if (j == 0)
                {
                    m_basis_function[j][i] = 0.0;
                    //cout << i << " "<< j << " "<<m_basis_function[j][i] << endl;
                }
                else if (j != 0)
                {
                    if ( m_knot_vector[i] != m_knot_vector[i+j] && m_knot_vector[i+j+1] != m_knot_vector[i+1] )
                        m_basis_function[j][i]= m_basis_function[j-1][i]*((t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i]))
                                                + m_basis_function[j-1][i+1]*((m_knot_vector[i+j+1] - t)/ (m_knot_vector[i+j+1]-m_knot_vector[i+1] ));

                    else if ( m_knot_vector[i] == m_knot_vector[i+j] && m_knot_vector[i+j+1] == m_knot_vector[i+1] )
                        m_basis_function[j][i] = 0.0;

                    else if (m_knot_vector[i] == m_knot_vector[i+j])
                        m_basis_function[j][i]= m_basis_function[j-1][i+1]*((m_knot_vector[i+j+1] - t)/ (m_knot_vector[i+j+1]-m_knot_vector[i+1] ));

                    else if (m_knot_vector[i+j+1] == m_knot_vector[i+1])
                        m_basis_function[j][i]= m_basis_function[j-1][i]*((t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i]));

                    //cout << i << " "<< j << " "<<m_basis_function[j][i] << endl;
                }
            }
        }
        for(j=0;j < m_degree;j++)
        {
            delete[] m_basis_function[j];
        }

return m_basis_function[m_degree];
}

Point Bsplines::ComputeBsplines(double t)
{
    double *m_basis_function = ComputeBasisFunction(t);

    Point C = {0.0,0.0};
    if (m_degree!= m_knot_vector.size() - m_control_points.size() -1 )
    {
        cout << "The parameters are not compatibles. Please recheck " << endl;
        return C;
    }
    if (m_degree == ((m_knot_vector.size()-1) - (m_control_points.size()-1)- 1) )
        {
            for (int i=0;i<m_control_points.size();i++)
            {
                C.x += m_basis_function[i] * m_control_points[i].x;
                C.y += m_basis_function[i] * m_control_points[i].y;
            }
        }
return C;
}

Bsplines Bsplines::DerivativeBsplines()
{
    std::vector<Point> Q;
    Q.clear();
    Q.reserve(m_control_points.size()-1);

    Point T;
    if (m_degree >=1)
    {
        for (int i=0;i<m_control_points.size()-1;i++)
        {
            T.x = ((m_control_points[i+1].x - m_control_points[i].x)*double(m_degree) )/ (m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]);

            T.y = ((m_control_points[i+1].y - m_control_points[i].y)*double(m_degree) )/ (m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]);
            Q.push_back(T);
        }
        Bsplines B(m_degree-1);
        B.SetControlPoints(Q);
        std::vector<double> new_knot_vector(m_knot_vector.begin()+1,m_knot_vector.end()-1);
        B.SetKnotVector(new_knot_vector);
        //cout <<"derivative function "<<endl;
        //B.PrintKnotVector();
        //B.PrintControlPoints();
        //B.PrintDegree();
        return B;
        }
    else
        {
            std::cout << "the function cannot be derivative " << std::endl;
            return Bsplines(m_degree);
        }
}

void Bsplines::SetDegree(int degree)
{
    m_degree = degree;
}

void Bsplines::SetControlPoints(std::vector<Point> &control_points)
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

std::vector<Point> Bsplines::GetControlPoints() const
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
    for (int i = 0;i<m_knot_vector.size();i++)
    {
        std::cout << m_knot_vector[i] << " , ";
    }
    std::cout <<" " <<std::endl;
}

void Bsplines::PrintControlPoints() const
{
    std::cout << "Control Points : "<< std::endl;
    for (int i = 0;i<m_control_points.size();i++)
    {
        std::cout << m_control_points[i].x << " , " << m_control_points[i].y << std::endl;
    }
}

void Bsplines::PrintDegree() const
{
    std::cout << "Degree: " << m_degree << std::endl;
}


// Class ZBplines heritage of class Bsplines
// create a foot trajectory of Z on function the time t


ZBsplines::ZBsplines(double FT, double FP, double ToMP, double MP):Bsplines(4)
{
    SetParameters( FT, FP, ToMP, MP);
}

ZBsplines::~ZBsplines()
{

}

double ZBsplines::ZComputePosition(double t)
{
    if (t<=m_FT)
        return ComputeBsplines(t).y;
    else
        return m_FP;
}

double ZBsplines::ZComputeVelocity(double t)
{
    if (m_degree >=1){
        if (t<m_FT)
            return DerivativeBsplines().ComputeBsplines(t).y;
        else
            return DerivativeBsplines().ComputeBsplines(m_FT - t).y;
    }
    else
    {
        cout << "ERROR" << endl;
        return -1;
    }
}

double ZBsplines::ZComputeAcc(double t)
{
    if (m_degree >=2){
        if (t<m_FT)
            return DerivativeBsplines().DerivativeBsplines().ComputeBsplines(t).y;
        else
            return DerivativeBsplines().DerivativeBsplines().ComputeBsplines(m_FT - t).y;
    }
    else
    {
        cout << "ERROR" << endl;
        return -1;
    }
}

void  ZBsplines::SetParameters(double FT, double FP, double ToMP, double MP)
{
    ZGenerateKnotVector(FT,ToMP);
    ZGenerateControlPoints(0.0, FT, FP, ToMP, MP);
}

void  ZBsplines::SetParametersWithInitPos(double IP, double FT, double FP, double ToMP, double MP)
{
    ZGenerateKnotVector(FT,ToMP);
    ZGenerateControlPoints(IP, FT, FP, ToMP, MP);
}

void ZBsplines::GetParametersWithInitPosInitSpeed(double &FT,
						  double &FP,
						  double &InitPos,
						  double &InitSpeed)
{
    FT = m_FT;
    FP = m_FP;
    InitPos = ZComputePosition(0.0);
    InitSpeed = ZComputeVelocity(0.0);
}

void ZBsplines::ZGenerateKnotVector(double FT, double ToMP)
{
    std::vector<double> knot;
    knot.clear();
    for (int i=0;i<=m_degree;i++)
    {
        knot.push_back(0.0);
    }

    knot.push_back(0.6*ToMP);
    knot.push_back(ToMP);
    knot.push_back(1.3*ToMP);

    for (int i =0;i<=m_degree;i++)
    {
        knot.push_back(FT);
    }

    SetKnotVector(knot);
}

void ZBsplines::ZGenerateControlPoints(double IP, double FT, double FP, double ToMP, double MP)
{
    m_FT = FT;
    m_FP = FP;
    m_ToMP = ToMP;
    m_MP = MP;
    m_IP = IP;
    std::vector<Point> control_points;
    control_points.clear();
    std::ofstream myfile1;
    myfile1.open("control_point.txt");

    Point A = {0.0,IP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT*0.05,IP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT*0.1,IP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.85*m_ToMP,m_MP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {1.15*m_ToMP,m_MP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.90*m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.95*m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    myfile1.close();

    SetControlPoints(control_points);
}

double ZBsplines::GetMP()
{
    return m_MP;
}


double ZBsplines::GetFT()
{
    return m_FT;
}
