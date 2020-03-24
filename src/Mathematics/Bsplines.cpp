#include <assert.h>
#include <iostream>
#include <vector>

#include <Debug.hh>
#include <Mathematics/Bsplines.hh>

using namespace ::std;
using namespace ::PatternGeneratorJRL;

Bsplines::Bsplines(long int degree) {
  m_degree = degree;
  m_control_points.clear();
  m_knot.clear();
}

Bsplines::~Bsplines() {}

void Bsplines::GenerateDegree() {
  long int degree = (m_knot.size() - 1) - (m_control_points.size() - 1) - 1;
  if (degree < 0) {
    std::cerr << __FILE__ << ":" << __FUNCTION__ << "(#" << __LINE__ << "):"
              << " Carefull !! degree is smaller than 0 " << endl;
  }
  m_degree = (unsigned)degree;
}

int Bsplines::ComputeBasisFunctions(double t) {
  m_basis_functions.resize(m_knot.size() - m_degree - 1);
  // number of basis function for one specific order
  long int n;
  // temporary variables usefull t test divisions by 0
  double tmp1(0.0), tmp2(0.0), tmp3(0.0), tmp4(0.0);
  // handle limit conditions
  bool is_in;

  // compute the basis function of t : Nij(t)
  for (unsigned int j = 0; j <= m_degree; j++) {
    n = m_knot.size() - j - 2;
    m_basis_functions[j].resize(n + 1, 0.0);

    for (unsigned int i = 0; i <= n; i++) {
      if (j == 0) {
        if (t == 1)
          is_in = (m_knot[i] < t) && (t <= m_knot[i + 1]);
        else
          is_in = (m_knot[i] <= t) && (t < m_knot[i + 1]);

        if (is_in)
          m_basis_functions[j][i] = 1.0;
        else
          m_basis_functions[j][i] = 0.0;
      } else {
        if (m_knot[i] == m_knot[i + j])
          tmp1 = 0.0;
        else
          tmp1 = (t - m_knot[i]) / (m_knot[i + j] - m_knot[i]) *
                 m_basis_functions[j - 1][i];

        if (m_knot[i + j + 1] == m_knot[i + 1])
          tmp2 = 0.0;
        else
          tmp2 = (m_knot[i + j + 1] - t) / (m_knot[i + j + 1] - m_knot[i + 1]) *
                 m_basis_functions[j - 1][i + 1];

        m_basis_functions[j][i] = tmp1 + tmp2;
      }
    }
  }

  // compute their time derivative for the m_degree p=m_m_degree : d Nip(t) /dt
  m_basis_functions_derivative.resize(m_basis_functions[m_degree].size());
  for (unsigned int i = 0; i < m_basis_functions_derivative.size(); ++i) {
    tmp1 = 0.0;
    tmp2 = 0.0;
    if (m_knot[i + m_degree] == m_knot[i])
      tmp1 = 0.0;
    else
      tmp1 = (double)m_degree / (double)((m_knot[i + m_degree] - m_knot[i])) *
             m_basis_functions[m_degree - 1][i];

    if (m_knot[i + m_degree + 1] == m_knot[i + 1])
      tmp2 = 0.0;
    else
      tmp2 = (double)m_degree /
             (double)((m_knot[i + m_degree + 1] - m_knot[i + 1])) *
             m_basis_functions[m_degree - 1][i + 1];
    m_basis_functions_derivative[i] = tmp1 - tmp2;
  }

  // compute their time second derivative for the degree
  // p=m_degree : dd Nip(t) /dt
  m_basis_functions_sec_derivative.resize(m_basis_functions[m_degree].size());
  double factor = (double)(m_degree * (m_degree - 1));
  double den1(0.0), den2(0.0), den3(0.0), den4(0.0);
  for (unsigned long int i = 0; i < m_basis_functions_derivative.size(); ++i) {
    tmp1 = 0.0;
    tmp2 = 0.0;
    tmp3 = 0.0;
    tmp4 = 0.0;
    den1 = 0.0;
    den2 = 0.0;
    den3 = 0.0;
    den4 = 0.0;

    den1 = (m_knot[i + m_degree] - m_knot[i]) *
           (m_knot[i + m_degree - 1] - m_knot[i]);
    den2 = (m_knot[i + m_degree] - m_knot[i]) *
           (m_knot[i + m_degree] - m_knot[i + 1]);
    den3 = (m_knot[i + m_degree + 1] - m_knot[i + 1]) *
           (m_knot[i + m_degree] - m_knot[i + 1]);
    den4 = (m_knot[i + m_degree + 1] - m_knot[i + 1]) *
           (m_knot[i + m_degree + 1] - m_knot[i + 2]);

    if (den1 == 0)
      tmp1 = 0.0;
    else
      tmp1 = factor / den1;

    if (den2 == 0)
      tmp2 = 0.0;
    else
      tmp2 = -factor / den2;

    if (den3 == 0)
      tmp3 = 0.0;
    else
      tmp3 = -factor / den3;

    if (den4 == 0)
      tmp4 = 0.0;
    else
      tmp4 = factor / den4;

    m_basis_functions_sec_derivative[i] =
        tmp1 * m_basis_functions[m_degree - 2][i] +
        tmp2 * m_basis_functions[m_degree - 2][i + 1] +
        tmp3 * m_basis_functions[m_degree - 2][i + 1] +
        tmp4 * m_basis_functions[m_degree - 2][i + 2];
  }
  return 1;
}

int Bsplines::ComputeBasisFunctionsRecursively(double t,
                                               std::deque<double> &m_knot,
                                               unsigned int m_degree) {
  vector<double> basis_functions(m_knot.size() - m_degree - 1);
  for (unsigned int i = 0; i < basis_functions.size(); ++i)
    basis_functions[i] = Nij_t(i, m_degree, t, m_knot);

  m_basis_functions[m_degree] = basis_functions;
  return 0;
}

double Bsplines::Nij_t(int i, int j, double t, deque<double> &m_knot) {
  double Nij_t = 0.0;
  // i is the time interval, j is the order
  if ((j == 0 && m_knot[i] <= t && t < m_knot[i + 1] &&
       m_knot[i] < m_knot[i + 1])) {
    Nij_t = 1.0;
  } else if (j == 0) {
    Nij_t = 0.0;
  } else {
    double tmp1(0.0), tmp2(0.0);

    if (m_knot[i] == m_knot[i + j])
      tmp1 = 0.0;
    else
      tmp1 = (t - m_knot[i]) / (m_knot[i + j] - m_knot[i]) *
             Bsplines::Nij_t(i, j - 1, t, m_knot);

    if (m_knot[i + j + 1] == m_knot[i + 1])
      tmp2 = 0.0;
    else
      tmp2 = (m_knot[i + j + 1] - t) / (m_knot[i + j + 1] - m_knot[i + 1]) *
             Bsplines::Nij_t(i + 1, j - 1, t, m_knot);

    Nij_t = tmp1 + tmp2;
  }
  return Nij_t;
}

double Bsplines::ComputeBsplines(double t) {
  ComputeBasisFunctions(t);
  double result = 0.0;
  if (m_degree !=
      (long int)m_knot.size() - (long int)m_control_points.size() - 1) {
    cerr << "The parameters are not compatibles. Please recheck " << endl;
    return result;
  }
  for (unsigned int i = 0; i < m_control_points.size(); i++) {
    result += m_basis_functions[m_degree][i] * m_control_points[i];
  }
  return result;
}

Bsplines Bsplines::DerivativeBsplines() {
  if (m_degree >= 1) {
    Bsplines dB(m_degree - 1);
    std::vector<double> dB_control_points(m_control_points.size() - 1);
    std::deque<double> dB_knot_vector(m_knot.size() - 2);

    for (unsigned int i = 0; i < dB_control_points.size(); ++i) {
      if (m_knot[i + m_degree + 1] - m_knot[i + 1] == 0.0) {
        cerr << "Knot no differenciable : result in the zero function\n";
        dB_control_points[i] = 0.0;
      } else {
        dB_control_points[i] =
            ((m_control_points[i + 1] - m_control_points[i]) *
             double(m_degree)) /
            (m_knot[i + m_degree + 1] - m_knot[i + 1]);
      }
    }

    for (unsigned int i = 0; i < dB_knot_vector.size(); ++i)
      dB_knot_vector[i] = m_knot[i + 1];

    dB.SetKnotVector(dB_knot_vector);
    dB.SetControlPoints(dB_control_points);
    return dB;
  } else {
    std::cerr << "the function cannot be derivated " << std::endl;
    return Bsplines(m_degree);
  }
}

void Bsplines::SetDegree(long int degree) { m_degree = degree; }

void Bsplines::SetControlPoints(std::vector<double> &control_points) {
  if (control_points.size() >= 2) {
    m_control_points = control_points;
  } else {
    std::cerr << "You must give at least 2 control points" << std::endl;
  }
}

void Bsplines::SetKnotVector(std::deque<double> &knot_vector) {
  m_knot = knot_vector;
}

long int Bsplines::GetDegree() const { return m_degree; }

std::vector<double> Bsplines::GetControlPoints() const {
  return m_control_points;
}

std::deque<double> Bsplines::GetKnotVector() const { return m_knot; }

void Bsplines::PrintKnotVector() const {
  ODEBUG("Knot Vector (" << m_knot.size() << ") : ");
  for (unsigned int i = 0; i < m_knot.size(); i++) {
    ODEBUG(m_knot[i] << " , ");
  }
  ODEBUG(" ");
}

void Bsplines::PrintControlPoints() const {
  ODEBUG("Control Points (" << m_control_points.size() << ") : ");
  for (unsigned int i = 0; i < m_control_points.size(); i++) {
    std::cout << m_control_points[i] << " , ";
  }
  cout << std::endl;
}

void Bsplines::PrintDegree() const {
  std::cout << "Degree: " << m_degree << std::endl;
}

// Class ZBplines heritage of class Bsplines
// create a foot trajectory of Z in function of the time t

BSplinesFoot::BSplinesFoot(double FT, double IP, double FP, vector<double> ToMP,
                           vector<double> MP, double IS, double IA, double FS,
                           double FA)
    : Bsplines(5) {
  SetParameters(FT, IP, FP, ToMP, MP, IS, IA, FS, FA);
}

BSplinesFoot::~BSplinesFoot() {}

int BSplinesFoot::Compute(double t, double &x, double &dx, double &ddx) {
  double time = t / m_FT;
  if (time <= 0.0)
    time = 0.0;
  if (time >= 1.0)
    time = 1.0;

  ComputeBasisFunctions(time);
  x = 0.0;
  dx = 0.0;
  ddx = 0.0;
  for (unsigned int i = 0; i < m_control_points.size(); i++) {
    x += m_basis_functions[m_degree][i] * m_control_points[i];
    dx += m_basis_functions_derivative[i] * m_control_points[i];
    ddx += m_basis_functions_sec_derivative[i] * m_control_points[i];
  }
  return 1;
}

void BSplinesFoot::SetParameters(double FT, double IP, double FP,
                                 std::vector<double> ToMP,
                                 std::vector<double> MP, double IS, double IA,
                                 double FS, double FA) {
  // verify that each middle point has a reaching time parameter
  assert(ToMP.size() == MP.size());

  // save the parameters
  m_FT = FT;

  m_IP = IP;
  m_IS = IS;
  m_IA = IA;

  m_FP = FP;
  m_FS = FS;
  m_FA = FA;

  m_ToMP = ToMP;
  m_MP = MP;

  // initialize some variables
  std::deque<double> knot;
  knot.resize(ToMP.size() + 2 * (m_degree + 1));
  m_control_points.resize(ToMP.size() + m_degree + 1);

  // generation of the knot vector
  switch (ToMP.size()) {
  case 0:
    // set the first three knots to 0.0
    // the next one to 50% of the final time
    // and the last three to the final time
    for (unsigned int i = 0; i <= m_degree; i++) {
      knot[i] = 0.0;
    }

    for (long int i = knot.size() - (m_degree + 1); i <= (long int)knot.size();
         i++) {
      knot[i] = 1.0;
    }

    SetKnotVector(knot);
    ComputeControlPointFrom2DataPoint();
    break;

  case 1:
    for (unsigned int i = 0; i <= m_degree; i++) {
      knot[i] = 0.0;
    }

    knot[m_degree + 1] = m_ToMP[0] / m_FT;

    for (long int i = knot.size() - (m_degree + 1); i <= (long int)knot.size();
         i++) {
      knot[i] = 1.0;
    }

    SetKnotVector(knot);
    ComputeControlPointFrom3DataPoint();

    break;

  case 2:
    for (unsigned int i = 0; i <= m_degree; i++) {
      knot[i] = 0.0;
    }

    knot[m_degree + 1] = m_ToMP[0] / m_FT;
    knot[m_degree + 2] = m_ToMP[1] / m_FT;

    for (long int i = knot.size() - (m_degree + 1); i <= (long int)knot.size();
         i++) {
      knot[i] = 1.0;
    }

    SetKnotVector(knot);
    ComputeControlPointFrom4DataPoint();

    break;
  } // end switch case

  return;
}

void BSplinesFoot::ComputeControlPointFrom2DataPoint(void) {
  ComputeBasisFunctions(0);
  vector<double> dNi5T0 = m_basis_functions_derivative;
  vector<double> ddNi5T0 = m_basis_functions_sec_derivative;

  ComputeBasisFunctions(1);
  vector<double> dNi5T = m_basis_functions_derivative;
  vector<double> ddNi5T = m_basis_functions_sec_derivative;

  double IP = m_IP;
  double IS = m_IS;
  double IA = m_IA;
  double FP = m_FP;
  double FS = m_FS;
  double FA = m_FA;

  double dN0T0 = dNi5T0[0];
  double dN1T0 = dNi5T0[1];
  double dN2T0 = dNi5T0[2];
  double dN3T0 = dNi5T0[3];
  double dN4T0 = dNi5T0[4];

  double dN1T = dNi5T[1];
  double dN2T = dNi5T[2];
  double dN3T = dNi5T[3];
  double dN4T = dNi5T[4];
  double dN5T = dNi5T[5];

  double ddN0T0 = ddNi5T0[0];
  double ddN1T0 = ddNi5T0[1];
  double ddN2T0 = ddNi5T0[2];
  double ddN3T0 = ddNi5T0[3];
  double ddN4T0 = ddNi5T0[4];

  double ddN1T = ddNi5T[1];
  double ddN2T = ddNi5T[2];
  double ddN3T = ddNi5T[3];
  double ddN4T = ddNi5T[4];
  double ddN5T = ddNi5T[5];

  m_control_points[0] = IP;
  m_control_points[1] =
      1.0 /
      (dN2T * dN3T0 * ddN1T * ddN4T0 - dN3T * dN4T0 * ddN2T * ddN1T0 +
       dN1T * ddN4T0 * ddN3T * dN2T0 - dN2T * ddN3T0 * dN4T0 * ddN1T +
       ddN4T * dN1T * ddN2T0 * dN3T0 + dN1T0 * dN3T * ddN4T0 * ddN2T +
       dN4T * dN3T0 * ddN2T * ddN1T0 - dN1T * ddN2T0 * dN4T0 * ddN3T +
       dN2T * dN4T0 * ddN3T * ddN1T0 - dN1T * dN3T0 * ddN4T0 * ddN2T +
       ddN2T0 * dN3T * dN4T0 * ddN1T - ddN4T * ddN2T0 * dN1T0 * dN3T +
       dN2T * ddN4T * ddN3T0 * dN1T0 - dN3T * ddN1T * ddN4T0 * dN2T0 -
       dN4T * ddN2T0 * dN3T0 * ddN1T - dN2T * dN1T0 * ddN4T0 * ddN3T +
       dN4T * ddN2T0 * dN1T0 * ddN3T + ddN3T0 * dN4T * ddN1T * dN2T0 -
       dN4T * ddN3T * dN2T0 * ddN1T0 + ddN3T0 * dN1T * dN4T0 * ddN2T -
       dN2T * ddN4T * dN3T0 * ddN1T0 - ddN3T0 * dN4T * dN1T0 * ddN2T +
       ddN4T * dN3T * dN2T0 * ddN1T0 - ddN4T * ddN3T0 * dN1T * dN2T0) *
      (FS * ddN4T * ddN2T0 * dN3T0 - IA * dN3T * dN4T0 * ddN2T -
       ddN3T0 * dN4T * ddN2T * IS - dN2T * ddN0T0 * IP * dN4T0 * ddN3T -
       dN4T * ddN2T0 * dN3T0 * FA + ddN3T0 * dN4T * IP * dN0T0 * ddN2T -
       ddN0T0 * ddN4T * IP * dN3T * dN2T0 - dN3T * ddN4T0 * FA * dN2T0 +
       ddN0T0 * IP * dN3T * dN4T0 * ddN2T + IA * dN2T * dN4T0 * ddN3T +
       ddN4T * ddN3T0 * FP * dN5T * dN2T0 + ddN5T * dN2T * ddN3T0 * dN4T0 * FP +
       ddN0T0 * dN4T * IP * ddN3T * dN2T0 + dN3T0 * ddN4T0 * FP * dN5T * ddN2T -
       ddN3T0 * dN4T0 * FP * dN5T * ddN2T - dN4T * ddN2T0 * IP * ddN3T * dN0T0 -
       ddN4T * ddN2T0 * dN3T0 * FP * dN5T - IA * dN4T * ddN3T * dN2T0 +
       ddN5T * dN3T * ddN4T0 * FP * dN2T0 - ddN5T * ddN2T0 * dN3T * dN4T0 * FP -
       dN2T * ddN4T * ddN3T0 * IP * dN0T0 + IA * ddN4T * dN3T * dN2T0 -
       ddN5T * dN2T * dN3T0 * ddN4T0 * FP - ddN0T0 * dN4T * IP * dN3T0 * ddN2T +
       dN3T * ddN4T0 * ddN2T * IS + ddN5T * dN4T * ddN2T0 * dN3T0 * FP +
       IA * dN4T * dN3T0 * ddN2T - FS * ddN4T * ddN3T0 * dN2T0 +
       FS * ddN4T0 * ddN3T * dN2T0 - IP * dN3T * ddN4T0 * dN0T0 * ddN2T +
       dN4T * ddN2T0 * ddN3T * IS + dN2T * IP * ddN4T0 * ddN3T * dN0T0 +
       FS * ddN3T0 * dN4T0 * ddN2T - dN2T * ddN3T0 * dN4T0 * FA +
       ddN3T0 * dN4T * FA * dN2T0 + ddN2T0 * dN4T0 * FP * dN5T * ddN3T -
       FS * ddN2T0 * dN4T0 * ddN3T + dN2T * ddN4T * ddN3T0 * IS -
       dN2T * ddN4T0 * ddN3T * IS - ddN4T0 * FP * dN5T * ddN3T * dN2T0 +
       dN2T * ddN0T0 * ddN4T * IP * dN3T0 - IA * dN2T * ddN4T * dN3T0 +
       ddN2T0 * dN3T * dN4T0 * FA - FS * dN3T0 * ddN4T0 * ddN2T -
       ddN5T * ddN3T0 * dN4T * FP * dN2T0 + dN2T * dN3T0 * ddN4T0 * FA -
       ddN4T * ddN2T0 * dN3T * IS + ddN4T * ddN2T0 * IP * dN3T * dN0T0);
  m_control_points[2] =
      -1.0 /
      (dN2T * dN3T0 * ddN1T * ddN4T0 - dN3T * dN4T0 * ddN2T * ddN1T0 +
       dN1T * ddN4T0 * ddN3T * dN2T0 - dN2T * ddN3T0 * dN4T0 * ddN1T +
       ddN4T * dN1T * ddN2T0 * dN3T0 + dN1T0 * dN3T * ddN4T0 * ddN2T +
       dN4T * dN3T0 * ddN2T * ddN1T0 - dN1T * ddN2T0 * dN4T0 * ddN3T +
       dN2T * dN4T0 * ddN3T * ddN1T0 - dN1T * dN3T0 * ddN4T0 * ddN2T +
       ddN2T0 * dN3T * dN4T0 * ddN1T - ddN4T * ddN2T0 * dN1T0 * dN3T +
       dN2T * ddN4T * ddN3T0 * dN1T0 - dN3T * ddN1T * ddN4T0 * dN2T0 -
       dN4T * ddN2T0 * dN3T0 * ddN1T - dN2T * dN1T0 * ddN4T0 * ddN3T +
       dN4T * ddN2T0 * dN1T0 * ddN3T + ddN3T0 * dN4T * ddN1T * dN2T0 -
       dN4T * ddN3T * dN2T0 * ddN1T0 + ddN3T0 * dN1T * dN4T0 * ddN2T -
       dN2T * ddN4T * dN3T0 * ddN1T0 - ddN3T0 * dN4T * dN1T0 * ddN2T +
       ddN4T * dN3T * dN2T0 * ddN1T0 - ddN4T * ddN3T0 * dN1T * dN2T0) *
      (ddN0T0 * dN4T * dN1T0 * IP * ddN3T + ddN5T * ddN3T0 * dN1T * dN4T0 * FP -
       dN1T * ddN4T0 * ddN3T * IS + dN1T * IP * ddN4T0 * ddN3T * dN0T0 +
       dN4T0 * FP * dN5T * ddN3T * ddN1T0 - ddN5T * ddN3T0 * dN4T * dN1T0 * FP +
       dN1T * dN3T0 * ddN4T0 * FA - ddN4T * dN3T0 * FP * dN5T * ddN1T0 -
       FS * dN4T0 * ddN3T * ddN1T0 - dN1T0 * ddN4T0 * FP * dN5T * ddN3T +
       ddN3T0 * dN4T * IP * ddN1T * dN0T0 - ddN0T0 * dN4T * IP * dN3T0 * ddN1T -
       IA * ddN4T * dN1T * dN3T0 + IA * dN4T * dN3T0 * ddN1T +
       IA * dN1T * dN4T0 * ddN3T - IP * dN3T * ddN1T * ddN4T0 * dN0T0 -
       ddN4T * ddN3T0 * dN1T * IP * dN0T0 + FS * ddN3T0 * dN4T0 * ddN1T -
       IA * dN4T * dN1T0 * ddN3T + ddN4T * ddN3T0 * dN1T0 * FP * dN5T -
       FS * dN3T0 * ddN1T * ddN4T0 + ddN4T * IP * dN3T * dN0T0 * ddN1T0 -
       dN1T0 * dN3T * ddN4T0 * FA + FS * ddN4T * dN3T0 * ddN1T0 -
       ddN3T0 * dN4T * ddN1T * IS - ddN0T0 * dN1T * IP * dN4T0 * ddN3T -
       ddN3T0 * dN4T0 * ddN1T * FP * dN5T - ddN3T0 * dN1T * dN4T0 * FA -
       ddN5T * dN1T * dN3T0 * ddN4T0 * FP - dN4T * dN3T0 * FA * ddN1T0 +
       ddN3T0 * dN4T * dN1T0 * FA - ddN4T * dN3T * ddN1T0 * IS +
       dN3T * ddN1T * ddN4T0 * IS + dN3T * dN4T0 * FA * ddN1T0 -
       dN4T * IP * ddN3T * dN0T0 * ddN1T0 + ddN4T * ddN3T0 * dN1T * IS +
       IA * ddN4T * dN1T0 * dN3T + ddN0T0 * ddN4T * dN1T * IP * dN3T0 +
       dN3T0 * ddN1T * ddN4T0 * FP * dN5T - ddN0T0 * ddN4T * dN1T0 * IP * dN3T -
       FS * ddN4T * ddN3T0 * dN1T0 - IA * dN3T * dN4T0 * ddN1T +
       ddN5T * dN4T * dN3T0 * FP * ddN1T0 + ddN0T0 * IP * dN3T * dN4T0 * ddN1T +
       dN4T * ddN3T * ddN1T0 * IS - ddN5T * dN3T * dN4T0 * FP * ddN1T0 +
       FS * dN1T0 * ddN4T0 * ddN3T + ddN5T * dN1T0 * dN3T * ddN4T0 * FP);
  m_control_points[3] =
      -1.0 /
      (dN2T * dN3T0 * ddN1T * ddN4T0 - dN3T * dN4T0 * ddN2T * ddN1T0 +
       dN1T * ddN4T0 * ddN3T * dN2T0 - dN2T * ddN3T0 * dN4T0 * ddN1T +
       ddN4T * dN1T * ddN2T0 * dN3T0 + dN1T0 * dN3T * ddN4T0 * ddN2T +
       dN4T * dN3T0 * ddN2T * ddN1T0 - dN1T * ddN2T0 * dN4T0 * ddN3T +
       dN2T * dN4T0 * ddN3T * ddN1T0 - dN1T * dN3T0 * ddN4T0 * ddN2T +
       ddN2T0 * dN3T * dN4T0 * ddN1T - ddN4T * ddN2T0 * dN1T0 * dN3T +
       dN2T * ddN4T * ddN3T0 * dN1T0 - dN3T * ddN1T * ddN4T0 * dN2T0 -
       dN4T * ddN2T0 * dN3T0 * ddN1T - dN2T * dN1T0 * ddN4T0 * ddN3T +
       dN4T * ddN2T0 * dN1T0 * ddN3T + ddN3T0 * dN4T * ddN1T * dN2T0 -
       dN4T * ddN3T * dN2T0 * ddN1T0 + ddN3T0 * dN1T * dN4T0 * ddN2T -
       dN2T * ddN4T * dN3T0 * ddN1T0 - ddN3T0 * dN4T * dN1T0 * ddN2T +
       ddN4T * dN3T * dN2T0 * ddN1T0 - ddN4T * ddN3T0 * dN1T * dN2T0) *
      (dN2T * ddN0T0 * ddN4T * dN1T0 * IP + dN4T * IP * dN0T0 * ddN2T * ddN1T0 -
       ddN5T * dN4T * FP * dN2T0 * ddN1T0 - IA * dN1T * dN4T0 * ddN2T +
       ddN5T * dN2T * dN4T0 * FP * ddN1T0 - ddN4T * ddN2T0 * dN1T0 * FP * dN5T -
       dN1T * ddN4T0 * FA * dN2T0 - ddN4T * dN1T * ddN2T0 * IS +
       ddN4T * FP * dN5T * dN2T0 * ddN1T0 + dN2T * IP * ddN1T * ddN4T0 * dN0T0 -
       ddN5T * dN2T * dN1T0 * ddN4T0 * FP + FS * ddN1T * ddN4T0 * dN2T0 +
       IA * dN4T * dN1T0 * ddN2T + ddN5T * dN4T * ddN2T0 * dN1T0 * FP +
       FS * ddN4T * ddN2T0 * dN1T0 + ddN2T0 * dN4T0 * ddN1T * FP * dN5T -
       dN2T * dN4T0 * FA * ddN1T0 - ddN1T * ddN4T0 * FP * dN5T * dN2T0 -
       dN2T * ddN1T * ddN4T0 * IS - FS * ddN2T0 * dN4T0 * ddN1T +
       ddN5T * dN1T * ddN4T0 * FP * dN2T0 + dN1T * ddN2T0 * dN4T0 * FA -
       ddN0T0 * dN4T * dN1T0 * IP * ddN2T + dN1T0 * ddN4T0 * FP * dN5T * ddN2T -
       IA * dN4T * ddN1T * dN2T0 + ddN0T0 * dN4T * IP * ddN1T * dN2T0 -
       dN1T * IP * ddN4T0 * dN0T0 * ddN2T + IA * ddN4T * dN1T * dN2T0 -
       dN4T * ddN2T0 * IP * ddN1T * dN0T0 - dN2T * ddN4T * IP * dN0T0 * ddN1T0 -
       dN2T * ddN0T0 * IP * dN4T0 * ddN1T + dN4T * ddN2T0 * ddN1T * IS +
       FS * dN4T0 * ddN2T * ddN1T0 - FS * dN1T0 * ddN4T0 * ddN2T +
       ddN4T * dN1T * ddN2T0 * IP * dN0T0 + IA * dN2T * dN4T0 * ddN1T -
       dN4T * ddN2T0 * dN1T0 * FA - dN4T0 * FP * dN5T * ddN2T * ddN1T0 -
       dN4T * ddN2T * ddN1T0 * IS - IA * dN2T * ddN4T * dN1T0 +
       dN2T * dN1T0 * ddN4T0 * FA + dN1T * ddN4T0 * ddN2T * IS +
       ddN0T0 * dN1T * IP * dN4T0 * ddN2T - ddN5T * dN1T * ddN2T0 * dN4T0 * FP +
       dN4T * FA * dN2T0 * ddN1T0 + dN2T * ddN4T * ddN1T0 * IS -
       FS * ddN4T * dN2T0 * ddN1T0 - ddN0T0 * ddN4T * dN1T * IP * dN2T0);
  m_control_points[4] =
      -1.0 /
      (dN2T * dN3T0 * ddN1T * ddN4T0 - dN3T * dN4T0 * ddN2T * ddN1T0 +
       dN1T * ddN4T0 * ddN3T * dN2T0 - dN2T * ddN3T0 * dN4T0 * ddN1T +
       ddN4T * dN1T * ddN2T0 * dN3T0 + dN1T0 * dN3T * ddN4T0 * ddN2T +
       dN4T * dN3T0 * ddN2T * ddN1T0 - dN1T * ddN2T0 * dN4T0 * ddN3T +
       dN2T * dN4T0 * ddN3T * ddN1T0 - dN1T * dN3T0 * ddN4T0 * ddN2T +
       ddN2T0 * dN3T * dN4T0 * ddN1T - ddN4T * ddN2T0 * dN1T0 * dN3T +
       dN2T * ddN4T * ddN3T0 * dN1T0 - dN3T * ddN1T * ddN4T0 * dN2T0 -
       dN4T * ddN2T0 * dN3T0 * ddN1T - dN2T * dN1T0 * ddN4T0 * ddN3T +
       dN4T * ddN2T0 * dN1T0 * ddN3T + ddN3T0 * dN4T * ddN1T * dN2T0 -
       dN4T * ddN3T * dN2T0 * ddN1T0 + ddN3T0 * dN1T * dN4T0 * ddN2T -
       dN2T * ddN4T * dN3T0 * ddN1T0 - ddN3T0 * dN4T * dN1T0 * ddN2T +
       ddN4T * dN3T * dN2T0 * ddN1T0 - ddN4T * ddN3T0 * dN1T * dN2T0) *
      (ddN2T0 * dN1T0 * dN3T * FA + ddN5T * dN1T * ddN2T0 * dN3T0 * FP -
       ddN0T0 * IP * dN3T * ddN1T * dN2T0 - IA * dN1T * ddN3T * dN2T0 -
       ddN0T0 * dN1T * IP * dN3T0 * ddN2T - dN2T * ddN0T0 * dN1T0 * IP * ddN3T +
       dN3T * ddN2T * ddN1T0 * IS - FS * dN3T0 * ddN2T * ddN1T0 +
       ddN2T0 * dN1T0 * FP * dN5T * ddN3T - ddN2T0 * dN3T0 * ddN1T * FP * dN5T +
       dN1T * ddN2T0 * ddN3T * IS + ddN0T0 * dN1T0 * IP * dN3T * ddN2T +
       IA * dN3T * ddN1T * dN2T0 + ddN5T * dN2T * ddN3T0 * dN1T0 * FP +
       dN3T0 * FP * dN5T * ddN2T * ddN1T0 + IA * dN2T * dN1T0 * ddN3T +
       dN2T * ddN0T0 * IP * dN3T0 * ddN1T + dN2T * IP * ddN3T * dN0T0 * ddN1T0 -
       ddN3T0 * dN1T * ddN2T * IS - FP * dN5T * ddN3T * dN2T0 * ddN1T0 -
       dN1T * ddN2T0 * IP * ddN3T * dN0T0 - dN3T * FA * dN2T0 * ddN1T0 -
       IA * dN2T * dN3T0 * ddN1T - dN1T * ddN2T0 * dN3T0 * FA +
       ddN3T0 * dN1T * IP * dN0T0 * ddN2T - FS * ddN3T0 * ddN1T * dN2T0 +
       FS * ddN2T0 * dN3T0 * ddN1T - dN2T * ddN3T0 * IP * ddN1T * dN0T0 -
       ddN5T * ddN2T0 * dN1T0 * dN3T * FP + FS * ddN3T * dN2T0 * ddN1T0 +
       ddN3T0 * ddN1T * FP * dN5T * dN2T0 - dN2T * ddN3T * ddN1T0 * IS -
       ddN3T0 * dN1T0 * FP * dN5T * ddN2T + ddN5T * dN3T * FP * dN2T0 * ddN1T0 -
       ddN5T * dN2T * dN3T0 * FP * ddN1T0 - ddN2T0 * dN3T * ddN1T * IS -
       FS * ddN2T0 * dN1T0 * ddN3T - ddN5T * ddN3T0 * dN1T * FP * dN2T0 -
       dN2T * ddN3T0 * dN1T0 * FA + IA * dN1T * dN3T0 * ddN2T +
       ddN0T0 * dN1T * IP * ddN3T * dN2T0 + FS * ddN3T0 * dN1T0 * ddN2T +
       dN2T * ddN3T0 * ddN1T * IS + ddN2T0 * IP * dN3T * ddN1T * dN0T0 +
       dN2T * dN3T0 * FA * ddN1T0 - IA * dN1T0 * dN3T * ddN2T -
       IP * dN3T * dN0T0 * ddN2T * ddN1T0 + ddN3T0 * dN1T * FA * dN2T0);
  m_control_points[5] = FP;
  return;
}

void BSplinesFoot::ComputeControlPointFrom3DataPoint(void) {
  ComputeBasisFunctions(0);
  vector<double> dNi5T0 = m_basis_functions_derivative;
  vector<double> ddNi5T0 = m_basis_functions_sec_derivative;

  ComputeBasisFunctions(m_ToMP[0] / m_FT);
  vector<double> Ni5Tm = m_basis_functions[m_degree];

  ComputeBasisFunctions(1.0);
  vector<double> dNi5T = m_basis_functions_derivative;
  vector<double> ddNi5T = m_basis_functions_sec_derivative;

  double IP = m_IP;
  double IS = m_IS;
  double IA = m_IA;
  double FP = m_FP;
  double FS = m_FS;
  double FA = m_FA;
  double MidP1 = m_MP[0];

  double N1Tm = Ni5Tm[1];
  double N2Tm = Ni5Tm[2];
  double N3Tm = Ni5Tm[3];
  double N4Tm = Ni5Tm[4];
  double N5Tm = Ni5Tm[5];

  double dN0T0 = dNi5T0[0];
  double dN1T0 = dNi5T0[1];
  double dN2T0 = dNi5T0[2];
  double dN3T0 = dNi5T0[3];
  double dN4T0 = dNi5T0[4];

  double dN2T = dNi5T[2];
  double dN3T = dNi5T[3];
  double dN4T = dNi5T[4];
  double dN5T = dNi5T[5];
  double dN6T = dNi5T[6];

  double ddN0T0 = ddNi5T0[0];
  double ddN1T0 = ddNi5T0[1];
  double ddN2T0 = ddNi5T0[2];
  double ddN3T0 = ddNi5T0[3];
  double ddN4T0 = ddNi5T0[4];

  double ddN2T = ddNi5T[2];
  double ddN3T = ddNi5T[3];
  double ddN4T = ddNi5T[4];
  double ddN5T = ddNi5T[5];
  double ddN6T = ddNi5T[6];

  m_control_points[0] = IP;
  m_control_points[1] =
      -(IA * N4Tm * dN3T0 * dN5T * ddN2T -
        N4Tm * ddN2T0 * IP * dN5T * ddN3T * dN0T0 -
        ddN4T0 * N2Tm * dN5T * ddN3T * IS +
        ddN5T * dN2T * MidP1 * dN3T0 * ddN4T0 +
        MidP1 * ddN3T0 * dN4T0 * dN5T * ddN2T +
        dN4T * ddN2T0 * IP * ddN3T * dN0T0 * N5Tm +
        ddN0T0 * dN4T * IP * dN3T0 * N5Tm * ddN2T +
        ddN5T * FS * ddN3T0 * dN4T0 * N2Tm +
        ddN0T0 * ddN4T * IP * dN3T0 * N2Tm * dN5T -
        ddN5T * MidP1 * dN4T * ddN2T0 * dN3T0 +
        ddN3T0 * dN4T0 * dN6T * FP * N5Tm * ddN2T -
        ddN5T * IA * N3Tm * dN4T * dN2T0 +
        ddN0T0 * ddN4T * IP * dN3T * N5Tm * dN2T0 +
        FS * ddN2T0 * dN4T0 * ddN3T * N5Tm -
        ddN5T * IP * dN3T * ddN4T0 * N2Tm * dN0T0 -
        ddN5T * N4Tm * ddN2T0 * dN3T0 * dN6T * FP -
        dN4T * ddN2T0 * ddN3T * N5Tm * IS - N3Tm * ddN4T0 * dN5T * FA * dN2T0 +
        ddN5T * IA * dN4T * dN3T0 * N2Tm - N4Tm * ddN2T0 * dN3T0 * dN5T * FA -
        N3Tm * IP * ddN4T0 * dN5T * dN0T0 * ddN2T -
        ddN0T0 * IP * dN4T0 * N2Tm * dN5T * ddN3T +
        ddN2T0 * ddN6T * dN3T * dN4T0 * FP * N5Tm +
        ddN5T * FS * N4Tm * ddN2T0 * dN3T0 - IA * dN4T * dN3T0 * N5Tm * ddN2T +
        ddN5T * ddN0T0 * N3Tm * dN4T * IP * dN2T0 -
        ddN5T * dN2T * MidP1 * ddN3T0 * dN4T0 +
        IA * ddN4T * N3Tm * dN5T * dN2T0 + IA * dN3T * dN4T0 * N5Tm * ddN2T +
        ddN5T * MidP1 * ddN3T0 * dN4T * dN2T0 -
        MidP1 * ddN2T0 * dN4T0 * dN5T * ddN3T -
        N3Tm * ddN2T0 * ddN6T * dN4T0 * FP * dN5T +
        ddN0T0 * N4Tm * IP * dN5T * ddN3T * dN2T0 +
        N4Tm * ddN2T0 * ddN6T * dN3T0 * FP * dN5T -
        ddN5T * ddN3T0 * dN4T * N2Tm * IS -
        dN2T * ddN3T0 * ddN6T * dN4T0 * FP * N5Tm -
        ddN3T0 * dN4T * IP * dN0T0 * N5Tm * ddN2T -
        ddN5T * FS * N3Tm * ddN2T0 * dN4T0 + ddN5T * IA * N4Tm * dN3T * dN2T0 -
        ddN5T * dN2T * ddN3T0 * N4Tm * IP * dN0T0 -
        ddN3T0 * dN4T * FA * N5Tm * dN2T0 - dN3T * ddN4T0 * N5Tm * ddN2T * IS +
        N3Tm * ddN2T0 * dN4T0 * dN5T * FA + ddN5T * IA * dN2T * N3Tm * dN4T0 -
        FS * ddN3T0 * dN4T0 * N5Tm * ddN2T + IA * dN4T0 * N2Tm * dN5T * ddN3T -
        dN2T * ddN0T0 * ddN4T * IP * dN3T0 * N5Tm +
        ddN4T * MidP1 * ddN2T0 * dN3T0 * dN5T +
        IA * dN2T * ddN4T * dN3T0 * N5Tm - IA * N3Tm * dN4T0 * dN5T * ddN2T +
        IP * dN3T * ddN4T0 * dN0T0 * N5Tm * ddN2T -
        ddN2T0 * dN3T * dN4T0 * FA * N5Tm -
        ddN6T * dN3T0 * ddN4T0 * FP * N2Tm * dN5T -
        ddN5T * FS * dN3T0 * ddN4T0 * N2Tm -
        ddN6T * dN3T * ddN4T0 * FP * N5Tm * dN2T0 -
        ddN0T0 * IP * dN3T * dN4T0 * N5Tm * ddN2T +
        ddN3T0 * dN4T * ddN6T * FP * N5Tm * dN2T0 -
        ddN5T * IA * dN2T * N4Tm * dN3T0 -
        dN3T0 * dN6T * ddN4T0 * FP * N5Tm * ddN2T +
        ddN3T0 * dN4T * N5Tm * ddN2T * IS -
        ddN5T * ddN0T0 * dN4T * IP * dN3T0 * N2Tm +
        ddN3T0 * N4Tm * dN5T * FA * dN2T0 +
        IP * ddN4T0 * N2Tm * dN5T * ddN3T * dN0T0 -
        dN2T * dN3T0 * ddN4T0 * FA * N5Tm +
        dN2T * ddN4T * ddN3T0 * IP * dN0T0 * N5Tm +
        dN3T * ddN4T0 * FA * N5Tm * dN2T0 +
        ddN5T * MidP1 * ddN2T0 * dN3T * dN4T0 +
        ddN4T * ddN2T0 * dN3T0 * dN6T * FP * N5Tm -
        FS * ddN4T0 * ddN3T * N5Tm * dN2T0 +
        FS * ddN4T * ddN3T0 * N5Tm * dN2T0 +
        dN2T * ddN6T * dN3T0 * ddN4T0 * FP * N5Tm -
        dN4T * ddN2T0 * ddN6T * dN3T0 * FP * N5Tm -
        ddN5T * ddN0T0 * N4Tm * IP * dN3T * dN2T0 -
        ddN4T * N3Tm * ddN2T0 * dN5T * IS +
        dN6T * ddN4T0 * FP * ddN3T * N5Tm * dN2T0 -
        ddN5T * MidP1 * dN3T * ddN4T0 * dN2T0 -
        ddN4T * ddN2T0 * IP * dN3T * dN0T0 * N5Tm +
        MidP1 * ddN4T0 * dN5T * ddN3T * dN2T0 +
        ddN5T * N3Tm * ddN2T0 * dN4T0 * dN6T * FP +
        N3Tm * ddN4T0 * dN5T * ddN2T * IS -
        ddN2T0 * dN4T0 * dN6T * FP * ddN3T * N5Tm -
        ddN5T * N3Tm * dN4T * ddN2T0 * IP * dN0T0 -
        ddN5T * dN2T * N3Tm * ddN4T0 * IS +
        ddN5T * dN2T * ddN0T0 * N4Tm * IP * dN3T0 -
        MidP1 * dN3T0 * ddN4T0 * dN5T * ddN2T -
        ddN5T * dN2T * ddN0T0 * N3Tm * IP * dN4T0 +
        ddN4T * ddN2T0 * dN3T * N5Tm * IS +
        ddN5T * N4Tm * ddN2T0 * IP * dN3T * dN0T0 -
        ddN3T0 * N4Tm * ddN6T * FP * dN5T * dN2T0 +
        IA * dN4T * ddN3T * N5Tm * dN2T0 +
        ddN5T * ddN3T0 * N4Tm * dN6T * FP * dN2T0 -
        FS * ddN4T * ddN2T0 * dN3T0 * N5Tm +
        ddN4T * N3Tm * ddN2T0 * IP * dN5T * dN0T0 +
        dN4T * ddN2T0 * dN3T0 * FA * N5Tm - ddN5T * IA * dN3T * dN4T0 * N2Tm +
        ddN5T * dN3T * ddN4T0 * N2Tm * IS +
        ddN5T * ddN3T0 * dN4T * IP * N2Tm * dN0T0 +
        N3Tm * ddN6T * ddN4T0 * FP * dN5T * dN2T0 -
        ddN0T0 * ddN4T * N3Tm * IP * dN5T * dN2T0 -
        ddN0T0 * dN4T * IP * ddN3T * N5Tm * dN2T0 +
        dN2T * ddN0T0 * IP * dN4T0 * ddN3T * N5Tm +
        ddN5T * N3Tm * dN4T * ddN2T0 * IS - ddN5T * FS * ddN3T0 * N4Tm * dN2T0 +
        ddN0T0 * N3Tm * IP * dN4T0 * dN5T * ddN2T +
        N4Tm * ddN2T0 * dN5T * ddN3T * IS - IA * ddN4T * dN3T * N5Tm * dN2T0 +
        ddN5T * ddN0T0 * IP * dN3T * dN4T0 * N2Tm +
        ddN3T0 * N4Tm * IP * dN5T * dN0T0 * ddN2T +
        ddN5T * dN3T0 * dN6T * ddN4T0 * FP * N2Tm -
        ddN3T0 * N4Tm * dN5T * ddN2T * IS -
        ddN4T * MidP1 * ddN3T0 * dN5T * dN2T0 -
        ddN5T * N4Tm * ddN2T0 * dN3T * IS - IA * ddN4T * dN3T0 * N2Tm * dN5T -
        ddN5T * N3Tm * dN6T * ddN4T0 * FP * dN2T0 -
        ddN4T * ddN3T0 * dN6T * FP * N5Tm * dN2T0 +
        FS * dN3T0 * ddN4T0 * N5Tm * ddN2T -
        dN2T * IP * ddN4T0 * ddN3T * dN0T0 * N5Tm +
        ddN5T * dN2T * ddN3T0 * N4Tm * IS -
        ddN0T0 * N4Tm * IP * dN3T0 * dN5T * ddN2T +
        dN2T * ddN3T0 * dN4T0 * FA * N5Tm -
        ddN5T * ddN3T0 * dN4T0 * dN6T * FP * N2Tm -
        ddN4T * ddN3T0 * IP * N2Tm * dN5T * dN0T0 +
        dN3T0 * ddN4T0 * N2Tm * dN5T * FA + ddN5T * FS * N3Tm * ddN4T0 * dN2T0 -
        IA * dN2T * dN4T0 * ddN3T * N5Tm - ddN3T0 * dN4T0 * N2Tm * dN5T * FA +
        ddN4T * ddN3T0 * N2Tm * dN5T * IS - IA * N4Tm * dN5T * ddN3T * dN2T0 -
        dN2T * ddN4T * ddN3T0 * N5Tm * IS + dN2T * ddN4T0 * ddN3T * N5Tm * IS +
        ddN5T * dN2T * N3Tm * IP * ddN4T0 * dN0T0 +
        ddN3T0 * ddN6T * dN4T0 * FP * N2Tm * dN5T) /
      (ddN5T * dN3T * dN4T0 * N2Tm * ddN1T0 -
       ddN5T * dN4T * dN3T0 * N2Tm * ddN1T0 -
       ddN5T * N1Tm * ddN3T0 * dN4T * dN2T0 +
       dN4T * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       dN3T * dN4T0 * N5Tm * ddN2T * ddN1T0 +
       ddN5T * N1Tm * dN4T * ddN2T0 * dN3T0 -
       N1Tm * ddN4T0 * dN5T * ddN3T * dN2T0 -
       ddN4T * N3Tm * dN5T * dN2T0 * ddN1T0 +
       ddN5T * N4Tm * ddN2T0 * dN1T0 * dN3T -
       N1Tm * ddN3T0 * dN4T0 * dN5T * ddN2T -
       ddN5T * N1Tm * ddN2T0 * dN3T * dN4T0 -
       N4Tm * dN3T0 * dN5T * ddN2T * ddN1T0 +
       ddN5T * ddN3T0 * dN4T * dN1T0 * N2Tm +
       ddN4T * N3Tm * ddN2T0 * dN1T0 * dN5T +
       ddN5T * N1Tm * dN3T * ddN4T0 * dN2T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * dN2T0 -
       ddN4T * ddN2T0 * dN1T0 * dN3T * N5Tm -
       dN4T * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * dN4T * dN2T0 * ddN1T0 +
       ddN5T * dN2T * N3Tm * dN1T0 * ddN4T0 -
       N3Tm * dN1T0 * ddN4T0 * dN5T * ddN2T -
       N4Tm * ddN2T0 * dN1T0 * dN5T * ddN3T +
       dN1T0 * dN3T * ddN4T0 * N5Tm * ddN2T +
       N1Tm * ddN2T0 * dN4T0 * dN5T * ddN3T +
       dN2T * dN4T0 * ddN3T * N5Tm * ddN1T0 -
       ddN4T * ddN3T0 * dN1T0 * N2Tm * dN5T -
       ddN5T * dN1T0 * dN3T * ddN4T0 * N2Tm +
       N4Tm * dN5T * ddN3T * dN2T0 * ddN1T0 +
       dN2T * ddN4T * ddN3T0 * dN1T0 * N5Tm -
       ddN5T * dN2T * ddN3T0 * N4Tm * dN1T0 -
       ddN5T * dN2T * N1Tm * dN3T0 * ddN4T0 +
       ddN5T * dN2T * N1Tm * ddN3T0 * dN4T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * ddN2T -
       ddN5T * dN2T * N3Tm * dN4T0 * ddN1T0 -
       dN2T * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN4T * dN3T0 * N2Tm * dN5T * ddN1T0 -
       dN4T0 * N2Tm * dN5T * ddN3T * ddN1T0 +
       dN4T * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN3T0 * dN4T * dN1T0 * N5Tm * ddN2T -
       ddN5T * N4Tm * dN3T * dN2T0 * ddN1T0 +
       ddN4T * dN3T * N5Tm * dN2T0 * ddN1T0 +
       dN1T0 * ddN4T0 * N2Tm * dN5T * ddN3T -
       N1Tm * ddN4T * ddN2T0 * dN3T0 * dN5T +
       ddN5T * dN2T * N4Tm * dN3T0 * ddN1T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * ddN2T -
       ddN5T * N3Tm * dN4T * ddN2T0 * dN1T0 -
       dN2T * ddN4T * dN3T0 * N5Tm * ddN1T0 +
       N3Tm * dN4T0 * dN5T * ddN2T * ddN1T0);
  m_control_points[2] =
      (ddN0T0 * N1Tm * ddN4T * IP * dN3T0 * dN5T -
       IA * N1Tm * ddN4T * dN3T0 * dN5T +
       ddN5T * MidP1 * dN3T * dN4T0 * ddN1T0 +
       dN4T * dN3T0 * FA * N5Tm * ddN1T0 - N1Tm * ddN3T0 * dN4T0 * dN5T * FA -
       N1Tm * ddN4T0 * dN5T * ddN3T * IS - IA * ddN4T * dN1T0 * dN3T * N5Tm -
       ddN5T * N4Tm * dN3T0 * dN6T * FP * ddN1T0 +
       N1Tm * IP * ddN4T0 * dN5T * ddN3T * dN0T0 +
       ddN6T * dN3T * dN4T0 * FP * N5Tm * ddN1T0 +
       N3Tm * dN1T0 * ddN6T * ddN4T0 * FP * dN5T +
       ddN5T * N4Tm * IP * dN3T * dN0T0 * ddN1T0 -
       IA * N4Tm * dN1T0 * dN5T * ddN3T + ddN5T * FS * N3Tm * dN1T0 * ddN4T0 +
       ddN0T0 * ddN4T * dN1T0 * IP * dN3T * N5Tm -
       ddN5T * MidP1 * dN1T0 * dN3T * ddN4T0 +
       ddN5T * N1Tm * dN3T0 * dN6T * ddN4T0 * FP +
       FS * dN4T0 * ddN3T * N5Tm * ddN1T0 +
       N4Tm * ddN6T * dN3T0 * FP * dN5T * ddN1T0 +
       dN1T0 * dN6T * ddN4T0 * FP * ddN3T * N5Tm +
       IA * ddN4T * N3Tm * dN1T0 * dN5T - ddN5T * N1Tm * ddN3T0 * dN4T * IS -
       ddN4T * ddN3T0 * dN1T0 * dN6T * FP * N5Tm -
       dN3T * dN4T0 * FA * N5Tm * ddN1T0 +
       ddN0T0 * N4Tm * dN1T0 * IP * dN5T * ddN3T -
       ddN5T * N1Tm * ddN3T0 * dN4T0 * dN6T * FP -
       MidP1 * dN4T0 * dN5T * ddN3T * ddN1T0 +
       FS * ddN4T * ddN3T0 * dN1T0 * N5Tm - N3Tm * dN1T0 * ddN4T0 * dN5T * FA -
       ddN5T * IA * N1Tm * dN3T * dN4T0 -
       dN4T * ddN6T * dN3T0 * FP * N5Tm * ddN1T0 -
       dN4T * ddN3T * N5Tm * ddN1T0 * IS + ddN5T * N3Tm * dN4T * ddN1T0 * IS +
       ddN5T * MidP1 * ddN3T0 * dN4T * dN1T0 +
       ddN3T0 * dN4T * dN1T0 * ddN6T * FP * N5Tm +
       MidP1 * dN1T0 * ddN4T0 * dN5T * ddN3T -
       N4Tm * IP * dN5T * ddN3T * dN0T0 * ddN1T0 +
       IA * dN4T * dN1T0 * ddN3T * N5Tm - FS * ddN4T * dN3T0 * N5Tm * ddN1T0 -
       dN4T0 * dN6T * FP * ddN3T * N5Tm * ddN1T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * IS - FS * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN5T * N1Tm * dN3T * ddN4T0 * IS - ddN5T * IA * N3Tm * dN4T * dN1T0 -
       ddN4T * N3Tm * dN5T * ddN1T0 * IS + N3Tm * dN4T0 * dN5T * FA * ddN1T0 +
       dN1T0 * dN3T * ddN4T0 * FA * N5Tm +
       ddN4T * N3Tm * IP * dN5T * dN0T0 * ddN1T0 +
       ddN5T * IA * N1Tm * dN4T * dN3T0 + ddN4T * dN3T * N5Tm * ddN1T0 * IS -
       ddN5T * N4Tm * dN3T * ddN1T0 * IS +
       ddN5T * ddN3T0 * N4Tm * dN1T0 * dN6T * FP +
       ddN4T * dN3T0 * dN6T * FP * N5Tm * ddN1T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * FA -
       ddN5T * MidP1 * dN4T * dN3T0 * ddN1T0 +
       IA * N1Tm * dN4T0 * dN5T * ddN3T -
       ddN5T * N3Tm * dN1T0 * dN6T * ddN4T0 * FP -
       ddN5T * FS * N3Tm * dN4T0 * ddN1T0 -
       ddN0T0 * dN4T * dN1T0 * IP * ddN3T * N5Tm -
       ddN5T * ddN0T0 * N1Tm * dN4T * IP * dN3T0 -
       ddN4T * IP * dN3T * dN0T0 * N5Tm * ddN1T0 +
       N1Tm * ddN3T0 * ddN6T * dN4T0 * FP * dN5T +
       ddN5T * N3Tm * dN4T0 * dN6T * FP * ddN1T0 -
       dN1T0 * ddN6T * dN3T * ddN4T0 * FP * N5Tm -
       ddN5T * N3Tm * dN4T * IP * dN0T0 * ddN1T0 +
       ddN5T * N1Tm * ddN3T0 * dN4T * IP * dN0T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * FA + ddN5T * FS * N4Tm * dN3T0 * ddN1T0 -
       ddN5T * ddN0T0 * N4Tm * dN1T0 * IP * dN3T -
       ddN5T * FS * ddN3T0 * N4Tm * dN1T0 -
       ddN3T0 * N4Tm * dN1T0 * ddN6T * FP * dN5T +
       ddN5T * N1Tm * FS * ddN3T0 * dN4T0 +
       dN4T * IP * ddN3T * dN0T0 * N5Tm * ddN1T0 -
       ddN0T0 * N1Tm * IP * dN4T0 * dN5T * ddN3T -
       N4Tm * dN3T0 * dN5T * FA * ddN1T0 +
       ddN5T * ddN0T0 * N1Tm * IP * dN3T * dN4T0 -
       ddN0T0 * ddN4T * N3Tm * dN1T0 * IP * dN5T -
       ddN3T0 * dN4T * dN1T0 * FA * N5Tm -
       ddN4T * MidP1 * ddN3T0 * dN1T0 * dN5T +
       N4Tm * dN5T * ddN3T * ddN1T0 * IS -
       ddN5T * N1Tm * IP * dN3T * ddN4T0 * dN0T0 +
       ddN5T * ddN0T0 * N3Tm * dN4T * dN1T0 * IP -
       N3Tm * ddN6T * dN4T0 * FP * dN5T * ddN1T0 -
       ddN5T * N1Tm * FS * dN3T0 * ddN4T0 -
       N1Tm * ddN4T * ddN3T0 * IP * dN5T * dN0T0 +
       ddN5T * IA * N4Tm * dN1T0 * dN3T +
       ddN4T * MidP1 * dN3T0 * dN5T * ddN1T0 -
       N1Tm * ddN6T * dN3T0 * ddN4T0 * FP * dN5T) /
      (ddN5T * dN3T * dN4T0 * N2Tm * ddN1T0 -
       ddN5T * dN4T * dN3T0 * N2Tm * ddN1T0 -
       ddN5T * N1Tm * ddN3T0 * dN4T * dN2T0 +
       dN4T * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       dN3T * dN4T0 * N5Tm * ddN2T * ddN1T0 +
       ddN5T * N1Tm * dN4T * ddN2T0 * dN3T0 -
       N1Tm * ddN4T0 * dN5T * ddN3T * dN2T0 -
       ddN4T * N3Tm * dN5T * dN2T0 * ddN1T0 +
       ddN5T * N4Tm * ddN2T0 * dN1T0 * dN3T -
       N1Tm * ddN3T0 * dN4T0 * dN5T * ddN2T -
       ddN5T * N1Tm * ddN2T0 * dN3T * dN4T0 -
       N4Tm * dN3T0 * dN5T * ddN2T * ddN1T0 +
       ddN5T * ddN3T0 * dN4T * dN1T0 * N2Tm +
       ddN4T * N3Tm * ddN2T0 * dN1T0 * dN5T +
       ddN5T * N1Tm * dN3T * ddN4T0 * dN2T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * dN2T0 -
       ddN4T * ddN2T0 * dN1T0 * dN3T * N5Tm -
       dN4T * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * dN4T * dN2T0 * ddN1T0 +
       ddN5T * dN2T * N3Tm * dN1T0 * ddN4T0 -
       N3Tm * dN1T0 * ddN4T0 * dN5T * ddN2T -
       N4Tm * ddN2T0 * dN1T0 * dN5T * ddN3T +
       dN1T0 * dN3T * ddN4T0 * N5Tm * ddN2T +
       N1Tm * ddN2T0 * dN4T0 * dN5T * ddN3T +
       dN2T * dN4T0 * ddN3T * N5Tm * ddN1T0 -
       ddN4T * ddN3T0 * dN1T0 * N2Tm * dN5T -
       ddN5T * dN1T0 * dN3T * ddN4T0 * N2Tm +
       N4Tm * dN5T * ddN3T * dN2T0 * ddN1T0 +
       dN2T * ddN4T * ddN3T0 * dN1T0 * N5Tm -
       ddN5T * dN2T * ddN3T0 * N4Tm * dN1T0 -
       ddN5T * dN2T * N1Tm * dN3T0 * ddN4T0 +
       ddN5T * dN2T * N1Tm * ddN3T0 * dN4T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * ddN2T -
       ddN5T * dN2T * N3Tm * dN4T0 * ddN1T0 -
       dN2T * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN4T * dN3T0 * N2Tm * dN5T * ddN1T0 -
       dN4T0 * N2Tm * dN5T * ddN3T * ddN1T0 +
       dN4T * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN3T0 * dN4T * dN1T0 * N5Tm * ddN2T -
       ddN5T * N4Tm * dN3T * dN2T0 * ddN1T0 +
       ddN4T * dN3T * N5Tm * dN2T0 * ddN1T0 +
       dN1T0 * ddN4T0 * N2Tm * dN5T * ddN3T -
       N1Tm * ddN4T * ddN2T0 * dN3T0 * dN5T +
       ddN5T * dN2T * N4Tm * dN3T0 * ddN1T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * ddN2T -
       ddN5T * N3Tm * dN4T * ddN2T0 * dN1T0 -
       dN2T * ddN4T * dN3T0 * N5Tm * ddN1T0 +
       N3Tm * dN4T0 * dN5T * ddN2T * ddN1T0);
  m_control_points[3] =
      -(FS * ddN4T * ddN2T0 * dN1T0 * N5Tm + N1Tm * ddN4T0 * dN5T * FA * dN2T0 +
        ddN5T * IA * dN2T * N4Tm * dN1T0 +
        ddN5T * dN2T * N4Tm * IP * dN0T0 * ddN1T0 +
        dN1T0 * ddN6T * ddN4T0 * FP * N2Tm * dN5T +
        MidP1 * dN1T0 * ddN4T0 * dN5T * ddN2T +
        dN1T0 * dN6T * ddN4T0 * FP * N5Tm * ddN2T +
        ddN5T * dN2T * MidP1 * dN4T0 * ddN1T0 +
        ddN5T * FS * dN1T0 * ddN4T0 * N2Tm +
        N4Tm * ddN6T * FP * dN5T * dN2T0 * ddN1T0 -
        dN4T0 * dN6T * FP * N5Tm * ddN2T * ddN1T0 -
        ddN5T * N1Tm * dN4T * ddN2T0 * IS +
        ddN4T * IP * N2Tm * dN5T * dN0T0 * ddN1T0 +
        dN4T * ddN2T0 * dN1T0 * ddN6T * FP * N5Tm +
        ddN5T * MidP1 * dN4T * ddN2T0 * dN1T0 -
        IA * N4Tm * dN1T0 * dN5T * ddN2T + ddN5T * N1Tm * FS * ddN2T0 * dN4T0 +
        ddN5T * dN4T0 * dN6T * FP * N2Tm * ddN1T0 +
        ddN5T * IA * N1Tm * dN4T * dN2T0 +
        ddN5T * N4Tm * ddN2T0 * dN1T0 * dN6T * FP -
        ddN5T * N1Tm * ddN2T0 * dN4T0 * dN6T * FP +
        dN2T * ddN4T * N5Tm * ddN1T0 * IS - FS * dN1T0 * ddN4T0 * N5Tm * ddN2T +
        FS * dN4T0 * N5Tm * ddN2T * ddN1T0 - N1Tm * ddN2T0 * dN4T0 * dN5T * FA -
        N4Tm * ddN2T0 * dN1T0 * ddN6T * FP * dN5T +
        ddN4T * dN6T * FP * N5Tm * dN2T0 * ddN1T0 -
        ddN5T * dN2T * ddN0T0 * N4Tm * dN1T0 * IP +
        ddN0T0 * N4Tm * dN1T0 * IP * dN5T * ddN2T -
        ddN5T * dN2T * N1Tm * IP * ddN4T0 * dN0T0 +
        N1Tm * IP * ddN4T0 * dN5T * dN0T0 * ddN2T +
        N1Tm * ddN4T * ddN2T0 * dN5T * IS + N4Tm * ddN2T0 * dN1T0 * dN5T * FA +
        IA * dN4T * dN1T0 * N5Tm * ddN2T - ddN5T * FS * N4Tm * ddN2T0 * dN1T0 +
        dN4T * IP * dN0T0 * N5Tm * ddN2T * ddN1T0 +
        N1Tm * ddN2T0 * ddN6T * dN4T0 * FP * dN5T +
        dN2T * ddN6T * dN4T0 * FP * N5Tm * ddN1T0 +
        ddN4T * MidP1 * dN5T * dN2T0 * ddN1T0 -
        N1Tm * ddN4T * ddN2T0 * IP * dN5T * dN0T0 +
        IA * ddN4T * dN1T0 * N2Tm * dN5T -
        N4Tm * IP * dN5T * dN0T0 * ddN2T * ddN1T0 -
        ddN5T * N1Tm * FS * ddN4T0 * dN2T0 + N4Tm * dN5T * ddN2T * ddN1T0 * IS -
        ddN5T * IA * dN4T * dN1T0 * N2Tm +
        dN2T * ddN0T0 * ddN4T * dN1T0 * IP * N5Tm -
        MidP1 * dN4T0 * dN5T * ddN2T * ddN1T0 -
        ddN0T0 * N1Tm * IP * dN4T0 * dN5T * ddN2T +
        ddN5T * N1Tm * dN4T * ddN2T0 * IP * dN0T0 -
        N1Tm * ddN4T0 * dN5T * ddN2T * IS + dN4T * FA * N5Tm * dN2T0 * ddN1T0 -
        ddN5T * dN2T * MidP1 * dN1T0 * ddN4T0 -
        FS * ddN4T * N5Tm * dN2T0 * ddN1T0 -
        ddN0T0 * dN4T * dN1T0 * IP * N5Tm * ddN2T -
        ddN0T0 * ddN4T * dN1T0 * IP * N2Tm * dN5T -
        ddN4T * N2Tm * dN5T * ddN1T0 * IS - ddN5T * FS * dN4T0 * N2Tm * ddN1T0 -
        ddN5T * ddN0T0 * N1Tm * dN4T * IP * dN2T0 +
        ddN5T * dN4T * N2Tm * ddN1T0 * IS - IA * N1Tm * ddN4T * dN5T * dN2T0 -
        dN4T * ddN6T * FP * N5Tm * dN2T0 * ddN1T0 +
        ddN5T * ddN0T0 * dN4T * dN1T0 * IP * N2Tm -
        dN2T * ddN4T * IP * dN0T0 * N5Tm * ddN1T0 +
        ddN0T0 * N1Tm * ddN4T * IP * dN5T * dN2T0 +
        IA * N1Tm * dN4T0 * dN5T * ddN2T -
        ddN5T * dN4T * IP * N2Tm * dN0T0 * ddN1T0 -
        dN1T0 * ddN4T0 * N2Tm * dN5T * FA -
        dN2T * dN1T0 * ddN6T * ddN4T0 * FP * N5Tm -
        dN4T * N5Tm * ddN2T * ddN1T0 * IS + ddN5T * dN2T * N1Tm * ddN4T0 * IS -
        ddN5T * N4Tm * dN6T * FP * dN2T0 * ddN1T0 -
        ddN5T * MidP1 * dN4T * dN2T0 * ddN1T0 +
        ddN5T * dN2T * ddN0T0 * N1Tm * IP * dN4T0 -
        dN4T * ddN2T0 * dN1T0 * FA * N5Tm -
        N1Tm * ddN6T * ddN4T0 * FP * dN5T * dN2T0 -
        ddN5T * dN1T0 * dN6T * ddN4T0 * FP * N2Tm -
        dN2T * dN4T0 * FA * N5Tm * ddN1T0 + ddN5T * FS * N4Tm * dN2T0 * ddN1T0 -
        N4Tm * dN5T * FA * dN2T0 * ddN1T0 - ddN5T * dN2T * N4Tm * ddN1T0 * IS -
        ddN4T * MidP1 * ddN2T0 * dN1T0 * dN5T -
        ddN5T * IA * dN2T * N1Tm * dN4T0 +
        ddN5T * N1Tm * dN6T * ddN4T0 * FP * dN2T0 -
        IA * dN2T * ddN4T * dN1T0 * N5Tm + dN2T * dN1T0 * ddN4T0 * FA * N5Tm -
        ddN6T * dN4T0 * FP * N2Tm * dN5T * ddN1T0 -
        ddN4T * ddN2T0 * dN1T0 * dN6T * FP * N5Tm +
        dN4T0 * N2Tm * dN5T * FA * ddN1T0) /
      (ddN5T * dN3T * dN4T0 * N2Tm * ddN1T0 -
       ddN5T * dN4T * dN3T0 * N2Tm * ddN1T0 -
       ddN5T * N1Tm * ddN3T0 * dN4T * dN2T0 +
       dN4T * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       dN3T * dN4T0 * N5Tm * ddN2T * ddN1T0 +
       ddN5T * N1Tm * dN4T * ddN2T0 * dN3T0 -
       N1Tm * ddN4T0 * dN5T * ddN3T * dN2T0 -
       ddN4T * N3Tm * dN5T * dN2T0 * ddN1T0 +
       ddN5T * N4Tm * ddN2T0 * dN1T0 * dN3T -
       N1Tm * ddN3T0 * dN4T0 * dN5T * ddN2T -
       ddN5T * N1Tm * ddN2T0 * dN3T * dN4T0 -
       N4Tm * dN3T0 * dN5T * ddN2T * ddN1T0 +
       ddN5T * ddN3T0 * dN4T * dN1T0 * N2Tm +
       ddN4T * N3Tm * ddN2T0 * dN1T0 * dN5T +
       ddN5T * N1Tm * dN3T * ddN4T0 * dN2T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * dN2T0 -
       ddN4T * ddN2T0 * dN1T0 * dN3T * N5Tm -
       dN4T * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * dN4T * dN2T0 * ddN1T0 +
       ddN5T * dN2T * N3Tm * dN1T0 * ddN4T0 -
       N3Tm * dN1T0 * ddN4T0 * dN5T * ddN2T -
       N4Tm * ddN2T0 * dN1T0 * dN5T * ddN3T +
       dN1T0 * dN3T * ddN4T0 * N5Tm * ddN2T +
       N1Tm * ddN2T0 * dN4T0 * dN5T * ddN3T +
       dN2T * dN4T0 * ddN3T * N5Tm * ddN1T0 -
       ddN4T * ddN3T0 * dN1T0 * N2Tm * dN5T -
       ddN5T * dN1T0 * dN3T * ddN4T0 * N2Tm +
       N4Tm * dN5T * ddN3T * dN2T0 * ddN1T0 +
       dN2T * ddN4T * ddN3T0 * dN1T0 * N5Tm -
       ddN5T * dN2T * ddN3T0 * N4Tm * dN1T0 -
       ddN5T * dN2T * N1Tm * dN3T0 * ddN4T0 +
       ddN5T * dN2T * N1Tm * ddN3T0 * dN4T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * ddN2T -
       ddN5T * dN2T * N3Tm * dN4T0 * ddN1T0 -
       dN2T * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN4T * dN3T0 * N2Tm * dN5T * ddN1T0 -
       dN4T0 * N2Tm * dN5T * ddN3T * ddN1T0 +
       dN4T * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN3T0 * dN4T * dN1T0 * N5Tm * ddN2T -
       ddN5T * N4Tm * dN3T * dN2T0 * ddN1T0 +
       ddN4T * dN3T * N5Tm * dN2T0 * ddN1T0 +
       dN1T0 * ddN4T0 * N2Tm * dN5T * ddN3T -
       N1Tm * ddN4T * ddN2T0 * dN3T0 * dN5T +
       ddN5T * dN2T * N4Tm * dN3T0 * ddN1T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * ddN2T -
       ddN5T * N3Tm * dN4T * ddN2T0 * dN1T0 -
       dN2T * ddN4T * dN3T0 * N5Tm * ddN1T0 +
       N3Tm * dN4T0 * dN5T * ddN2T * ddN1T0);
  m_control_points[4] =
      (dN2T * ddN6T * dN3T0 * FP * N5Tm * ddN1T0 -
       ddN3T0 * dN1T0 * N2Tm * dN5T * FA + dN3T0 * N2Tm * dN5T * FA * ddN1T0 -
       ddN5T * IA * dN2T * N1Tm * dN3T0 -
       ddN6T * dN3T0 * FP * N2Tm * dN5T * ddN1T0 -
       ddN5T * ddN0T0 * N1Tm * IP * dN3T * dN2T0 -
       ddN5T * FS * dN3T0 * N2Tm * ddN1T0 - ddN5T * N1Tm * FS * ddN3T0 * dN2T0 -
       ddN5T * ddN3T0 * dN1T0 * dN6T * FP * N2Tm -
       ddN5T * N3Tm * dN6T * FP * dN2T0 * ddN1T0 -
       N1Tm * ddN3T0 * ddN6T * FP * dN5T * dN2T0 +
       dN2T * ddN3T0 * dN1T0 * FA * N5Tm +
       ddN5T * MidP1 * ddN2T0 * dN1T0 * dN3T -
       ddN5T * dN2T * N3Tm * ddN1T0 * IS + ddN5T * dN3T * N2Tm * ddN1T0 * IS -
       IA * dN2T * dN1T0 * ddN3T * N5Tm + ddN5T * FS * N3Tm * dN2T0 * ddN1T0 +
       ddN3T0 * dN1T0 * ddN6T * FP * N2Tm * dN5T -
       ddN0T0 * N1Tm * IP * dN3T0 * dN5T * ddN2T +
       FS * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       ddN0T0 * dN1T0 * IP * N2Tm * dN5T * ddN3T +
       N1Tm * ddN3T0 * dN5T * FA * dN2T0 +
       ddN5T * dN2T * ddN0T0 * N1Tm * IP * dN3T0 -
       N3Tm * dN5T * FA * dN2T0 * ddN1T0 -
       ddN5T * IP * dN3T * N2Tm * dN0T0 * ddN1T0 -
       N3Tm * IP * dN5T * dN0T0 * ddN2T * ddN1T0 +
       MidP1 * dN5T * ddN3T * dN2T0 * ddN1T0 -
       ddN5T * IA * dN1T0 * dN3T * N2Tm + N1Tm * ddN2T0 * dN5T * ddN3T * IS +
       N1Tm * ddN3T0 * IP * dN5T * dN0T0 * ddN2T +
       ddN5T * dN2T * N1Tm * ddN3T0 * IS - FS * ddN3T * N5Tm * dN2T0 * ddN1T0 -
       ddN2T0 * dN1T0 * dN6T * FP * ddN3T * N5Tm +
       N3Tm * dN5T * ddN2T * ddN1T0 * IS -
       ddN5T * dN2T * N1Tm * ddN3T0 * IP * dN0T0 +
       IP * N2Tm * dN5T * ddN3T * dN0T0 * ddN1T0 -
       ddN0T0 * dN1T0 * IP * dN3T * N5Tm * ddN2T +
       ddN5T * FS * ddN3T0 * dN1T0 * N2Tm +
       ddN5T * N1Tm * ddN2T0 * IP * dN3T * dN0T0 +
       IA * N1Tm * dN3T0 * dN5T * ddN2T - ddN2T0 * dN1T0 * dN3T * FA * N5Tm -
       ddN5T * FS * N3Tm * ddN2T0 * dN1T0 - N1Tm * ddN3T0 * dN5T * ddN2T * IS +
       dN6T * FP * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * ddN2T0 * dN1T0 * dN6T * FP -
       ddN5T * MidP1 * dN3T * dN2T0 * ddN1T0 -
       ddN5T * dN2T * MidP1 * ddN3T0 * dN1T0 -
       IA * N1Tm * dN5T * ddN3T * dN2T0 -
       ddN5T * dN2T * ddN0T0 * N3Tm * dN1T0 * IP -
       N2Tm * dN5T * ddN3T * ddN1T0 * IS +
       IP * dN3T * dN0T0 * N5Tm * ddN2T * ddN1T0 -
       dN2T * dN3T0 * FA * N5Tm * ddN1T0 - dN3T * N5Tm * ddN2T * ddN1T0 * IS +
       N3Tm * ddN6T * FP * dN5T * dN2T0 * ddN1T0 +
       IA * dN1T0 * N2Tm * dN5T * ddN3T + dN2T * ddN3T * N5Tm * ddN1T0 * IS -
       IA * N3Tm * dN1T0 * dN5T * ddN2T +
       dN2T * ddN0T0 * dN1T0 * IP * ddN3T * N5Tm +
       ddN5T * dN3T0 * dN6T * FP * N2Tm * ddN1T0 +
       N3Tm * ddN2T0 * dN1T0 * dN5T * FA - ddN5T * N1Tm * ddN2T0 * dN3T * IS +
       ddN0T0 * N3Tm * dN1T0 * IP * dN5T * ddN2T -
       FS * ddN3T0 * dN1T0 * N5Tm * ddN2T + ddN5T * N1Tm * FS * ddN2T0 * dN3T0 +
       ddN5T * dN2T * MidP1 * dN3T0 * ddN1T0 +
       ddN2T0 * dN1T0 * ddN6T * dN3T * FP * N5Tm -
       MidP1 * ddN2T0 * dN1T0 * dN5T * ddN3T +
       N1Tm * ddN2T0 * ddN6T * dN3T0 * FP * dN5T +
       ddN0T0 * N1Tm * IP * dN5T * ddN3T * dN2T0 +
       ddN3T0 * dN1T0 * dN6T * FP * N5Tm * ddN2T -
       dN2T * ddN3T0 * dN1T0 * ddN6T * FP * N5Tm -
       dN3T0 * dN6T * FP * N5Tm * ddN2T * ddN1T0 -
       N3Tm * ddN2T0 * dN1T0 * ddN6T * FP * dN5T +
       ddN5T * ddN0T0 * dN1T0 * IP * dN3T * N2Tm -
       N1Tm * ddN2T0 * dN3T0 * dN5T * FA + IA * dN1T0 * dN3T * N5Tm * ddN2T +
       ddN5T * N1Tm * ddN3T0 * dN6T * FP * dN2T0 -
       MidP1 * dN3T0 * dN5T * ddN2T * ddN1T0 -
       ddN5T * N1Tm * ddN2T0 * dN3T0 * dN6T * FP +
       FS * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN6T * dN3T * FP * N5Tm * dN2T0 * ddN1T0 +
       dN3T * FA * N5Tm * dN2T0 * ddN1T0 -
       N1Tm * ddN2T0 * IP * dN5T * ddN3T * dN0T0 +
       ddN5T * IA * dN2T * N3Tm * dN1T0 -
       dN2T * IP * ddN3T * dN0T0 * N5Tm * ddN1T0 +
       MidP1 * ddN3T0 * dN1T0 * dN5T * ddN2T +
       ddN5T * dN2T * N3Tm * IP * dN0T0 * ddN1T0 +
       ddN5T * IA * N1Tm * dN3T * dN2T0) /
      (ddN5T * dN3T * dN4T0 * N2Tm * ddN1T0 -
       ddN5T * dN4T * dN3T0 * N2Tm * ddN1T0 -
       ddN5T * N1Tm * ddN3T0 * dN4T * dN2T0 +
       dN4T * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       dN3T * dN4T0 * N5Tm * ddN2T * ddN1T0 +
       ddN5T * N1Tm * dN4T * ddN2T0 * dN3T0 -
       N1Tm * ddN4T0 * dN5T * ddN3T * dN2T0 -
       ddN4T * N3Tm * dN5T * dN2T0 * ddN1T0 +
       ddN5T * N4Tm * ddN2T0 * dN1T0 * dN3T -
       N1Tm * ddN3T0 * dN4T0 * dN5T * ddN2T -
       ddN5T * N1Tm * ddN2T0 * dN3T * dN4T0 -
       N4Tm * dN3T0 * dN5T * ddN2T * ddN1T0 +
       ddN5T * ddN3T0 * dN4T * dN1T0 * N2Tm +
       ddN4T * N3Tm * ddN2T0 * dN1T0 * dN5T +
       ddN5T * N1Tm * dN3T * ddN4T0 * dN2T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * dN2T0 -
       ddN4T * ddN2T0 * dN1T0 * dN3T * N5Tm -
       dN4T * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * dN4T * dN2T0 * ddN1T0 +
       ddN5T * dN2T * N3Tm * dN1T0 * ddN4T0 -
       N3Tm * dN1T0 * ddN4T0 * dN5T * ddN2T -
       N4Tm * ddN2T0 * dN1T0 * dN5T * ddN3T +
       dN1T0 * dN3T * ddN4T0 * N5Tm * ddN2T +
       N1Tm * ddN2T0 * dN4T0 * dN5T * ddN3T +
       dN2T * dN4T0 * ddN3T * N5Tm * ddN1T0 -
       ddN4T * ddN3T0 * dN1T0 * N2Tm * dN5T -
       ddN5T * dN1T0 * dN3T * ddN4T0 * N2Tm +
       N4Tm * dN5T * ddN3T * dN2T0 * ddN1T0 +
       dN2T * ddN4T * ddN3T0 * dN1T0 * N5Tm -
       ddN5T * dN2T * ddN3T0 * N4Tm * dN1T0 -
       ddN5T * dN2T * N1Tm * dN3T0 * ddN4T0 +
       ddN5T * dN2T * N1Tm * ddN3T0 * dN4T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * ddN2T -
       ddN5T * dN2T * N3Tm * dN4T0 * ddN1T0 -
       dN2T * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN4T * dN3T0 * N2Tm * dN5T * ddN1T0 -
       dN4T0 * N2Tm * dN5T * ddN3T * ddN1T0 +
       dN4T * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN3T0 * dN4T * dN1T0 * N5Tm * ddN2T -
       ddN5T * N4Tm * dN3T * dN2T0 * ddN1T0 +
       ddN4T * dN3T * N5Tm * dN2T0 * ddN1T0 +
       dN1T0 * ddN4T0 * N2Tm * dN5T * ddN3T -
       N1Tm * ddN4T * ddN2T0 * dN3T0 * dN5T +
       ddN5T * dN2T * N4Tm * dN3T0 * ddN1T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * ddN2T -
       ddN5T * N3Tm * dN4T * ddN2T0 * dN1T0 -
       dN2T * ddN4T * dN3T0 * N5Tm * ddN1T0 +
       N3Tm * dN4T0 * dN5T * ddN2T * ddN1T0);
  m_control_points[5] =
      -(N1Tm * dN4T * ddN2T0 * ddN3T * IS + FS * dN4T0 * N2Tm * ddN3T * ddN1T0 -
        N3Tm * dN1T0 * dN6T * ddN4T0 * FP * ddN2T +
        N1Tm * FS * ddN4T0 * ddN3T * dN2T0 -
        N1Tm * FS * ddN4T * ddN3T0 * dN2T0 -
        ddN0T0 * dN4T * dN1T0 * IP * N2Tm * ddN3T -
        N1Tm * dN4T * ddN2T0 * IP * ddN3T * dN0T0 +
        N1Tm * FS * ddN3T0 * dN4T0 * ddN2T +
        MidP1 * ddN3T0 * dN4T * dN1T0 * ddN2T -
        N3Tm * dN4T * IP * dN0T0 * ddN2T * ddN1T0 +
        N1Tm * ddN3T0 * dN4T * FA * dN2T0 + IA * dN2T * N1Tm * dN4T0 * ddN3T -
        ddN0T0 * N1Tm * dN4T * IP * dN3T0 * ddN2T -
        FS * ddN3T0 * N4Tm * dN1T0 * ddN2T + N1Tm * dN3T * ddN4T0 * ddN2T * IS +
        N3Tm * dN4T0 * dN6T * FP * ddN2T * ddN1T0 +
        ddN3T0 * N4Tm * dN1T0 * dN6T * FP * ddN2T -
        FS * ddN4T * N3Tm * ddN2T0 * dN1T0 +
        FS * N4Tm * ddN2T0 * dN1T0 * ddN3T + N4Tm * dN3T * FA * dN2T0 * ddN1T0 -
        FS * N4Tm * ddN3T * dN2T0 * ddN1T0 - dN2T * N3Tm * dN1T0 * ddN4T0 * FA +
        N3Tm * dN4T * ddN2T0 * dN1T0 * FA + N3Tm * dN4T * ddN2T * ddN1T0 * IS +
        dN2T * ddN0T0 * N1Tm * ddN4T * IP * dN3T0 -
        N1Tm * ddN4T * ddN2T0 * dN3T0 * dN6T * FP +
        dN4T * IP * N2Tm * ddN3T * dN0T0 * ddN1T0 +
        N1Tm * dN4T * ddN2T0 * ddN6T * dN3T0 * FP +
        N1Tm * ddN4T * ddN3T0 * dN6T * FP * dN2T0 -
        dN2T * N1Tm * ddN4T * ddN3T0 * IP * dN0T0 +
        N4Tm * IP * dN3T * dN0T0 * ddN2T * ddN1T0 -
        N4Tm * dN3T * ddN2T * ddN1T0 * IS -
        MidP1 * dN4T * ddN2T0 * dN1T0 * ddN3T -
        N1Tm * FS * dN3T0 * ddN4T0 * ddN2T -
        dN1T0 * ddN6T * dN3T * ddN4T0 * FP * N2Tm -
        dN2T * ddN4T * N3Tm * ddN1T0 * IS + N1Tm * ddN2T0 * dN3T * dN4T0 * FA -
        N1Tm * ddN3T0 * dN4T0 * dN6T * FP * ddN2T -
        IA * N3Tm * dN4T * dN1T0 * ddN2T +
        MidP1 * dN4T * ddN3T * dN2T0 * ddN1T0 +
        N3Tm * dN4T * ddN6T * FP * dN2T0 * ddN1T0 +
        ddN4T * N3Tm * ddN2T0 * dN1T0 * dN6T * FP -
        ddN0T0 * N4Tm * dN1T0 * IP * dN3T * ddN2T -
        dN3T * dN4T0 * N2Tm * FA * ddN1T0 - N3Tm * dN4T * FA * dN2T0 * ddN1T0 +
        N1Tm * ddN3T0 * dN4T * IP * dN0T0 * ddN2T -
        N1Tm * ddN4T * ddN2T0 * dN3T * IS -
        dN2T * ddN3T0 * N4Tm * dN1T0 * ddN6T * FP -
        dN2T * MidP1 * dN4T0 * ddN3T * ddN1T0 +
        IA * dN2T * ddN4T * N3Tm * dN1T0 +
        dN2T * ddN4T * N3Tm * IP * dN0T0 * ddN1T0 -
        N4Tm * ddN2T0 * dN1T0 * dN3T * FA -
        dN2T * ddN0T0 * ddN4T * N3Tm * dN1T0 * IP -
        ddN3T0 * dN4T * dN1T0 * N2Tm * FA -
        N4Tm * dN3T0 * dN6T * FP * ddN2T * ddN1T0 -
        N1Tm * dN6T * ddN4T0 * FP * ddN3T * dN2T0 +
        IA * N1Tm * dN4T * dN3T0 * ddN2T +
        N1Tm * dN3T0 * dN6T * ddN4T0 * FP * ddN2T -
        N1Tm * FS * ddN2T0 * dN4T0 * ddN3T +
        dN2T * N1Tm * ddN3T0 * ddN6T * dN4T0 * FP +
        ddN4T * dN3T0 * dN6T * FP * N2Tm * ddN1T0 +
        dN4T * dN3T0 * N2Tm * FA * ddN1T0 + ddN4T * dN3T * N2Tm * ddN1T0 * IS +
        ddN4T * MidP1 * ddN2T0 * dN1T0 * dN3T -
        N1Tm * ddN3T0 * dN4T * ddN6T * FP * dN2T0 -
        dN4T0 * dN6T * FP * N2Tm * ddN3T * ddN1T0 +
        IA * N4Tm * dN1T0 * dN3T * ddN2T - dN4T * N2Tm * ddN3T * ddN1T0 * IS -
        N1Tm * dN3T * ddN4T0 * FA * dN2T0 - dN2T * N4Tm * dN3T0 * FA * ddN1T0 -
        dN2T * N3Tm * ddN6T * dN4T0 * FP * ddN1T0 +
        IA * N1Tm * ddN4T * dN3T * dN2T0 -
        ddN0T0 * N1Tm * ddN4T * IP * dN3T * dN2T0 +
        dN2T * ddN4T * MidP1 * dN3T0 * ddN1T0 -
        dN2T * N4Tm * IP * ddN3T * dN0T0 * ddN1T0 +
        dN2T * N1Tm * IP * ddN4T0 * ddN3T * dN0T0 +
        dN2T * MidP1 * dN1T0 * ddN4T0 * ddN3T +
        N4Tm * ddN2T0 * dN1T0 * ddN6T * dN3T * FP -
        N1Tm * ddN2T0 * ddN6T * dN3T * dN4T0 * FP -
        IA * N1Tm * dN4T * ddN3T * dN2T0 - FS * ddN4T * dN3T0 * N2Tm * ddN1T0 -
        dN2T * N1Tm * ddN3T0 * dN4T0 * FA + FS * N4Tm * dN3T0 * ddN2T * ddN1T0 +
        N1Tm * ddN4T * ddN2T0 * IP * dN3T * dN0T0 +
        dN2T * N3Tm * dN1T0 * ddN6T * ddN4T0 * FP -
        N1Tm * dN4T * ddN2T0 * dN3T0 * FA +
        ddN0T0 * N1Tm * IP * dN3T * dN4T0 * ddN2T -
        N1Tm * ddN3T0 * dN4T * ddN2T * IS +
        dN2T * ddN0T0 * N4Tm * dN1T0 * IP * ddN3T +
        N1Tm * FS * ddN4T * ddN2T0 * dN3T0 +
        dN1T0 * dN6T * ddN4T0 * FP * N2Tm * ddN3T +
        ddN3T0 * dN4T * dN1T0 * ddN6T * FP * N2Tm +
        FS * ddN4T * N3Tm * dN2T0 * ddN1T0 -
        dN2T * ddN4T * MidP1 * ddN3T0 * dN1T0 -
        N4Tm * ddN6T * dN3T * FP * dN2T0 * ddN1T0 +
        IA * dN4T * dN1T0 * N2Tm * ddN3T -
        dN2T * N1Tm * ddN6T * dN3T0 * ddN4T0 * FP +
        MidP1 * dN3T * dN4T0 * ddN2T * ddN1T0 +
        dN2T * N4Tm * ddN3T * ddN1T0 * IS -
        dN4T * ddN6T * dN3T0 * FP * N2Tm * ddN1T0 -
        dN2T * ddN0T0 * N1Tm * IP * dN4T0 * ddN3T +
        FS * ddN4T * ddN3T0 * dN1T0 * N2Tm - IA * dN2T * N1Tm * ddN4T * dN3T0 +
        N1Tm * ddN2T0 * dN4T0 * dN6T * FP * ddN3T +
        dN1T0 * dN3T * ddN4T0 * N2Tm * FA -
        ddN4T * ddN3T0 * dN1T0 * dN6T * FP * N2Tm -
        ddN4T * N3Tm * dN6T * FP * dN2T0 * ddN1T0 -
        N4Tm * ddN2T0 * dN1T0 * dN6T * FP * ddN3T -
        IA * dN2T * N4Tm * dN1T0 * ddN3T -
        MidP1 * dN4T * dN3T0 * ddN2T * ddN1T0 +
        ddN0T0 * N1Tm * dN4T * IP * ddN3T * dN2T0 +
        dN2T * N1Tm * ddN4T * ddN3T0 * IS - dN2T * N1Tm * ddN4T0 * ddN3T * IS -
        MidP1 * dN1T0 * dN3T * ddN4T0 * ddN2T +
        dN2T * N3Tm * dN4T0 * FA * ddN1T0 + FS * N3Tm * dN1T0 * ddN4T0 * ddN2T +
        ddN0T0 * ddN4T * dN1T0 * IP * dN3T * N2Tm -
        N3Tm * dN4T * ddN2T0 * dN1T0 * ddN6T * FP -
        FS * N3Tm * dN4T0 * ddN2T * ddN1T0 -
        ddN4T * MidP1 * dN3T * dN2T0 * ddN1T0 +
        dN2T * N4Tm * ddN6T * dN3T0 * FP * ddN1T0 +
        dN2T * N1Tm * dN3T0 * ddN4T0 * FA -
        ddN4T * IP * dN3T * N2Tm * dN0T0 * ddN1T0 +
        ddN6T * dN3T * dN4T0 * FP * N2Tm * ddN1T0 -
        N1Tm * IP * dN3T * ddN4T0 * dN0T0 * ddN2T -
        IA * N1Tm * dN3T * dN4T0 * ddN2T +
        ddN0T0 * N3Tm * dN4T * dN1T0 * IP * ddN2T +
        N4Tm * dN6T * FP * ddN3T * dN2T0 * ddN1T0 +
        dN2T * ddN3T0 * N4Tm * dN1T0 * FA - FS * dN1T0 * ddN4T0 * N2Tm * ddN3T +
        N1Tm * ddN6T * dN3T * ddN4T0 * FP * dN2T0 -
        IA * ddN4T * dN1T0 * dN3T * N2Tm) /
      (ddN5T * dN3T * dN4T0 * N2Tm * ddN1T0 -
       ddN5T * dN4T * dN3T0 * N2Tm * ddN1T0 -
       ddN5T * N1Tm * ddN3T0 * dN4T * dN2T0 +
       dN4T * dN3T0 * N5Tm * ddN2T * ddN1T0 -
       dN3T * dN4T0 * N5Tm * ddN2T * ddN1T0 +
       ddN5T * N1Tm * dN4T * ddN2T0 * dN3T0 -
       N1Tm * ddN4T0 * dN5T * ddN3T * dN2T0 -
       ddN4T * N3Tm * dN5T * dN2T0 * ddN1T0 +
       ddN5T * N4Tm * ddN2T0 * dN1T0 * dN3T -
       N1Tm * ddN3T0 * dN4T0 * dN5T * ddN2T -
       ddN5T * N1Tm * ddN2T0 * dN3T * dN4T0 -
       N4Tm * dN3T0 * dN5T * ddN2T * ddN1T0 +
       ddN5T * ddN3T0 * dN4T * dN1T0 * N2Tm +
       ddN4T * N3Tm * ddN2T0 * dN1T0 * dN5T +
       ddN5T * N1Tm * dN3T * ddN4T0 * dN2T0 +
       N1Tm * ddN4T * ddN3T0 * dN5T * dN2T0 -
       ddN4T * ddN2T0 * dN1T0 * dN3T * N5Tm -
       dN4T * ddN3T * N5Tm * dN2T0 * ddN1T0 +
       ddN5T * N3Tm * dN4T * dN2T0 * ddN1T0 +
       ddN5T * dN2T * N3Tm * dN1T0 * ddN4T0 -
       N3Tm * dN1T0 * ddN4T0 * dN5T * ddN2T -
       N4Tm * ddN2T0 * dN1T0 * dN5T * ddN3T +
       dN1T0 * dN3T * ddN4T0 * N5Tm * ddN2T +
       N1Tm * ddN2T0 * dN4T0 * dN5T * ddN3T +
       dN2T * dN4T0 * ddN3T * N5Tm * ddN1T0 -
       ddN4T * ddN3T0 * dN1T0 * N2Tm * dN5T -
       ddN5T * dN1T0 * dN3T * ddN4T0 * N2Tm +
       N4Tm * dN5T * ddN3T * dN2T0 * ddN1T0 +
       dN2T * ddN4T * ddN3T0 * dN1T0 * N5Tm -
       ddN5T * dN2T * ddN3T0 * N4Tm * dN1T0 -
       ddN5T * dN2T * N1Tm * dN3T0 * ddN4T0 +
       ddN5T * dN2T * N1Tm * ddN3T0 * dN4T0 +
       N1Tm * dN3T0 * ddN4T0 * dN5T * ddN2T -
       ddN5T * dN2T * N3Tm * dN4T0 * ddN1T0 -
       dN2T * dN1T0 * ddN4T0 * ddN3T * N5Tm +
       ddN4T * dN3T0 * N2Tm * dN5T * ddN1T0 -
       dN4T0 * N2Tm * dN5T * ddN3T * ddN1T0 +
       dN4T * ddN2T0 * dN1T0 * ddN3T * N5Tm -
       ddN3T0 * dN4T * dN1T0 * N5Tm * ddN2T -
       ddN5T * N4Tm * dN3T * dN2T0 * ddN1T0 +
       ddN4T * dN3T * N5Tm * dN2T0 * ddN1T0 +
       dN1T0 * ddN4T0 * N2Tm * dN5T * ddN3T -
       N1Tm * ddN4T * ddN2T0 * dN3T0 * dN5T +
       ddN5T * dN2T * N4Tm * dN3T0 * ddN1T0 +
       ddN3T0 * N4Tm * dN1T0 * dN5T * ddN2T -
       ddN5T * N3Tm * dN4T * ddN2T0 * dN1T0 -
       dN2T * ddN4T * dN3T0 * N5Tm * ddN1T0 +
       N3Tm * dN4T0 * dN5T * ddN2T * ddN1T0);
  m_control_points[6] = FP;
  return;
}

void BSplinesFoot::ComputeControlPointFrom4DataPoint(void) {
  ComputeBasisFunctions(0);
  vector<double> dNi5T0 = m_basis_functions_derivative;
  vector<double> ddNi5T0 = m_basis_functions_sec_derivative;

  ComputeBasisFunctions(m_ToMP[0] / m_FT);
  vector<double> Ni5Tm1 = m_basis_functions[m_degree];

  ComputeBasisFunctions(m_ToMP[1] / m_FT);
  vector<double> Ni5Tm2 = m_basis_functions[m_degree];

  ComputeBasisFunctions(1.0);
  vector<double> dNi5T = m_basis_functions_derivative;
  vector<double> ddNi5T = m_basis_functions_sec_derivative;

  double IP = m_IP;
  double IS = m_IS;
  double IA = m_IA;
  double FP = m_FP;
  double FS = m_FS;
  double FA = m_FA;
  double MidP1 = m_MP[0];
  double MidP2 = m_MP[1];

  double N1Tm1 = Ni5Tm1[1];
  double N2Tm1 = Ni5Tm1[2];
  double N3Tm1 = Ni5Tm1[3];
  double N4Tm1 = Ni5Tm1[4];
  double N5Tm1 = Ni5Tm1[5];

  double N2Tm2 = Ni5Tm2[2];
  double N3Tm2 = Ni5Tm2[3];
  double N4Tm2 = Ni5Tm2[4];
  double N5Tm2 = Ni5Tm2[5];
  double N6Tm2 = Ni5Tm2[6];

  double dN0T0 = dNi5T0[0];
  double dN1T0 = dNi5T0[1];
  double dN2T0 = dNi5T0[2];
  double dN3T0 = dNi5T0[3];
  double dN4T0 = dNi5T0[4];

  double dN3T = dNi5T[3];
  double dN4T = dNi5T[4];
  double dN5T = dNi5T[5];
  double dN6T = dNi5T[6];
  double dN7T = dNi5T[7];

  double ddN0T0 = ddNi5T0[0];
  double ddN1T0 = ddNi5T0[1];
  double ddN2T0 = ddNi5T0[2];
  double ddN3T0 = ddNi5T0[3];
  double ddN4T0 = ddNi5T0[4];

  double ddN3T = ddNi5T[3];
  double ddN4T = ddNi5T[4];
  double ddN5T = ddNi5T[5];
  double ddN6T = ddNi5T[6];
  double ddN7T = ddNi5T[7];

  m_control_points[0] = IP;
  m_control_points[1] = -1.0 /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T) *
                        (N3Tm1 * IA * dN6T * ddN5T * N2Tm2 * dN4T0 +
                         ddN3T0 * N2Tm1 * dN6T * IP * dN0T0 * ddN5T * N4Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * FA * dN4T0 +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * IP * ddN3T * dN0T0 -
                         N3Tm1 * IS * dN6T * ddN4T0 * ddN5T * N2Tm2 +
                         N5Tm1 * ddN2T0 * N6Tm2 * ddN3T * dN7T * FP * dN4T0 +
                         N6Tm2 * N2Tm1 * IS * ddN4T0 * dN5T * ddN3T +
                         N3Tm1 * dN6T * ddN4T0 * IP * dN0T0 * ddN5T * N2Tm2 -
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * MidP1 * dN4T0 -
                         N4Tm1 * N6Tm2 * IA * dN3T * dN2T0 * ddN5T +
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N4Tm2 * MidP1 -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * IP * dN0T0 -
                         N2Tm1 * N5Tm2 * IS * dN6T * ddN4T0 * ddN3T +
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * IS * ddN5T -
                         N5Tm1 * dN6T * ddN4T0 * dN2T0 * ddN3T * MidP2 -
                         dN3T0 * ddN7T * N2Tm1 * N5Tm2 * dN6T * ddN4T0 * FP +
                         dN3T0 * N5Tm1 * ddN2T0 * dN6T * FA * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN2T0 * MidP2 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * MidP2 * dN4T0 +
                         N2Tm1 * N5Tm2 * IA * dN6T * ddN3T * dN4T0 -
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * MidP1 +
                         ddN3T0 * ddN6T * dN5T * MidP1 * N2Tm2 * dN4T0 -
                         dN3T0 * N5Tm1 * ddN6T * ddN4T0 * dN7T * FP * N2Tm2 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * IS * ddN5T +
                         dN3T0 * N6Tm2 * N2Tm1 * IA * dN5T * ddN4T -
                         ddN2T0 * N4Tm1 * ddN6T * IP * dN5T * dN0T0 * N3Tm2 -
                         N5Tm1 * IA * dN6T * ddN3T * N2Tm2 * dN4T0 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * MidP2 * dN4T0 +
                         dN3T0 * ddN7T * N5Tm1 * dN6T * ddN4T0 * FP * N2Tm2 +
                         ddN7T * N3Tm1 * N5Tm2 * dN6T * ddN4T0 * dN2T0 * FP -
                         N6Tm2 * N2Tm1 * ddN4T0 * IP * dN5T * ddN3T * dN0T0 +
                         N3Tm1 * dN6T * IP * dN2T0 * ddN5T * N4Tm2 * ddN0T0 -
                         N3Tm1 * IA * dN6T * dN2T0 * ddN5T * N4Tm2 -
                         ddN3T0 * ddN7T * N6Tm2 * N2Tm1 * dN5T * FP * dN4T0 -
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * IS +
                         ddN3T0 * N2Tm1 * dN6T * ddN5T * MidP2 * dN4T0 -
                         N3Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN2T0 * dN7T * FP +
                         N5Tm1 * dN6T * IP * dN2T0 * N3Tm2 * ddN0T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN0T0 -
                         ddN3T0 * N4Tm1 * ddN6T * IS * dN5T * N2Tm2 -
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IA * dN2T0 +
                         ddN3T0 * N4Tm1 * N6Tm2 * dN2T0 * FS * ddN5T +
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * IA -
                         N5Tm1 * ddN6T * dN3T * IP * N2Tm2 * ddN0T0 * dN4T0 -
                         N5Tm1 * ddN2T0 * dN6T * FA * N3Tm2 * dN4T0 -
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * MidP1 * ddN4T +
                         N3Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN2T0 * FS -
                         N5Tm1 * IA * dN6T * dN2T0 * N3Tm2 * ddN4T -
                         N4Tm1 * dN6T * IP * dN2T0 * N3Tm2 * ddN5T * ddN0T0 -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * IP * ddN0T0 +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * IP * dN0T0 * ddN4T +
                         N3Tm1 * ddN6T * IA * dN2T0 * dN5T * N4Tm2 -
                         dN3T0 * ddN6T * ddN4T0 * dN5T * MidP1 * N2Tm2 +
                         N3Tm1 * N5Tm2 * IA * dN6T * dN2T0 * ddN4T -
                         N5Tm1 * ddN6T * IA * dN3T * dN2T0 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * IP * dN2T0 * ddN3T * ddN0T0 -
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * MidP1 +
                         N2Tm1 * IS * dN6T * ddN4T0 * N3Tm2 * ddN5T -
                         dN3T0 * ddN2T0 * N4Tm1 * ddN6T * dN5T * MidP2 -
                         ddN3T0 * N2Tm1 * ddN6T * N5Tm2 * dN7T * FP * dN4T0 +
                         N3Tm1 * N6Tm2 * IP * dN2T0 * dN5T * ddN0T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * IP * ddN3T * dN0T0 -
                         N3Tm1 * N5Tm2 * dN6T * IP * dN2T0 * ddN0T0 * ddN4T -
                         N3Tm1 * ddN2T0 * N5Tm2 * IS * dN6T * ddN4T -
                         N3Tm1 * N5Tm2 * dN6T * ddN4T0 * dN2T0 * FA -
                         dN3T0 * N5Tm1 * dN4T * ddN2T0 * N6Tm2 * FA +
                         ddN3T0 * N5Tm1 * dN4T * N6Tm2 * dN2T0 * FA +
                         ddN3T0 * N2Tm1 * ddN6T * IS * dN5T * N4Tm2 -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * IP * dN0T0 -
                         ddN3T0 * N4Tm1 * N6Tm2 * dN2T0 * dN5T * FA -
                         N3Tm1 * ddN2T0 * N6Tm2 * IP * dN5T * dN0T0 * ddN4T -
                         N5Tm1 * ddN6T * dN3T * IS * ddN4T0 * N2Tm2 +
                         ddN3T0 * N4Tm1 * N5Tm2 * dN6T * dN2T0 * FA +
                         N5Tm1 * dN6T * IP * ddN3T * N2Tm2 * ddN0T0 * dN4T0 -
                         dN3T0 * ddN7T * ddN2T0 * N4Tm1 * N6Tm2 * dN5T * FP +
                         N5Tm1 * ddN2T0 * dN6T * IP * ddN3T * dN0T0 * N4Tm2 -
                         N5Tm1 * ddN2T0 * ddN6T * dN7T * FP * N3Tm2 * dN4T0 +
                         N5Tm1 * IS * dN6T * ddN4T0 * ddN3T * N2Tm2 +
                         ddN7T * N5Tm1 * N6Tm2 * dN3T * ddN4T0 * dN2T0 * FP -
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * IS +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * IP * dN0T0 * ddN5T +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN2T0 * MidP2 -
                         ddN2T0 * N6Tm2 * dN3T * ddN5T * MidP1 * dN4T0 +
                         N4Tm1 * ddN6T * IP * dN2T0 * dN5T * N3Tm2 * ddN0T0 +
                         dN3T0 * dN6T * ddN4T0 * ddN5T * MidP1 * N2Tm2 +
                         dN3T0 * N2Tm1 * IA * dN6T * ddN5T * N4Tm2 +
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * IS * N4Tm2 +
                         N3Tm1 * dN6T * ddN4T0 * dN2T0 * ddN5T * MidP2 -
                         dN3T0 * N4Tm1 * IA * dN6T * ddN5T * N2Tm2 -
                         ddN3T0 * N5Tm1 * ddN6T * dN2T0 * dN7T * FP * N4Tm2 -
                         N5Tm1 * dN4T * N6Tm2 * IA * dN2T0 * ddN3T -
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * IA * ddN5T -
                         N2Tm1 * ddN6T * IS * ddN4T0 * dN5T * N3Tm2 -
                         dN3T0 * N2Tm1 * N5Tm2 * IA * dN6T * ddN4T +
                         N5Tm1 * IA * dN6T * dN2T0 * ddN3T * N4Tm2 +
                         ddN3T0 * ddN7T * N5Tm1 * dN6T * dN2T0 * FP * N4Tm2 +
                         dN3T0 * N5Tm1 * IA * dN6T * N2Tm2 * ddN4T +
                         N2Tm1 * dN6T * IP * N3Tm2 * ddN5T * ddN0T0 * dN4T0 -
                         N4Tm1 * N5Tm2 * IA * dN6T * dN2T0 * ddN3T -
                         ddN3T0 * N2Tm1 * IS * dN6T * ddN5T * N4Tm2 +
                         dN3T0 * N2Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN7T * FP -
                         ddN3T0 * N2Tm1 * ddN6T * IP * dN5T * dN0T0 * N4Tm2 -
                         N5Tm1 * ddN2T0 * dN6T * IP * dN0T0 * N3Tm2 * ddN4T -
                         ddN3T0 * N5Tm1 * ddN6T * FS * N2Tm2 * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * ddN4T0 * dN2T0 * FA -
                         ddN2T0 * N4Tm1 * N6Tm2 * IS * dN5T * ddN3T -
                         N2Tm1 * N5Tm2 * dN6T * IP * ddN3T * ddN0T0 * dN4T0 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * IP * dN0T0 * N3Tm2 +
                         N6Tm2 * N2Tm1 * IP * dN5T * ddN3T * ddN0T0 * dN4T0 +
                         N3Tm1 * ddN6T * IS * ddN4T0 * dN5T * N2Tm2 -
                         ddN3T0 * N4Tm1 * N6Tm2 * dN2T0 * dN7T * FP * ddN5T -
                         ddN2T0 * ddN6T * dN5T * N3Tm2 * MidP1 * dN4T0 -
                         ddN3T0 * N4Tm1 * ddN6T * N5Tm2 * dN2T0 * FS +
                         ddN3T0 * N2Tm1 * ddN6T * N5Tm2 * FS * dN4T0 -
                         N5Tm1 * ddN6T * ddN4T0 * dN2T0 * FS * N3Tm2 -
                         N3Tm1 * ddN6T * ddN4T0 * IP * dN5T * dN0T0 * N2Tm2 -
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * IS * ddN4T +
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * IS * ddN3T -
                         ddN3T0 * N5Tm1 * IS * dN6T * N2Tm2 * ddN4T +
                         ddN2T0 * N4Tm1 * dN6T * IP * dN0T0 * N3Tm2 * ddN5T -
                         N3Tm1 * dN6T * IP * ddN5T * N2Tm2 * ddN0T0 * dN4T0 -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * IP * dN0T0 * N4Tm2 -
                         N3Tm1 * ddN6T * IP * dN2T0 * dN5T * N4Tm2 * ddN0T0 -
                         N5Tm1 * ddN2T0 * N6Tm2 * ddN3T * FS * dN4T0 +
                         dN3T0 * ddN7T * ddN2T0 * N4Tm1 * N5Tm2 * dN6T * FP +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * IP * ddN5T * ddN0T0 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN7T * FP * ddN5T * dN4T0 +
                         dN3T0 * ddN7T * N6Tm2 * N2Tm1 * ddN4T0 * dN5T * FP +
                         ddN3T0 * N4Tm1 * IS * dN6T * ddN5T * N2Tm2 +
                         N2Tm1 * ddN6T * IA * dN5T * N3Tm2 * dN4T0 -
                         N6Tm2 * N2Tm1 * dN3T * IP * ddN5T * ddN0T0 * dN4T0 -
                         dN3T0 * N2Tm1 * dN6T * IP * ddN5T * N4Tm2 * ddN0T0 -
                         ddN3T0 * N5Tm1 * N6Tm2 * dN2T0 * FS * ddN4T +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IP * dN2T0 * ddN0T0 +
                         dN3T0 * ddN7T * N5Tm1 * dN4T * ddN2T0 * N6Tm2 * FP -
                         N5Tm1 * dN6T * ddN4T0 * IP * ddN3T * dN0T0 * N2Tm2 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * IP * N2Tm2 * ddN0T0 +
                         N3Tm1 * ddN2T0 * IS * dN6T * ddN5T * N4Tm2 +
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N3Tm2 * MidP1 +
                         ddN3T0 * N5Tm1 * ddN6T * dN7T * FP * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * FA * dN4T0 -
                         N2Tm1 * ddN6T * IP * dN5T * N3Tm2 * ddN0T0 * dN4T0 +
                         N4Tm1 * IA * dN6T * dN2T0 * N3Tm2 * ddN5T +
                         N4Tm1 * ddN6T * N5Tm2 * IA * dN3T * dN2T0 +
                         ddN3T0 * N4Tm1 * ddN6T * N5Tm2 * dN2T0 * dN7T * FP -
                         N6Tm2 * N2Tm1 * IA * dN5T * ddN3T * dN4T0 -
                         N3Tm1 * ddN2T0 * ddN6T * N5Tm2 * FS * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * IP * dN2T0 * ddN0T0 * ddN4T -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * FA * dN4T0 -
                         dN3T0 * N6Tm2 * N2Tm1 * ddN4T0 * dN7T * FP * ddN5T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * IP * dN0T0 -
                         ddN3T0 * N6Tm2 * N2Tm1 * IS * dN5T * ddN4T -
                         N2Tm1 * IA * dN6T * N3Tm2 * ddN5T * dN4T0 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * IP * dN0T0 * N2Tm2 +
                         ddN2T0 * N4Tm1 * ddN6T * IS * dN5T * N3Tm2 -
                         dN3T0 * N2Tm1 * ddN6T * N5Tm2 * ddN4T0 * FS -
                         ddN3T0 * N4Tm1 * dN6T * dN2T0 * ddN5T * MidP2 -
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * MidP1 -
                         dN3T0 * N5Tm1 * ddN2T0 * N6Tm2 * dN7T * FP * ddN4T -
                         N5Tm1 * dN6T * IP * dN2T0 * ddN3T * N4Tm2 * ddN0T0 -
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * IS * ddN5T -
                         N5Tm1 * N6Tm2 * ddN4T0 * dN2T0 * ddN3T * dN7T * FP -
                         dN3T0 * N5Tm1 * dN4T * ddN6T * IA * N2Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * IP * dN0T0 * N2Tm2 * ddN4T +
                         ddN3T0 * N5Tm1 * ddN6T * dN2T0 * FS * N4Tm2 -
                         ddN7T * N5Tm1 * dN6T * ddN4T0 * dN2T0 * FP * N3Tm2 +
                         dN3T0 * N4Tm1 * ddN6T * IA * dN5T * N2Tm2 -
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * MidP1 * ddN4T -
                         dN3T0 * ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN7T * FP +
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * MidP1 * dN4T0 +
                         N5Tm1 * dN4T * ddN6T * IA * dN2T0 * N3Tm2 -
                         dN3T0 * N5Tm1 * ddN2T0 * dN6T * MidP2 * ddN4T -
                         ddN3T0 * N5Tm1 * dN6T * dN2T0 * FA * N4Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * IP * dN5T * dN0T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * IP * dN5T * dN0T0 * N3Tm2 +
                         N5Tm1 * N6Tm2 * IA * dN3T * dN2T0 * ddN4T +
                         ddN3T0 * ddN7T * N4Tm1 * N6Tm2 * dN2T0 * dN5T * FP +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN7T * FP * ddN5T * dN4T0 +
                         dN3T0 * N5Tm1 * ddN2T0 * N6Tm2 * FS * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN2T0 * dN5T * MidP2 -
                         ddN7T * N3Tm1 * ddN2T0 * N5Tm2 * dN6T * FP * dN4T0 +
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * ddN5T * MidP1 -
                         ddN3T0 * N4Tm1 * dN6T * IP * dN0T0 * ddN5T * N2Tm2 +
                         N5Tm1 * ddN2T0 * ddN6T * FS * N3Tm2 * dN4T0 +
                         ddN2T0 * N4Tm1 * N5Tm2 * IS * dN6T * ddN3T -
                         ddN3T0 * ddN7T * N5Tm1 * dN4T * N6Tm2 * dN2T0 * FP +
                         ddN3T0 * dN6T * dN2T0 * ddN5T * N4Tm2 * MidP1 +
                         N3Tm1 * ddN6T * IP * dN5T * N2Tm2 * ddN0T0 * dN4T0 +
                         ddN7T * N5Tm1 * ddN2T0 * dN6T * FP * N3Tm2 * dN4T0 -
                         N4Tm1 * ddN6T * IA * dN2T0 * dN5T * N3Tm2 -
                         N3Tm1 * N6Tm2 * IA * dN2T0 * dN5T * ddN4T -
                         N2Tm1 * ddN6T * N5Tm2 * IA * dN3T * dN4T0 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN4T0 * FA +
                         N3Tm1 * ddN2T0 * N6Tm2 * FS * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * IP * ddN5T * N2Tm2 * ddN0T0 +
                         N3Tm1 * dN4T * N6Tm2 * IA * dN2T0 * ddN5T +
                         N5Tm1 * ddN2T0 * IS * dN6T * N3Tm2 * ddN4T +
                         N4Tm1 * N6Tm2 * dN3T * IP * dN2T0 * ddN5T * ddN0T0 +
                         dN3T0 * N5Tm1 * ddN2T0 * ddN6T * dN7T * FP * N4Tm2 +
                         ddN2T0 * dN6T * N3Tm2 * ddN5T * MidP1 * dN4T0 +
                         N3Tm1 * N6Tm2 * ddN4T0 * dN2T0 * dN7T * FP * ddN5T -
                         dN3T0 * N2Tm1 * ddN6T * IA * dN5T * N4Tm2 -
                         dN3T0 * N6Tm2 * N2Tm1 * IP * dN5T * ddN0T0 * ddN4T +
                         dN3T0 * ddN2T0 * N4Tm1 * dN6T * ddN5T * MidP2 +
                         N6Tm2 * N2Tm1 * IA * dN3T * ddN5T * dN4T0 -
                         N5Tm1 * ddN2T0 * IS * dN6T * ddN3T * N4Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * IP * ddN0T0 * ddN4T -
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * ddN5T * MidP1 +
                         ddN7T * N3Tm1 * ddN2T0 * N6Tm2 * dN5T * FP * dN4T0 -
                         N6Tm2 * N2Tm1 * dN3T * IS * ddN4T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * ddN4T0 * dN5T * FA -
                         ddN2T0 * N4Tm1 * IS * dN6T * N3Tm2 * ddN5T -
                         N3Tm1 * ddN2T0 * dN6T * IP * dN0T0 * ddN5T * N4Tm2 -
                         dN3T0 * ddN7T * N5Tm1 * ddN2T0 * dN6T * FP * N4Tm2 +
                         dN3T0 * N5Tm1 * ddN6T * ddN4T0 * FS * N2Tm2 -
                         N3Tm1 * N6Tm2 * ddN4T0 * dN2T0 * FS * ddN5T +
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * IS -
                         N2Tm1 * dN6T * ddN4T0 * IP * dN0T0 * N3Tm2 * ddN5T +
                         N5Tm1 * ddN6T * dN3T * IP * dN2T0 * N4Tm2 * ddN0T0 +
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * MidP1 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN2T0 * ddN0T0 +
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * MidP1 * ddN4T -
                         ddN7T * N5Tm1 * ddN2T0 * N6Tm2 * dN3T * FP * dN4T0 +
                         ddN3T0 * N4Tm1 * ddN6T * IP * dN5T * dN0T0 * N2Tm2 +
                         ddN3T0 * N5Tm1 * N6Tm2 * dN2T0 * dN7T * FP * ddN4T -
                         dN3T0 * ddN2T0 * N4Tm1 * N5Tm2 * dN6T * FA -
                         ddN3T0 * ddN7T * N5Tm1 * dN6T * FP * N2Tm2 * dN4T0 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * IP * dN0T0 * ddN5T -
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N4Tm2 * MidP1 -
                         dN3T0 * N5Tm1 * dN6T * ddN4T0 * FA * N2Tm2 -
                         N5Tm1 * dN4T * ddN6T * IP * dN2T0 * N3Tm2 * ddN0T0 -
                         dN3T0 * N5Tm1 * ddN2T0 * ddN6T * FS * N4Tm2 -
                         ddN3T0 * ddN7T * N4Tm1 * N5Tm2 * dN6T * dN2T0 * FP +
                         N3Tm1 * N6Tm2 * ddN4T0 * dN2T0 * dN5T * FA +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * IS * ddN4T0 -
                         dN3T0 * N5Tm1 * dN6T * IP * N2Tm2 * ddN0T0 * ddN4T -
                         ddN3T0 * dN6T * ddN5T * MidP1 * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * IP * dN5T * N2Tm2 * ddN0T0 +
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * MidP1 * dN4T0 +
                         ddN3T0 * N2Tm1 * N5Tm2 * IS * dN6T * ddN4T +
                         N4Tm1 * N5Tm2 * dN6T * IP * dN2T0 * ddN3T * ddN0T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN2T0 * ddN6T * MidP2 +
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * MidP1 * ddN4T -
                         N4Tm1 * N6Tm2 * IP * dN2T0 * dN5T * ddN3T * ddN0T0 -
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * IS * N3Tm2 -
                         ddN3T0 * N6Tm2 * N2Tm1 * FS * ddN5T * dN4T0 +
                         dN3T0 * N2Tm1 * ddN6T * ddN4T0 * dN5T * MidP2 +
                         dN3T0 * N6Tm2 * N2Tm1 * ddN4T0 * FS * ddN5T -
                         N3Tm1 * ddN2T0 * ddN6T * IS * dN5T * N4Tm2 +
                         dN3T0 * N2Tm1 * ddN6T * IP * dN5T * N4Tm2 * ddN0T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * IP * dN0T0 * N2Tm2 +
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * MidP1 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * IP * ddN0T0 * dN4T0 +
                         dN3T0 * ddN2T0 * N4Tm1 * N6Tm2 * dN7T * FP * ddN5T +
                         N5Tm1 * ddN2T0 * dN6T * ddN3T * MidP2 * dN4T0 -
                         dN3T0 * ddN2T0 * N4Tm1 * N6Tm2 * FS * ddN5T +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * FA * dN4T0 -
                         dN6T * ddN4T0 * dN2T0 * N3Tm2 * ddN5T * MidP1 -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * IP * ddN3T * dN0T0 +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * FA * dN4T0 +
                         N5Tm1 * N6Tm2 * ddN4T0 * dN2T0 * ddN3T * FS -
                         N3Tm1 * ddN6T * IA * dN5T * N2Tm2 * dN4T0 +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * IP * dN0T0 * ddN5T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * IP * dN0T0 * ddN4T +
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * ddN5T * MidP1 -
                         dN3T0 * ddN2T0 * dN6T * ddN5T * N4Tm2 * MidP1 +
                         N5Tm1 * dN6T * ddN4T0 * dN2T0 * FA * N3Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * IP * dN0T0 * ddN4T -
                         dN3T0 * N2Tm1 * dN6T * ddN4T0 * ddN5T * MidP2 +
                         ddN3T0 * N5Tm1 * dN6T * FA * N2Tm2 * dN4T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * IP * dN5T * dN0T0 * ddN4T +
                         N5Tm1 * ddN6T * IA * dN3T * N2Tm2 * dN4T0 -
                         N3Tm1 * dN4T * N6Tm2 * IP * dN2T0 * ddN5T * ddN0T0 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN2T0 * MidP2 +
                         ddN2T0 * N4Tm1 * N6Tm2 * IP * dN5T * ddN3T * dN0T0 +
                         N5Tm1 * ddN6T * ddN4T0 * dN2T0 * dN7T * FP * N3Tm2 -
                         ddN7T * N3Tm1 * N6Tm2 * ddN4T0 * dN2T0 * dN5T * FP +
                         N3Tm1 * ddN2T0 * ddN6T * N5Tm2 * dN7T * FP * dN4T0 +
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * IS * N2Tm2 -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * IP * dN0T0 * ddN5T +
                         dN3T0 * ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * FS +
                         dN3T0 * ddN2T0 * N4Tm1 * N6Tm2 * dN5T * FA +
                         N4Tm1 * N6Tm2 * IA * dN2T0 * dN5T * ddN3T -
                         N3Tm1 * ddN6T * ddN4T0 * dN2T0 * dN5T * MidP2 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * MidP2 * dN4T0 +
                         ddN3T0 * ddN7T * N2Tm1 * N5Tm2 * dN6T * FP * dN4T0 -
                         N3Tm1 * ddN2T0 * dN6T * ddN5T * MidP2 * dN4T0 +
                         N3Tm1 * ddN2T0 * N6Tm2 * IS * dN5T * ddN4T);
  m_control_points[2] = (N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * MidP2 -
                         N4Tm1 * dN6T * IP * dN1T0 * N3Tm2 * ddN5T * ddN0T0 -
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IA * dN1T0 -
                         N3Tm1 * ddN6T * IS * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * IS * ddN3T * ddN1T0 +
                         N5Tm2 * IA * dN6T * ddN3T * N1Tm1 * dN4T0 -
                         dN3T0 * N6Tm2 * IP * dN5T * N1Tm1 * ddN0T0 * ddN4T +
                         ddN3T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * FA +
                         dN3T0 * N4Tm1 * N6Tm2 * dN7T * ddN1T0 * FP * ddN5T +
                         ddN3T0 * N5Tm2 * IS * dN6T * N1Tm1 * ddN4T +
                         ddN3T0 * dN6T * IP * dN0T0 * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * IP * dN0T0 * N1Tm1 +
                         ddN7T * N5Tm1 * N6Tm2 * dN3T * ddN4T0 * dN1T0 * FP -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * MidP2 -
                         ddN7T * N5Tm1 * N6Tm2 * dN3T * ddN1T0 * FP * dN4T0 +
                         N4Tm1 * IA * dN6T * dN1T0 * N3Tm2 * ddN5T -
                         N4Tm1 * N6Tm2 * IS * dN5T * ddN3T * ddN1T0 -
                         N3Tm1 * N5Tm2 * IS * dN6T * ddN1T0 * ddN4T -
                         ddN3T0 * N5Tm1 * N6Tm2 * dN1T0 * FS * ddN4T -
                         dN3T0 * N4Tm1 * N5Tm2 * dN6T * FA * ddN1T0 -
                         N3Tm1 * N6Tm2 * dN7T * ddN1T0 * FP * ddN5T * dN4T0 -
                         N6Tm2 * IA * dN5T * ddN3T * N1Tm1 * dN4T0 -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * MidP2 * ddN4T -
                         N5Tm1 * N6Tm2 * dN3T * IP * dN1T0 * ddN0T0 * ddN4T -
                         ddN3T0 * IS * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * ddN6T * IS * dN5T * N1Tm1 * N4Tm2 +
                         ddN3T0 * N4Tm1 * ddN6T * N5Tm2 * dN1T0 * dN7T * FP +
                         ddN6T * N5Tm2 * dN3T * IS * ddN4T0 * N1Tm1 -
                         N5Tm1 * dN4T * N6Tm2 * IA * dN1T0 * ddN3T -
                         dN3T0 * ddN6T * N5Tm2 * ddN4T0 * FS * N1Tm1 +
                         N6Tm2 * dN3T * ddN4T0 * IP * dN0T0 * N1Tm1 * ddN5T +
                         dN3T0 * N5Tm1 * dN6T * FA * ddN1T0 * N4Tm2 +
                         dN3T0 * N5Tm2 * dN6T * IP * N1Tm1 * ddN0T0 * ddN4T +
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN0T0 * ddN1T0 -
                         N5Tm1 * N6Tm2 * dN3T * IS * ddN1T0 * ddN4T -
                         dN3T0 * N5Tm2 * IA * dN6T * N1Tm1 * ddN4T +
                         N3Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N4Tm2 -
                         ddN3T0 * N4Tm1 * ddN6T * N5Tm2 * dN1T0 * FS -
                         N4Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * ddN6T * N5Tm2 * dN7T * FP * N1Tm1 * dN4T0 -
                         N3Tm1 * dN4T * N6Tm2 * IS * ddN1T0 * ddN5T -
                         dN3T0 * N4Tm1 * N6Tm2 * FS * ddN1T0 * ddN5T -
                         N4Tm1 * N5Tm2 * IA * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * ddN6T * N5Tm2 * dN7T * ddN1T0 * FP * dN4T0 +
                         dN3T0 * N5Tm2 * dN6T * ddN1T0 * MidP1 * ddN4T +
                         N4Tm1 * N6Tm2 * dN3T * IP * dN1T0 * ddN5T * ddN0T0 -
                         N5Tm1 * N6Tm2 * dN3T * ddN4T0 * dN1T0 * FA +
                         N3Tm1 * dN6T * IP * dN1T0 * ddN5T * N4Tm2 * ddN0T0 +
                         ddN3T0 * dN4T * N6Tm2 * IS * N1Tm1 * ddN5T -
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * FA * N4Tm2 -
                         dN3T0 * ddN6T * IA * dN5T * N1Tm1 * N4Tm2 +
                         dN3T0 * N5Tm1 * ddN6T * dN7T * ddN1T0 * FP * N4Tm2 +
                         N5Tm1 * dN6T * IP * dN1T0 * N3Tm2 * ddN0T0 * ddN4T +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * MidP2 * dN4T0 -
                         ddN7T * N5Tm1 * dN6T * ddN4T0 * dN1T0 * FP * N3Tm2 -
                         N5Tm1 * N6Tm2 * ddN3T * FS * ddN1T0 * dN4T0 +
                         ddN3T0 * N5Tm1 * ddN6T * dN1T0 * FS * N4Tm2 -
                         N5Tm1 * dN4T * ddN6T * IS * ddN1T0 * N3Tm2 +
                         N6Tm2 * IP * dN5T * ddN3T * N1Tm1 * ddN0T0 * dN4T0 +
                         N6Tm2 * dN3T * ddN4T0 * dN1T0 * ddN5T * MidP1 +
                         IS * dN6T * ddN4T0 * N1Tm1 * N3Tm2 * ddN5T +
                         N4Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N3Tm2 * ddN5T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * MidP2 -
                         N3Tm1 * dN6T * IP * dN0T0 * ddN1T0 * ddN5T * N4Tm2 +
                         N6Tm2 * IA * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 * MidP1 -
                         ddN6T * N5Tm2 * IA * dN3T * N1Tm1 * dN4T0 +
                         ddN3T0 * ddN6T * N5Tm2 * FS * N1Tm1 * dN4T0 +
                         dN6T * ddN1T0 * N3Tm2 * ddN5T * MidP1 * dN4T0 -
                         N5Tm1 * dN4T * N6Tm2 * IP * ddN3T * dN0T0 * ddN1T0 -
                         N4Tm1 * N6Tm2 * IA * dN3T * dN1T0 * ddN5T -
                         N5Tm1 * dN6T * FA * ddN1T0 * N3Tm2 * dN4T0 +
                         ddN3T0 * N6Tm2 * dN5T * FA * N1Tm1 * dN4T0 -
                         N6Tm2 * dN3T * IS * ddN4T0 * N1Tm1 * ddN5T -
                         N5Tm1 * dN4T * ddN6T * IP * dN1T0 * N3Tm2 * ddN0T0 +
                         dN3T0 * N5Tm2 * dN6T * ddN4T0 * FA * N1Tm1 -
                         ddN3T0 * N4Tm1 * N6Tm2 * dN1T0 * dN7T * FP * ddN5T +
                         N4Tm1 * N5Tm2 * dN6T * IP * dN1T0 * ddN3T * ddN0T0 -
                         ddN7T * N3Tm1 * N5Tm2 * dN6T * ddN1T0 * FP * dN4T0 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * IS * ddN1T0 +
                         dN3T0 * ddN6T * IP * dN5T * N1Tm1 * N4Tm2 * ddN0T0 +
                         dN3T0 * ddN6T * N5Tm2 * ddN4T0 * dN7T * FP * N1Tm1 +
                         ddN7T * N3Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * FP +
                         N5Tm1 * ddN6T * FS * ddN1T0 * N3Tm2 * dN4T0 -
                         ddN3T0 * dN4T * N6Tm2 * dN1T0 * ddN5T * MidP1 -
                         N3Tm1 * ddN6T * N5Tm2 * FS * ddN1T0 * dN4T0 -
                         ddN6T * IS * ddN4T0 * dN5T * N1Tm1 * N3Tm2 -
                         N5Tm1 * ddN6T * dN7T * ddN1T0 * FP * N3Tm2 * dN4T0 -
                         N3Tm1 * N6Tm2 * ddN4T0 * dN1T0 * FS * ddN5T -
                         N3Tm1 * N6Tm2 * dN5T * FA * ddN1T0 * dN4T0 +
                         N5Tm1 * IS * dN6T * ddN1T0 * N3Tm2 * ddN4T +
                         N5Tm1 * N6Tm2 * dN3T * IP * dN0T0 * ddN1T0 * ddN4T +
                         N5Tm1 * ddN6T * dN3T * IP * dN1T0 * N4Tm2 * ddN0T0 +
                         N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T * MidP1 +
                         ddN3T0 * N6Tm2 * dN5T * dN1T0 * MidP1 * ddN4T +
                         ddN6T * IA * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         N4Tm1 * N6Tm2 * IP * dN5T * dN1T0 * ddN3T * ddN0T0 -
                         ddN3T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * FA -
                         N6Tm2 * dN3T * IP * N1Tm1 * ddN5T * ddN0T0 * dN4T0 -
                         N5Tm2 * dN6T * IP * ddN3T * N1Tm1 * ddN0T0 * dN4T0 +
                         N3Tm1 * N5Tm2 * IA * dN6T * dN1T0 * ddN4T +
                         N6Tm2 * dN5T * ddN3T * ddN1T0 * MidP1 * dN4T0 -
                         ddN3T0 * ddN7T * N6Tm2 * dN5T * FP * N1Tm1 * dN4T0 +
                         N3Tm1 * IS * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IP * dN0T0 * ddN1T0 +
                         dN3T0 * ddN7T * N6Tm2 * ddN4T0 * dN5T * FP * N1Tm1 +
                         dN3T0 * IA * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         N3Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN1T0 * FS -
                         dN3T0 * N5Tm1 * dN4T * N6Tm2 * FA * ddN1T0 +
                         ddN3T0 * N5Tm1 * dN4T * N6Tm2 * dN1T0 * FA -
                         ddN3T0 * ddN6T * IP * dN5T * dN0T0 * N1Tm1 * N4Tm2 +
                         N3Tm1 * ddN6T * IA * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * dN1T0 * N4Tm2 * MidP1 +
                         N5Tm1 * N6Tm2 * ddN4T0 * dN1T0 * ddN3T * FS -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * MidP2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * FA +
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * FA * N3Tm2 -
                         dN3T0 * ddN7T * N5Tm1 * dN6T * ddN1T0 * FP * N4Tm2 -
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 * MidP1 -
                         N3Tm1 * dN4T * N6Tm2 * IP * dN1T0 * ddN5T * ddN0T0 -
                         N5Tm1 * ddN6T * IA * dN3T * dN1T0 * N4Tm2 +
                         dN3T0 * N4Tm1 * N6Tm2 * dN5T * FA * ddN1T0 +
                         dN3T0 * N4Tm1 * ddN6T * N5Tm2 * FS * ddN1T0 +
                         ddN6T * N5Tm2 * dN3T * IP * N1Tm1 * ddN0T0 * dN4T0 -
                         ddN3T0 * N5Tm1 * ddN6T * dN1T0 * dN7T * FP * N4Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN1T0 * ddN0T0 -
                         dN3T0 * ddN7T * N4Tm1 * N6Tm2 * dN5T * ddN1T0 * FP -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * MidP2 * dN4T0 +
                         N4Tm1 * N6Tm2 * dN3T * IS * ddN1T0 * ddN5T -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * MidP2 * dN4T0 -
                         ddN3T0 * N6Tm2 * IS * dN5T * N1Tm1 * ddN4T +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * MidP2 * ddN4T +
                         ddN3T0 * dN6T * dN1T0 * ddN5T * N4Tm2 * MidP1 +
                         N4Tm1 * N6Tm2 * IP * dN5T * ddN3T * dN0T0 * ddN1T0 +
                         dN3T0 * N5Tm1 * N6Tm2 * FS * ddN1T0 * ddN4T +
                         N3Tm1 * N6Tm2 * IP * dN5T * dN1T0 * ddN0T0 * ddN4T -
                         N3Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN1T0 * dN7T * FP -
                         ddN3T0 * N6Tm2 * FS * N1Tm1 * ddN5T * dN4T0 +
                         ddN3T0 * N6Tm2 * dN7T * FP * N1Tm1 * ddN5T * dN4T0 +
                         N3Tm1 * N6Tm2 * ddN4T0 * dN5T * dN1T0 * FA -
                         N3Tm1 * IA * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N5Tm2 * dN6T * FA * ddN1T0 * dN4T0 -
                         dN3T0 * ddN7T * N5Tm2 * dN6T * ddN4T0 * FP * N1Tm1 +
                         N5Tm1 * dN4T * ddN6T * IP * dN0T0 * ddN1T0 * N3Tm2 +
                         dN3T0 * ddN7T * N5Tm1 * dN4T * N6Tm2 * ddN1T0 * FP +
                         N6Tm2 * IS * ddN4T0 * dN5T * ddN3T * N1Tm1 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * MidP2 +
                         ddN6T * N5Tm2 * dN3T * ddN1T0 * MidP1 * dN4T0 -
                         dN3T0 * dN6T * ddN1T0 * ddN5T * N4Tm2 * MidP1 -
                         dN3T0 * N4Tm1 * ddN6T * N5Tm2 * dN7T * ddN1T0 * FP -
                         ddN3T0 * N5Tm2 * dN6T * FA * N1Tm1 * dN4T0 +
                         dN3T0 * dN4T * ddN6T * N5Tm2 * IA * N1Tm1 +
                         N3Tm1 * N6Tm2 * ddN4T0 * dN1T0 * dN7T * FP * ddN5T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * MidP2 +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * MidP2 * dN4T0 -
                         N5Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N3Tm2 * ddN4T +
                         N4Tm1 * ddN6T * IS * dN5T * ddN1T0 * N3Tm2 +
                         ddN3T0 * N6Tm2 * IP * dN5T * dN0T0 * N1Tm1 * ddN4T +
                         N4Tm1 * N5Tm2 * IS * dN6T * ddN3T * ddN1T0 +
                         N3Tm1 * N6Tm2 * IS * dN5T * ddN1T0 * ddN4T +
                         ddN7T * N3Tm1 * N6Tm2 * dN5T * ddN1T0 * FP * dN4T0 -
                         ddN3T0 * ddN7T * N4Tm1 * N5Tm2 * dN6T * dN1T0 * FP -
                         N3Tm1 * N5Tm2 * dN6T * IP * dN1T0 * ddN0T0 * ddN4T -
                         ddN6T * dN5T * ddN1T0 * N3Tm2 * MidP1 * dN4T0 -
                         N5Tm2 * IS * dN6T * ddN4T0 * ddN3T * N1Tm1 -
                         N5Tm1 * IS * dN6T * ddN3T * ddN1T0 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * IP * dN1T0 * ddN3T * ddN0T0 -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * MidP2 +
                         N4Tm1 * N6Tm2 * IA * dN5T * dN1T0 * ddN3T -
                         dN3T0 * N5Tm1 * ddN6T * FS * ddN1T0 * N4Tm2 -
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * IP * dN0T0 * N1Tm1 +
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN1T0 * MidP1 +
                         N3Tm1 * dN4T * N6Tm2 * IP * dN0T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * N6Tm2 * dN7T * ddN1T0 * FP * ddN4T -
                         ddN7T * N3Tm1 * N6Tm2 * ddN4T0 * dN5T * dN1T0 * FP +
                         dN3T0 * N6Tm2 * IA * dN5T * N1Tm1 * ddN4T -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * MidP2 +
                         N4Tm1 * ddN6T * N5Tm2 * IA * dN3T * dN1T0 +
                         ddN6T * ddN4T0 * IP * dN5T * dN0T0 * N1Tm1 * N3Tm2 -
                         N4Tm1 * N5Tm2 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 -
                         N6Tm2 * ddN4T0 * dN5T * dN1T0 * ddN3T * MidP1 -
                         dN3T0 * dN4T * N6Tm2 * IA * N1Tm1 * ddN5T +
                         N5Tm2 * dN6T * ddN4T0 * IP * ddN3T * dN0T0 * N1Tm1 -
                         ddN3T0 * N5Tm2 * dN6T * dN1T0 * MidP1 * ddN4T -
                         N5Tm1 * IA * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * N5Tm2 * dN6T * IP * dN0T0 * N1Tm1 * ddN4T -
                         N5Tm1 * ddN6T * dN3T * IP * dN0T0 * ddN1T0 * N4Tm2 +
                         ddN3T0 * N4Tm1 * N6Tm2 * dN1T0 * FS * ddN5T -
                         N5Tm1 * N6Tm2 * ddN4T0 * dN1T0 * ddN3T * dN7T * FP -
                         N5Tm1 * dN6T * IP * dN1T0 * ddN3T * N4Tm2 * ddN0T0 +
                         ddN3T0 * ddN7T * N5Tm2 * dN6T * FP * N1Tm1 * dN4T0 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * MidP2 +
                         N5Tm1 * ddN6T * dN3T * IS * ddN1T0 * N4Tm2 +
                         dN6T * IP * N1Tm1 * N3Tm2 * ddN5T * ddN0T0 * dN4T0 -
                         dN3T0 * N6Tm2 * ddN4T0 * dN5T * FA * N1Tm1 -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * IS * N1Tm1 -
                         dN6T * ddN4T0 * IP * dN0T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN3T * ddN1T0 * MidP1 * dN4T0 +
                         N5Tm1 * IA * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * ddN7T * N5Tm1 * dN6T * dN1T0 * FP * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IP * dN1T0 * ddN0T0 +
                         dN3T0 * dN4T * N6Tm2 * ddN1T0 * ddN5T * MidP1 +
                         ddN7T * N5Tm1 * dN6T * ddN1T0 * FP * N3Tm2 * dN4T0 -
                         dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T * MidP1 -
                         N3Tm1 * ddN6T * IP * dN5T * dN1T0 * N4Tm2 * ddN0T0 -
                         N3Tm1 * N6Tm2 * IP * dN5T * dN0T0 * ddN1T0 * ddN4T +
                         N5Tm1 * N6Tm2 * dN3T * FA * ddN1T0 * dN4T0 +
                         N5Tm1 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 * N4Tm2 -
                         dN3T0 * N6Tm2 * ddN4T0 * dN7T * FP * N1Tm1 * ddN5T +
                         dN3T0 * N6Tm2 * ddN4T0 * FS * N1Tm1 * ddN5T +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * MidP2 -
                         N6Tm2 * ddN4T0 * IP * dN5T * ddN3T * dN0T0 * N1Tm1 -
                         ddN6T * IP * dN5T * N1Tm1 * N3Tm2 * ddN0T0 * dN4T0 +
                         N5Tm1 * ddN6T * ddN4T0 * dN1T0 * dN7T * FP * N3Tm2 -
                         IA * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * MidP2 +
                         N5Tm1 * N6Tm2 * IA * dN3T * dN1T0 * ddN4T +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * MidP2 -
                         ddN3T0 * ddN7T * N5Tm1 * dN4T * N6Tm2 * dN1T0 * FP +
                         N3Tm1 * N5Tm2 * dN6T * IP * dN0T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N5Tm1 * N6Tm2 * dN1T0 * dN7T * FP * ddN4T -
                         dN3T0 * dN4T * ddN6T * N5Tm2 * IP * N1Tm1 * ddN0T0 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * IS * ddN1T0 +
                         dN3T0 * ddN7T * N4Tm1 * N5Tm2 * dN6T * ddN1T0 * FP -
                         N3Tm1 * N6Tm2 * IA * dN5T * dN1T0 * ddN4T -
                         N4Tm1 * ddN6T * IA * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN6T * IA * dN1T0 * N3Tm2 +
                         dN3T0 * ddN6T * dN5T * ddN1T0 * N4Tm2 * MidP1 +
                         N3Tm1 * dN4T * N6Tm2 * IA * dN1T0 * ddN5T -
                         dN3T0 * dN4T * ddN6T * N5Tm2 * ddN1T0 * MidP1 +
                         N4Tm1 * ddN6T * IP * dN5T * dN1T0 * N3Tm2 * ddN0T0 -
                         dN3T0 * dN6T * IP * N1Tm1 * ddN5T * N4Tm2 * ddN0T0 +
                         N5Tm1 * N6Tm2 * ddN3T * dN7T * ddN1T0 * FP * dN4T0 +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * MidP2 +
                         ddN3T0 * ddN7T * N4Tm1 * N6Tm2 * dN5T * dN1T0 * FP +
                         N3Tm1 * N6Tm2 * FS * ddN1T0 * ddN5T * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * IP * N1Tm1 * ddN5T * ddN0T0 -
                         N6Tm2 * dN3T * ddN1T0 * ddN5T * MidP1 * dN4T0 -
                         N5Tm1 * ddN6T * ddN4T0 * dN1T0 * FS * N3Tm2 -
                         dN3T0 * N6Tm2 * dN5T * ddN1T0 * MidP1 * ddN4T +
                         N3Tm1 * ddN6T * dN5T * ddN1T0 * MidP2 * dN4T0 -
                         N4Tm1 * N6Tm2 * dN3T * IP * dN0T0 * ddN1T0 * ddN5T -
                         ddN3T0 * dN4T * N6Tm2 * IP * dN0T0 * N1Tm1 * ddN5T -
                         N4Tm1 * IS * dN6T * ddN1T0 * N3Tm2 * ddN5T) /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T);
  m_control_points[3] = -1.0 /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T) *
                        (ddN6T * IP * dN2T0 * dN5T * N1Tm1 * N4Tm2 * ddN0T0 +
                         N5Tm2 * dN6T * dN2T0 * ddN1T0 * MidP1 * ddN4T +
                         ddN7T * N5Tm1 * dN6T * ddN1T0 * FP * N2Tm2 * dN4T0 -
                         ddN6T * dN5T * ddN1T0 * MidP1 * N2Tm2 * dN4T0 +
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * FA * ddN1T0 +
                         ddN7T * N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * FP -
                         dN4T * N6Tm2 * N2Tm1 * IS * ddN1T0 * ddN5T -
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * FA * ddN1T0 -
                         ddN7T * ddN2T0 * N6Tm2 * dN5T * FP * N1Tm1 * dN4T0 +
                         N6Tm2 * N2Tm1 * IP * dN5T * dN1T0 * ddN0T0 * ddN4T -
                         N2Tm1 * N5Tm2 * dN6T * IP * dN1T0 * ddN0T0 * ddN4T -
                         N2Tm1 * ddN6T * N5Tm2 * FS * ddN1T0 * dN4T0 -
                         ddN2T0 * N5Tm2 * dN6T * IP * dN0T0 * N1Tm1 * ddN4T -
                         dN6T * ddN4T0 * dN1T0 * ddN5T * MidP1 * N2Tm2 +
                         N6Tm2 * N2Tm1 * ddN4T0 * dN1T0 * dN7T * FP * ddN5T -
                         N5Tm1 * ddN2T0 * N6Tm2 * dN1T0 * FS * ddN4T -
                         ddN7T * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * FP * dN4T0 -
                         ddN7T * N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * FP -
                         dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T * MidP1 -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN1T0 * FS * ddN5T +
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * FA * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN5T * FA * ddN1T0 * dN4T0 -
                         IA * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         N4Tm1 * ddN6T * N5Tm2 * dN2T0 * dN7T * ddN1T0 * FP -
                         N2Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN1T0 * dN7T * FP -
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * FA -
                         ddN6T * IA * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N2Tm1 * ddN6T * IA * dN5T * dN1T0 * N4Tm2 +
                         ddN2T0 * N6Tm2 * IP * dN5T * dN0T0 * N1Tm1 * ddN4T -
                         ddN7T * N5Tm2 * dN6T * ddN4T0 * dN2T0 * FP * N1Tm1 +
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * MidP2 -
                         dN4T * ddN6T * N5Tm2 * IP * dN2T0 * N1Tm1 * ddN0T0 +
                         N2Tm1 * IS * dN6T * ddN1T0 * ddN5T * N4Tm2 +
                         N5Tm1 * ddN2T0 * ddN6T * dN1T0 * FS * N4Tm2 +
                         N2Tm1 * N5Tm2 * IA * dN6T * dN1T0 * ddN4T -
                         dN4T * N6Tm2 * IA * dN2T0 * N1Tm1 * ddN5T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * FA * N4Tm2 +
                         ddN2T0 * N6Tm2 * dN7T * FP * N1Tm1 * ddN5T * dN4T0 -
                         dN4T * N6Tm2 * N2Tm1 * IP * dN1T0 * ddN5T * ddN0T0 +
                         N2Tm1 * ddN6T * N5Tm2 * ddN4T0 * dN1T0 * FS +
                         dN6T * IP * N1Tm1 * ddN5T * N2Tm2 * ddN0T0 * dN4T0 -
                         ddN6T * IS * ddN4T0 * dN5T * N1Tm1 * N2Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * FA +
                         N4Tm1 * ddN6T * IP * dN5T * dN1T0 * N2Tm2 * ddN0T0 -
                         dN4T * N2Tm1 * ddN6T * N5Tm2 * IP * dN0T0 * ddN1T0 -
                         N4Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N2Tm2 +
                         dN4T * N2Tm1 * ddN6T * N5Tm2 * IS * ddN1T0 -
                         N2Tm1 * N5Tm2 * IS * dN6T * ddN1T0 * ddN4T -
                         ddN6T * IP * dN5T * N1Tm1 * N2Tm2 * ddN0T0 * dN4T0 +
                         N2Tm1 * N5Tm2 * dN6T * IP * dN0T0 * ddN1T0 * ddN4T -
                         ddN2T0 * N5Tm2 * dN6T * FA * N1Tm1 * dN4T0 -
                         N6Tm2 * N2Tm1 * IP * dN5T * dN0T0 * ddN1T0 * ddN4T -
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T * MidP2 +
                         dN4T * N6Tm2 * IP * dN2T0 * N1Tm1 * ddN5T * ddN0T0 -
                         N4Tm1 * N6Tm2 * dN2T0 * FS * ddN1T0 * ddN5T -
                         N5Tm1 * dN4T * ddN6T * IS * ddN1T0 * N2Tm2 -
                         N2Tm1 * ddN6T * IS * dN5T * ddN1T0 * N4Tm2 -
                         ddN7T * N5Tm1 * dN6T * ddN4T0 * dN1T0 * FP * N2Tm2 -
                         N5Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N2Tm2 * ddN4T -
                         ddN6T * N5Tm2 * ddN4T0 * dN2T0 * FS * N1Tm1 -
                         ddN2T0 * N6Tm2 * FS * N1Tm1 * ddN5T * dN4T0 +
                         N4Tm1 * dN6T * IP * dN0T0 * ddN1T0 * ddN5T * N2Tm2 +
                         IS * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 * MidP1 -
                         ddN2T0 * N6Tm2 * IS * dN5T * N1Tm1 * ddN4T +
                         dN4T * ddN6T * N5Tm2 * IA * dN2T0 * N1Tm1 -
                         N5Tm1 * N6Tm2 * dN2T0 * dN7T * ddN1T0 * FP * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * MidP2 * ddN4T -
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * MidP2 * dN4T0 -
                         N2Tm1 * ddN6T * IP * dN5T * dN1T0 * N4Tm2 * ddN0T0 -
                         ddN7T * N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * FP -
                         dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 * MidP1 +
                         N5Tm1 * dN6T * dN2T0 * FA * ddN1T0 * N4Tm2 -
                         N5Tm1 * ddN6T * dN2T0 * FS * ddN1T0 * N4Tm2 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN1T0 * FS * ddN5T +
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * FA * N1Tm1 -
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * MidP2 -
                         dN4T * ddN2T0 * N6Tm2 * IP * dN0T0 * N1Tm1 * ddN5T +
                         ddN7T * N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * FP -
                         ddN2T0 * N5Tm2 * dN6T * dN1T0 * MidP1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN1T0 * dN7T * FP * N4Tm2 +
                         N4Tm1 * IA * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         ddN7T * N5Tm1 * ddN2T0 * dN6T * dN1T0 * FP * N4Tm2 +
                         dN4T * ddN2T0 * N6Tm2 * IS * N1Tm1 * ddN5T -
                         N5Tm1 * IA * dN6T * dN1T0 * N2Tm2 * ddN4T +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * MidP2 * ddN4T +
                         N2Tm1 * ddN6T * N5Tm2 * dN7T * ddN1T0 * FP * dN4T0 +
                         N5Tm1 * dN4T * ddN6T * IA * dN1T0 * N2Tm2 -
                         N5Tm1 * dN4T * ddN6T * IP * dN1T0 * N2Tm2 * ddN0T0 +
                         N2Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N4Tm2 -
                         dN4T * ddN2T0 * ddN6T * N5Tm2 * IS * N1Tm1 -
                         N4Tm1 * ddN6T * IA * dN5T * dN1T0 * N2Tm2 +
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * MidP2 -
                         N5Tm1 * dN6T * FA * ddN1T0 * N2Tm2 * dN4T0 -
                         N6Tm2 * N2Tm1 * dN7T * ddN1T0 * FP * ddN5T * dN4T0 +
                         ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 * MidP1 +
                         N5Tm1 * IS * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N2Tm1 * dN6T * IP * dN0T0 * ddN1T0 * ddN5T * N4Tm2 -
                         ddN2T0 * ddN6T * N5Tm2 * dN7T * FP * N1Tm1 * dN4T0 -
                         N5Tm1 * ddN6T * dN7T * ddN1T0 * FP * N2Tm2 * dN4T0 +
                         ddN2T0 * N6Tm2 * dN5T * FA * N1Tm1 * dN4T0 +
                         N5Tm1 * ddN6T * FS * ddN1T0 * N2Tm2 * dN4T0 +
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * MidP2 +
                         ddN7T * N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * FP +
                         dN4T * ddN2T0 * ddN6T * N5Tm2 * IP * dN0T0 * N1Tm1 +
                         dN4T * N2Tm1 * ddN6T * N5Tm2 * IP * dN1T0 * ddN0T0 +
                         ddN7T * ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * FP -
                         N5Tm1 * ddN6T * ddN4T0 * dN1T0 * FS * N2Tm2 +
                         N2Tm1 * dN6T * IP * dN1T0 * ddN5T * N4Tm2 * ddN0T0 -
                         N4Tm1 * IS * dN6T * ddN1T0 * ddN5T * N2Tm2 +
                         ddN2T0 * ddN6T * N5Tm2 * FS * N1Tm1 * dN4T0 +
                         N5Tm2 * dN6T * IP * dN2T0 * N1Tm1 * ddN0T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN1T0 * dN7T * FP -
                         N6Tm2 * IP * dN2T0 * dN5T * N1Tm1 * ddN0T0 * ddN4T -
                         dN6T * ddN4T0 * IP * dN0T0 * N1Tm1 * ddN5T * N2Tm2 +
                         N4Tm1 * ddN6T * N5Tm2 * dN2T0 * FS * ddN1T0 -
                         ddN7T * N5Tm1 * dN6T * dN2T0 * ddN1T0 * FP * N4Tm2 -
                         N6Tm2 * N2Tm1 * IA * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * MidP2 * dN4T0 -
                         dN6T * IP * dN2T0 * N1Tm1 * ddN5T * N4Tm2 * ddN0T0 +
                         ddN6T * IA * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 * MidP1 +
                         N5Tm1 * dN6T * IP * dN1T0 * N2Tm2 * ddN0T0 * ddN4T -
                         dN4T * N2Tm1 * ddN6T * N5Tm2 * IA * dN1T0 +
                         ddN2T0 * dN6T * IP * dN0T0 * N1Tm1 * ddN5T * N4Tm2 +
                         N5Tm1 * ddN6T * dN2T0 * dN7T * ddN1T0 * FP * N4Tm2 -
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * FA * N1Tm1 +
                         N6Tm2 * N2Tm1 * FS * ddN1T0 * ddN5T * dN4T0 +
                         N5Tm1 * ddN6T * ddN4T0 * dN1T0 * dN7T * FP * N2Tm2 -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * FA +
                         ddN2T0 * N6Tm2 * dN5T * dN1T0 * MidP1 * ddN4T -
                         N4Tm1 * dN6T * IP * dN1T0 * ddN5T * N2Tm2 * ddN0T0 +
                         ddN2T0 * ddN6T * IS * dN5T * N1Tm1 * N4Tm2 -
                         N6Tm2 * ddN4T0 * dN2T0 * dN7T * FP * N1Tm1 * ddN5T -
                         ddN2T0 * IS * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * FA +
                         ddN6T * ddN4T0 * IP * dN5T * dN0T0 * N1Tm1 * N2Tm2 -
                         ddN2T0 * ddN6T * IP * dN5T * dN0T0 * N1Tm1 * N4Tm2 +
                         N6Tm2 * IA * dN2T0 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * MidP2 +
                         dN4T * N6Tm2 * N2Tm1 * IA * dN1T0 * ddN5T +
                         N6Tm2 * ddN4T0 * dN2T0 * FS * N1Tm1 * ddN5T +
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * FA -
                         N2Tm1 * dN6T * ddN1T0 * ddN5T * MidP2 * dN4T0 +
                         ddN2T0 * N5Tm2 * IS * dN6T * N1Tm1 * ddN4T +
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * MidP2 +
                         ddN6T * N5Tm2 * ddN4T0 * dN2T0 * dN7T * FP * N1Tm1 +
                         N2Tm1 * N5Tm2 * dN6T * FA * ddN1T0 * dN4T0 +
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * MidP2 -
                         N2Tm1 * IA * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         ddN7T * ddN2T0 * N5Tm2 * dN6T * FP * N1Tm1 * dN4T0 +
                         N6Tm2 * N2Tm1 * IS * dN5T * ddN1T0 * ddN4T +
                         ddN7T * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * FP * dN4T0 +
                         dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 * MidP1 -
                         dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 * MidP1 +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN1T0 * dN7T * FP * ddN4T +
                         dN4T * N6Tm2 * N2Tm1 * IP * dN0T0 * ddN1T0 * ddN5T -
                         ddN7T * N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * FP +
                         ddN2T0 * dN6T * N1Tm1 * ddN5T * MidP2 * dN4T0 +
                         ddN6T * ddN4T0 * dN5T * dN1T0 * MidP1 * N2Tm2 +
                         N5Tm1 * N6Tm2 * dN2T0 * FS * ddN1T0 * ddN4T -
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * MidP2 -
                         N6Tm2 * dN2T0 * dN5T * ddN1T0 * MidP1 * ddN4T -
                         ddN7T * ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * FP +
                         N5Tm1 * dN4T * ddN6T * IP * dN0T0 * ddN1T0 * N2Tm2 -
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * MidP2 -
                         N5Tm2 * IA * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN1T0 * FS +
                         dN6T * ddN1T0 * ddN5T * MidP1 * N2Tm2 * dN4T0 -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN1T0 * dN7T * FP * ddN5T +
                         N4Tm1 * N6Tm2 * dN2T0 * dN7T * ddN1T0 * FP * ddN5T -
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * FA * ddN1T0 +
                         N4Tm1 * ddN6T * IS * dN5T * ddN1T0 * N2Tm2 +
                         dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T * MidP1 +
                         ddN7T * N6Tm2 * ddN4T0 * dN2T0 * dN5T * FP * N1Tm1 +
                         IA * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2);
  m_control_points[4] = -(N3Tm1 * ddN6T * IA * dN5T * dN1T0 * N2Tm2 +
                          ddN2T0 * N6Tm2 * dN3T * IP * dN0T0 * N1Tm1 * ddN5T -
                          ddN3T0 * ddN6T * IP * dN5T * dN0T0 * N1Tm1 * N2Tm2 +
                          ddN3T0 * ddN6T * N5Tm2 * dN2T0 * FS * N1Tm1 -
                          N3Tm1 * ddN2T0 * ddN6T * N5Tm2 * dN1T0 * dN7T * FP -
                          N5Tm1 * ddN6T * IA * dN3T * dN1T0 * N2Tm2 -
                          dN3T0 * ddN7T * N5Tm1 * dN6T * ddN1T0 * FP * N2Tm2 +
                          dN3T0 * ddN7T * ddN2T0 * N6Tm2 * dN5T * FP * N1Tm1 -
                          ddN3T0 * N5Tm1 * ddN6T * dN1T0 * dN7T * FP * N2Tm2 +
                          dN3T0 * N2Tm1 * ddN6T * N5Tm2 * FS * ddN1T0 -
                          ddN7T * N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * FP -
                          dN3T0 * ddN2T0 * N6Tm2 * dN7T * FP * N1Tm1 * ddN5T +
                          ddN3T0 * ddN7T * N6Tm2 * N2Tm1 * dN5T * dN1T0 * FP +
                          N3Tm1 * IS * dN6T * ddN1T0 * ddN5T * N2Tm2 +
                          N6Tm2 * N2Tm1 * IP * dN5T * ddN3T * dN0T0 * ddN1T0 -
                          N2Tm1 * N5Tm2 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 +
                          dN3T0 * ddN6T * dN5T * ddN1T0 * MidP1 * N2Tm2 +
                          N5Tm1 * ddN2T0 * N6Tm2 * dN1T0 * ddN3T * FS -
                          dN3T0 * dN6T * IP * N1Tm1 * ddN5T * N2Tm2 * ddN0T0 +
                          dN3T0 * IA * dN6T * N1Tm1 * ddN5T * N2Tm2 -
                          ddN6T * N5Tm2 * IA * dN3T * dN2T0 * N1Tm1 +
                          ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * MidP2 +
                          N5Tm1 * ddN2T0 * dN6T * dN1T0 * FA * N3Tm2 -
                          N2Tm1 * IS * dN6T * ddN1T0 * N3Tm2 * ddN5T -
                          N5Tm1 * ddN2T0 * ddN6T * dN1T0 * FS * N3Tm2 -
                          ddN6T * IP * dN2T0 * dN5T * N1Tm1 * N3Tm2 * ddN0T0 -
                          N3Tm1 * ddN6T * IP * dN5T * dN1T0 * N2Tm2 * ddN0T0 +
                          N5Tm1 * N6Tm2 * dN3T * dN2T0 * FA * ddN1T0 -
                          N3Tm1 * ddN6T * N5Tm2 * dN2T0 * FS * ddN1T0 -
                          N2Tm1 * ddN6T * IA * dN5T * dN1T0 * N3Tm2 +
                          ddN6T * IA * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                          ddN2T0 * ddN6T * N5Tm2 * dN3T * IS * N1Tm1 +
                          N3Tm1 * ddN6T * N5Tm2 * dN2T0 * dN7T * ddN1T0 * FP -
                          N2Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN1T0 * ddN0T0 -
                          N3Tm1 * ddN2T0 * N6Tm2 * dN1T0 * FS * ddN5T +
                          dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * MidP2 +
                          ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 * MidP1 -
                          ddN3T0 * N5Tm1 * dN6T * dN1T0 * FA * N2Tm2 +
                          N2Tm1 * N5Tm2 * IS * dN6T * ddN3T * ddN1T0 +
                          N6Tm2 * IA * dN3T * dN2T0 * N1Tm1 * ddN5T +
                          dN3T0 * N5Tm1 * ddN6T * dN7T * ddN1T0 * FP * N2Tm2 -
                          dN3T0 * ddN6T * IA * dN5T * N1Tm1 * N2Tm2 -
                          N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * MidP2 +
                          N2Tm1 * ddN6T * IS * dN5T * ddN1T0 * N3Tm2 +
                          ddN3T0 * N5Tm1 * ddN6T * dN1T0 * FS * N2Tm2 -
                          dN3T0 * N2Tm1 * ddN6T * N5Tm2 * dN7T * ddN1T0 * FP -
                          ddN3T0 * N2Tm1 * ddN6T * N5Tm2 * dN1T0 * FS +
                          N2Tm1 * ddN6T * IP * dN5T * dN1T0 * N3Tm2 * ddN0T0 +
                          N6Tm2 * N2Tm1 * dN3T * IS * ddN1T0 * ddN5T -
                          ddN2T0 * N6Tm2 * IP * dN5T * ddN3T * dN0T0 * N1Tm1 -
                          ddN7T * N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * FP -
                          N3Tm1 * dN6T * IP * dN0T0 * ddN1T0 * ddN5T * N2Tm2 +
                          dN3T0 * N5Tm1 * dN6T * FA * ddN1T0 * N2Tm2 +
                          ddN3T0 * N6Tm2 * N2Tm1 * dN1T0 * FS * ddN5T -
                          N5Tm1 * dN6T * IP * dN1T0 * ddN3T * N2Tm2 * ddN0T0 +
                          N3Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N2Tm2 -
                          N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 * MidP1 -
                          ddN2T0 * ddN6T * N5Tm2 * dN3T * dN1T0 * MidP1 -
                          ddN3T0 * N5Tm2 * dN6T * dN2T0 * FA * N1Tm1 -
                          ddN3T0 * ddN7T * N6Tm2 * dN2T0 * dN5T * FP * N1Tm1 -
                          N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * MidP2 +
                          N3Tm1 * ddN2T0 * N6Tm2 * dN1T0 * dN7T * FP * ddN5T -
                          ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * MidP2 -
                          dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * MidP2 +
                          ddN2T0 * N5Tm2 * dN6T * IP * ddN3T * dN0T0 * N1Tm1 +
                          N5Tm1 * ddN2T0 * ddN6T * dN1T0 * dN7T * FP * N3Tm2 -
                          dN3T0 * ddN7T * ddN2T0 * N5Tm2 * dN6T * FP * N1Tm1 +
                          ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * FA -
                          dN3T0 * ddN7T * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * FP -
                          ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN3T * MidP1 -
                          ddN3T0 * IS * dN6T * N1Tm1 * ddN5T * N2Tm2 -
                          dN3T0 * dN6T * ddN1T0 * ddN5T * MidP1 * N2Tm2 -
                          N6Tm2 * N2Tm1 * dN3T * IP * dN0T0 * ddN1T0 * ddN5T +
                          N6Tm2 * N2Tm1 * IA * dN5T * dN1T0 * ddN3T +
                          ddN2T0 * N6Tm2 * IS * dN5T * ddN3T * N1Tm1 -
                          ddN7T * N5Tm1 * ddN2T0 * dN6T * dN1T0 * FP * N3Tm2 +
                          ddN3T0 * ddN6T * IS * dN5T * N1Tm1 * N2Tm2 +
                          N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * MidP2 +
                          N2Tm1 * ddN6T * N5Tm2 * IA * dN3T * dN1T0 -
                          N5Tm1 * dN6T * dN2T0 * FA * ddN1T0 * N3Tm2 -
                          N5Tm1 * N6Tm2 * dN2T0 * ddN3T * FS * ddN1T0 +
                          N3Tm1 * N5Tm2 * dN6T * dN2T0 * FA * ddN1T0 -
                          N3Tm1 * ddN6T * IS * dN5T * ddN1T0 * N2Tm2 -
                          N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * MidP2 -
                          ddN3T0 * ddN6T * N5Tm2 * dN2T0 * dN7T * FP * N1Tm1 -
                          dN3T0 * N2Tm1 * N5Tm2 * dN6T * FA * ddN1T0 -
                          N5Tm1 * ddN2T0 * N6Tm2 * dN1T0 * ddN3T * dN7T * FP +
                          N5Tm1 * ddN6T * dN2T0 * FS * ddN1T0 * N3Tm2 +
                          ddN3T0 * dN6T * dN1T0 * ddN5T * MidP1 * N2Tm2 +
                          ddN3T0 * dN6T * IP * dN0T0 * N1Tm1 * ddN5T * N2Tm2 -
                          ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN5T * MidP1 +
                          N2Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N3Tm2 * ddN5T +
                          N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * MidP2 +
                          N5Tm1 * N6Tm2 * dN2T0 * ddN3T * dN7T * ddN1T0 * FP +
                          ddN2T0 * ddN6T * dN5T * dN1T0 * N3Tm2 * MidP1 -
                          N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * FA +
                          dN3T0 * ddN7T * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * FP -
                          ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * FA +
                          dN6T * IP * dN2T0 * N1Tm1 * N3Tm2 * ddN5T * ddN0T0 -
                          ddN3T0 * ddN6T * dN5T * dN1T0 * MidP1 * N2Tm2 -
                          N6Tm2 * N2Tm1 * IA * dN3T * dN1T0 * ddN5T -
                          ddN3T0 * N6Tm2 * N2Tm1 * dN1T0 * dN7T * FP * ddN5T -
                          N3Tm1 * N6Tm2 * dN2T0 * dN5T * FA * ddN1T0 -
                          N2Tm1 * ddN6T * N5Tm2 * dN3T * IS * ddN1T0 +
                          ddN6T * N5Tm2 * dN3T * IP * dN2T0 * N1Tm1 * ddN0T0 +
                          dN3T0 * N6Tm2 * N2Tm1 * dN5T * FA * ddN1T0 +
                          ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * MidP2 +
                          N2Tm1 * ddN6T * N5Tm2 * dN3T * IP * dN0T0 * ddN1T0 +
                          N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * MidP2 -
                          N2Tm1 * ddN6T * IP * dN5T * dN0T0 * ddN1T0 * N3Tm2 -
                          ddN2T0 * dN6T * IP * dN0T0 * N1Tm1 * N3Tm2 * ddN5T +
                          ddN3T0 * ddN7T * N5Tm2 * dN6T * dN2T0 * FP * N1Tm1 +
                          N5Tm1 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 * N2Tm2 +
                          ddN3T0 * N6Tm2 * dN2T0 * dN5T * FA * N1Tm1 -
                          N5Tm1 * ddN6T * dN2T0 * dN7T * ddN1T0 * FP * N3Tm2 +
                          ddN7T * N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * FP +
                          ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN5T * MidP1 +
                          dN3T0 * ddN6T * IP * dN5T * N1Tm1 * N2Tm2 * ddN0T0 +
                          N3Tm1 * ddN2T0 * ddN6T * N5Tm2 * dN1T0 * FS +
                          N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 * MidP1 +
                          ddN7T * N5Tm1 * dN6T * dN2T0 * ddN1T0 * FP * N3Tm2 +
                          dN3T0 * ddN2T0 * N5Tm2 * dN6T * FA * N1Tm1 +
                          ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN3T * MidP1 +
                          ddN3T0 * ddN7T * N5Tm1 * dN6T * dN1T0 * FP * N2Tm2 +
                          N6Tm2 * IP * dN2T0 * dN5T * ddN3T * N1Tm1 * ddN0T0 -
                          N2Tm1 * dN6T * IP * dN1T0 * N3Tm2 * ddN5T * ddN0T0 +
                          N5Tm1 * ddN6T * dN3T * IP * dN1T0 * N2Tm2 * ddN0T0 +
                          N5Tm1 * IA * dN6T * dN1T0 * ddN3T * N2Tm2 -
                          N5Tm2 * dN6T * IP * dN2T0 * ddN3T * N1Tm1 * ddN0T0 -
                          ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 * MidP1 +
                          N6Tm2 * N2Tm1 * dN3T * IP * dN1T0 * ddN5T * ddN0T0 +
                          dN3T0 * ddN2T0 * N6Tm2 * FS * N1Tm1 * ddN5T -
                          N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T * MidP1 +
                          ddN3T0 * N2Tm1 * ddN6T * N5Tm2 * dN1T0 * dN7T * FP +
                          dN3T0 * ddN2T0 * ddN6T * N5Tm2 * dN7T * FP * N1Tm1 -
                          N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * MidP2 -
                          ddN3T0 * N6Tm2 * dN2T0 * FS * N1Tm1 * ddN5T +
                          dN3T0 * N6Tm2 * N2Tm1 * dN7T * ddN1T0 * FP * ddN5T -
                          dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * MidP2 +
                          dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T * MidP1 -
                          N5Tm1 * ddN6T * dN3T * IP * dN0T0 * ddN1T0 * N2Tm2 +
                          ddN7T * N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * FP +
                          N5Tm1 * ddN6T * dN3T * IS * ddN1T0 * N2Tm2 -
                          N6Tm2 * dN3T * IP * dN2T0 * N1Tm1 * ddN5T * ddN0T0 -
                          dN3T0 * N6Tm2 * N2Tm1 * FS * ddN1T0 * ddN5T -
                          ddN2T0 * N6Tm2 * dN3T * IS * N1Tm1 * ddN5T +
                          N5Tm2 * IA * dN6T * dN2T0 * ddN3T * N1Tm1 +
                          N2Tm1 * IA * dN6T * dN1T0 * N3Tm2 * ddN5T -
                          ddN7T * N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * FP +
                          N3Tm1 * dN6T * IP * dN1T0 * ddN5T * N2Tm2 * ddN0T0 +
                          ddN2T0 * IS * dN6T * N1Tm1 * N3Tm2 * ddN5T -
                          ddN2T0 * ddN6T * IS * dN5T * N1Tm1 * N3Tm2 -
                          N5Tm1 * IS * dN6T * ddN3T * ddN1T0 * N2Tm2 -
                          dN3T0 * ddN2T0 * ddN6T * N5Tm2 * FS * N1Tm1 +
                          ddN2T0 * ddN6T * IP * dN5T * dN0T0 * N1Tm1 * N3Tm2 -
                          N6Tm2 * N2Tm1 * IS * dN5T * ddN3T * ddN1T0 -
                          dN3T0 * N5Tm1 * ddN6T * FS * ddN1T0 * N2Tm2 -
                          dN3T0 * ddN2T0 * N6Tm2 * dN5T * FA * N1Tm1 +
                          N3Tm1 * N6Tm2 * dN2T0 * FS * ddN1T0 * ddN5T -
                          ddN2T0 * N5Tm2 * IS * dN6T * ddN3T * N1Tm1 +
                          N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * MidP2 -
                          N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * FA +
                          ddN7T * N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * FP -
                          IA * dN6T * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                          N3Tm1 * N6Tm2 * dN2T0 * dN7T * ddN1T0 * FP * ddN5T +
                          ddN3T0 * N6Tm2 * dN2T0 * dN7T * FP * N1Tm1 * ddN5T -
                          ddN2T0 * ddN6T * N5Tm2 * dN3T * IP * dN0T0 * N1Tm1 +
                          N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * FA -
                          ddN3T0 * ddN7T * N2Tm1 * N5Tm2 * dN6T * dN1T0 * FP +
                          N2Tm1 * N5Tm2 * dN6T * IP * dN1T0 * ddN3T * ddN0T0 -
                          N6Tm2 * N2Tm1 * IP * dN5T * dN1T0 * ddN3T * ddN0T0 -
                          N2Tm1 * N5Tm2 * IA * dN6T * dN1T0 * ddN3T -
                          N3Tm1 * IA * dN6T * dN1T0 * ddN5T * N2Tm2 -
                          N6Tm2 * IA * dN2T0 * dN5T * ddN3T * N1Tm1 +
                          dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * MidP2 -
                          ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * MidP2) /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T);
  m_control_points[5] = -(ddN2T0 * N4Tm1 * ddN6T * dN1T0 * FS * N3Tm2 -
                          N3Tm1 * ddN2T0 * N6Tm2 * dN1T0 * dN7T * FP * ddN4T -
                          N6Tm2 * N2Tm1 * ddN3T * FS * ddN1T0 * dN4T0 +
                          N6Tm2 * dN3T * IP * dN2T0 * N1Tm1 * ddN0T0 * ddN4T -
                          ddN6T * dN3T * ddN4T0 * IP * dN0T0 * N1Tm1 * N2Tm2 -
                          ddN3T0 * ddN6T * dN2T0 * FS * N1Tm1 * N4Tm2 -
                          ddN2T0 * N4Tm1 * dN6T * dN1T0 * FA * N3Tm2 +
                          dN6T * IP * dN2T0 * ddN3T * N1Tm1 * N4Tm2 * ddN0T0 -
                          ddN2T0 * ddN6T * dN3T * IS * N1Tm1 * N4Tm2 +
                          N6Tm2 * ddN4T0 * dN2T0 * ddN3T * dN7T * FP * N1Tm1 +
                          ddN3T0 * dN6T * dN2T0 * FA * N1Tm1 * N4Tm2 +
                          N3Tm1 * IA * dN6T * dN1T0 * N2Tm2 * ddN4T -
                          ddN7T * N3Tm1 * ddN2T0 * dN6T * dN1T0 * FP * N4Tm2 +
                          dN3T0 * dN4T * N2Tm1 * ddN6T * ddN1T0 * MidP2 +
                          dN4T * N6Tm2 * N2Tm1 * IP * dN1T0 * ddN3T * ddN0T0 -
                          ddN3T0 * N6Tm2 * N2Tm1 * dN1T0 * FS * ddN4T +
                          dN3T0 * N2Tm1 * ddN6T * dN7T * ddN1T0 * FP * N4Tm2 -
                          dN4T * ddN2T0 * N6Tm2 * IS * ddN3T * N1Tm1 +
                          ddN3T0 * ddN7T * dN6T * FP * N1Tm1 * N2Tm2 * dN4T0 +
                          dN4T * ddN2T0 * ddN6T * IS * N1Tm1 * N3Tm2 -
                          ddN6T * dN3T * ddN4T0 * dN2T0 * N1Tm1 * MidP2 -
                          N3Tm1 * dN6T * dN2T0 * FA * ddN1T0 * N4Tm2 +
                          ddN3T0 * N2Tm1 * dN6T * dN1T0 * MidP2 * ddN4T -
                          N6Tm2 * N2Tm1 * dN3T * IS * ddN1T0 * ddN4T +
                          dN3T0 * ddN2T0 * dN6T * N1Tm1 * MidP2 * ddN4T +
                          ddN2T0 * IS * dN6T * ddN3T * N1Tm1 * N4Tm2 -
                          ddN7T * N2Tm1 * dN6T * ddN4T0 * dN1T0 * FP * N3Tm2 +
                          N2Tm1 * ddN6T * dN3T * IP * dN1T0 * N4Tm2 * ddN0T0 -
                          N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * FA -
                          ddN7T * N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * FP -
                          N2Tm1 * ddN6T * dN7T * ddN1T0 * FP * N3Tm2 * dN4T0 +
                          N3Tm1 * ddN6T * dN2T0 * FS * ddN1T0 * N4Tm2 -
                          N6Tm2 * IA * dN3T * dN2T0 * N1Tm1 * ddN4T +
                          dN3T0 * ddN6T * ddN4T0 * dN7T * FP * N1Tm1 * N2Tm2 -
                          N3Tm1 * dN4T * ddN6T * IP * dN0T0 * ddN1T0 * N2Tm2 +
                          N2Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * MidP2 -
                          dN6T * ddN3T * ddN1T0 * MidP1 * N2Tm2 * dN4T0 +
                          N3Tm1 * dN4T * ddN6T * IS * ddN1T0 * N2Tm2 +
                          dN3T0 * N4Tm1 * ddN6T * FS * ddN1T0 * N2Tm2 -
                          ddN7T * N6Tm2 * N2Tm1 * dN3T * ddN1T0 * FP * dN4T0 +
                          ddN7T * N3Tm1 * dN6T * ddN4T0 * dN1T0 * FP * N2Tm2 -
                          ddN3T0 * ddN7T * dN6T * dN2T0 * FP * N1Tm1 * N4Tm2 +
                          dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 * MidP1 +
                          dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T * MidP1 +
                          N4Tm1 * IS * dN6T * ddN3T * ddN1T0 * N2Tm2 +
                          N3Tm1 * dN6T * dN2T0 * ddN1T0 * MidP2 * ddN4T -
                          ddN2T0 * N4Tm1 * ddN6T * dN1T0 * dN7T * FP * N3Tm2 -
                          dN6T * dN2T0 * ddN1T0 * N3Tm2 * MidP1 * ddN4T -
                          IA * dN6T * dN2T0 * ddN3T * N1Tm1 * N4Tm2 +
                          dN4T * N2Tm1 * ddN6T * IP * dN0T0 * ddN1T0 * N3Tm2 +
                          ddN2T0 * N4Tm1 * N6Tm2 * dN1T0 * ddN3T * dN7T * FP -
                          ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 * MidP1 -
                          ddN2T0 * N4Tm1 * N6Tm2 * dN1T0 * ddN3T * FS +
                          dN4T * N2Tm1 * ddN6T * IA * dN1T0 * N3Tm2 +
                          N6Tm2 * N2Tm1 * dN3T * IP * dN0T0 * ddN1T0 * ddN4T -
                          N4Tm1 * ddN6T * dN3T * IS * ddN1T0 * N2Tm2 -
                          ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 * MidP1 +
                          dN3T0 * ddN7T * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * FP +
                          N4Tm1 * ddN6T * dN3T * IP * dN0T0 * ddN1T0 * N2Tm2 -
                          dN3T0 * N2Tm1 * ddN6T * FS * ddN1T0 * N4Tm2 +
                          ddN7T * N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * FP +
                          N6Tm2 * dN3T * ddN4T0 * dN2T0 * FA * N1Tm1 -
                          N6Tm2 * ddN4T0 * dN2T0 * ddN3T * FS * N1Tm1 -
                          ddN3T0 * dN4T * ddN6T * IS * N1Tm1 * N2Tm2 -
                          dN6T * IP * dN2T0 * N1Tm1 * N3Tm2 * ddN0T0 * ddN4T +
                          N3Tm1 * ddN2T0 * dN6T * dN1T0 * FA * N4Tm2 +
                          dN3T0 * ddN7T * ddN2T0 * dN6T * FP * N1Tm1 * N4Tm2 -
                          N4Tm1 * IA * dN6T * dN1T0 * ddN3T * N2Tm2 -
                          N4Tm1 * ddN6T * dN3T * IP * dN1T0 * N2Tm2 * ddN0T0 +
                          N6Tm2 * N2Tm1 * ddN4T0 * dN1T0 * ddN3T * FS -
                          N3Tm1 * ddN2T0 * ddN6T * dN1T0 * FS * N4Tm2 -
                          N3Tm1 * dN6T * IP * dN1T0 * N2Tm2 * ddN0T0 * ddN4T -
                          ddN3T0 * ddN7T * N4Tm1 * dN6T * dN1T0 * FP * N2Tm2 -
                          ddN2T0 * N6Tm2 * dN3T * dN1T0 * MidP1 * ddN4T -
                          N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * FA -
                          dN3T0 * ddN2T0 * dN6T * FA * N1Tm1 * N4Tm2 -
                          dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 * MidP1 +
                          ddN7T * ddN2T0 * N4Tm1 * dN6T * dN1T0 * FP * N3Tm2 -
                          dN4T * N6Tm2 * N2Tm1 * IA * dN1T0 * ddN3T +
                          ddN3T0 * dN4T * ddN6T * IP * dN0T0 * N1Tm1 * N2Tm2 -
                          ddN7T * N4Tm1 * dN6T * dN2T0 * ddN1T0 * FP * N3Tm2 -
                          ddN2T0 * dN6T * ddN3T * N1Tm1 * MidP2 * dN4T0 +
                          dN3T0 * ddN2T0 * N6Tm2 * dN7T * FP * N1Tm1 * ddN4T -
                          ddN6T * dN3T * ddN4T0 * dN1T0 * MidP1 * N2Tm2 -
                          ddN3T0 * ddN7T * dN4T * N6Tm2 * N2Tm1 * dN1T0 * FP -
                          N2Tm1 * ddN6T * dN3T * ddN1T0 * MidP2 * dN4T0 +
                          N6Tm2 * dN3T * dN2T0 * ddN1T0 * MidP1 * ddN4T -
                          N3Tm1 * ddN6T * ddN4T0 * dN1T0 * dN7T * FP * N2Tm2 +
                          dN4T * N6Tm2 * N2Tm1 * IS * ddN3T * ddN1T0 -
                          ddN3T0 * dN6T * dN2T0 * N1Tm1 * MidP2 * ddN4T -
                          ddN6T * IA * dN3T * N1Tm1 * N2Tm2 * dN4T0 -
                          ddN3T0 * dN4T * N2Tm1 * ddN6T * dN1T0 * MidP2 +
                          ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 * MidP1 -
                          ddN3T0 * dN4T * N6Tm2 * dN2T0 * FA * N1Tm1 -
                          ddN2T0 * N4Tm1 * ddN6T * dN3T * dN1T0 * MidP2 -
                          N3Tm1 * ddN6T * dN2T0 * dN7T * ddN1T0 * FP * N4Tm2 -
                          N2Tm1 * ddN6T * IA * dN3T * dN1T0 * N4Tm2 +
                          N4Tm1 * ddN6T * dN2T0 * dN7T * ddN1T0 * FP * N3Tm2 -
                          N4Tm1 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 * N2Tm2 +
                          dN4T * N6Tm2 * IA * dN2T0 * ddN3T * N1Tm1 +
                          ddN3T0 * ddN7T * N2Tm1 * dN6T * dN1T0 * FP * N4Tm2 +
                          ddN3T0 * ddN6T * dN2T0 * dN7T * FP * N1Tm1 * N4Tm2 +
                          N2Tm1 * dN6T * ddN4T0 * dN1T0 * FA * N3Tm2 -
                          ddN3T0 * N2Tm1 * ddN6T * dN1T0 * dN7T * FP * N4Tm2 -
                          ddN2T0 * N6Tm2 * dN3T * IP * dN0T0 * N1Tm1 * ddN4T +
                          dN4T * ddN2T0 * N6Tm2 * IP * ddN3T * dN0T0 * N1Tm1 -
                          N2Tm1 * dN6T * FA * ddN1T0 * N3Tm2 * dN4T0 -
                          N4Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * MidP2 +
                          N3Tm1 * dN4T * ddN6T * IP * dN1T0 * N2Tm2 * ddN0T0 +
                          dN3T0 * dN6T * ddN4T0 * FA * N1Tm1 * N2Tm2 -
                          ddN2T0 * N6Tm2 * dN3T * FA * N1Tm1 * dN4T0 +
                          N3Tm1 * ddN2T0 * N6Tm2 * dN1T0 * FS * ddN4T +
                          ddN7T * N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * FP -
                          N3Tm1 * dN6T * ddN4T0 * dN1T0 * FA * N2Tm2 -
                          dN3T0 * ddN6T * ddN4T0 * FS * N1Tm1 * N2Tm2 -
                          dN3T0 * ddN7T * dN6T * ddN4T0 * FP * N1Tm1 * N2Tm2 -
                          dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 * MidP1 -
                          dN6T * ddN4T0 * dN2T0 * FA * N1Tm1 * N3Tm2 +
                          N2Tm1 * dN6T * IP * dN1T0 * N3Tm2 * ddN0T0 * ddN4T -
                          dN6T * IP * ddN3T * N1Tm1 * N2Tm2 * ddN0T0 * dN4T0 -
                          ddN7T * N3Tm1 * dN6T * ddN1T0 * FP * N2Tm2 * dN4T0 -
                          N6Tm2 * N2Tm1 * dN3T * IP * dN1T0 * ddN0T0 * ddN4T +
                          ddN6T * ddN4T0 * dN2T0 * FS * N1Tm1 * N3Tm2 +
                          N2Tm1 * ddN6T * FS * ddN1T0 * N3Tm2 * dN4T0 -
                          ddN7T * N6Tm2 * dN3T * ddN4T0 * dN2T0 * FP * N1Tm1 -
                          dN3T0 * dN4T * ddN6T * ddN1T0 * MidP1 * N2Tm2 +
                          dN6T * ddN4T0 * dN1T0 * ddN3T * MidP1 * N2Tm2 -
                          N4Tm1 * N6Tm2 * dN2T0 * ddN3T * dN7T * ddN1T0 * FP +
                          dN3T0 * dN4T * ddN2T0 * N6Tm2 * FA * N1Tm1 +
                          N2Tm1 * IS * dN6T * ddN1T0 * N3Tm2 * ddN4T +
                          N3Tm1 * ddN6T * dN7T * ddN1T0 * FP * N2Tm2 * dN4T0 +
                          N3Tm1 * dN4T * N6Tm2 * dN2T0 * FA * ddN1T0 -
                          ddN2T0 * ddN6T * FS * N1Tm1 * N3Tm2 * dN4T0 -
                          N4Tm1 * ddN6T * dN2T0 * FS * ddN1T0 * N3Tm2 +
                          ddN6T * dN3T * IS * ddN4T0 * N1Tm1 * N2Tm2 +
                          N3Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * MidP2 +
                          dN3T0 * dN6T * IP * N1Tm1 * N2Tm2 * ddN0T0 * ddN4T -
                          N3Tm1 * N6Tm2 * dN2T0 * FS * ddN1T0 * ddN4T +
                          ddN2T0 * N6Tm2 * ddN3T * FS * N1Tm1 * dN4T0 -
                          N2Tm1 * IS * dN6T * ddN3T * ddN1T0 * N4Tm2 -
                          ddN3T0 * dN6T * FA * N1Tm1 * N2Tm2 * dN4T0 -
                          ddN6T * ddN4T0 * dN2T0 * dN7T * FP * N1Tm1 * N3Tm2 +
                          dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 * MidP1 -
                          ddN2T0 * IS * dN6T * N1Tm1 * N3Tm2 * ddN4T -
                          dN3T0 * N2Tm1 * dN6T * ddN1T0 * MidP2 * ddN4T +
                          N4Tm1 * N6Tm2 * dN2T0 * ddN3T * FS * ddN1T0 -
                          IS * dN6T * ddN4T0 * ddN3T * N1Tm1 * N2Tm2 -
                          ddN3T0 * dN6T * IP * dN0T0 * N1Tm1 * N2Tm2 * ddN4T +
                          N4Tm1 * dN6T * dN2T0 * FA * ddN1T0 * N3Tm2 +
                          ddN2T0 * dN6T * dN1T0 * N3Tm2 * MidP1 * ddN4T +
                          ddN2T0 * N6Tm2 * dN3T * IS * N1Tm1 * ddN4T -
                          N2Tm1 * IA * dN6T * dN1T0 * N3Tm2 * ddN4T +
                          ddN2T0 * ddN6T * dN7T * FP * N1Tm1 * N3Tm2 * dN4T0 +
                          N2Tm1 * dN6T * IP * ddN3T * dN0T0 * ddN1T0 * N4Tm2 +
                          dN3T0 * N6Tm2 * N2Tm1 * FS * ddN1T0 * ddN4T +
                          ddN3T0 * ddN6T * FS * N1Tm1 * N2Tm2 * dN4T0 +
                          ddN2T0 * dN6T * FA * N1Tm1 * N3Tm2 * dN4T0 +
                          N2Tm1 * ddN6T * dN3T * IS * ddN1T0 * N4Tm2 +
                          N3Tm1 * ddN2T0 * ddN6T * dN1T0 * dN7T * FP * N4Tm2 +
                          N2Tm1 * ddN6T * ddN4T0 * dN1T0 * dN7T * FP * N3Tm2 +
                          ddN3T0 * IS * dN6T * N1Tm1 * N2Tm2 * ddN4T +
                          ddN3T0 * N6Tm2 * dN2T0 * FS * N1Tm1 * ddN4T -
                          dN4T * N6Tm2 * IP * dN2T0 * ddN3T * N1Tm1 * ddN0T0 -
                          dN3T0 * N6Tm2 * N2Tm1 * dN7T * ddN1T0 * FP * ddN4T +
                          N6Tm2 * N2Tm1 * ddN3T * dN7T * ddN1T0 * FP * dN4T0 +
                          dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 * MidP2 -
                          ddN6T * dN3T * IP * dN2T0 * N1Tm1 * N4Tm2 * ddN0T0 -
                          ddN3T0 * N2Tm1 * dN6T * dN1T0 * FA * N4Tm2 -
                          ddN2T0 * N6Tm2 * ddN3T * dN7T * FP * N1Tm1 * dN4T0 +
                          ddN3T0 * N2Tm1 * ddN6T * dN1T0 * FS * N4Tm2 +
                          ddN2T0 * ddN6T * dN3T * N1Tm1 * MidP2 * dN4T0 -
                          dN3T0 * ddN7T * dN4T * ddN2T0 * N6Tm2 * FP * N1Tm1 +
                          ddN3T0 * ddN7T * dN4T * N6Tm2 * dN2T0 * FP * N1Tm1 -
                          N3Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * MidP2 +
                          N4Tm1 * dN6T * IP * dN1T0 * ddN3T * N2Tm2 * ddN0T0 -
                          dN3T0 * N4Tm1 * dN6T * FA * ddN1T0 * N2Tm2 -
                          ddN3T0 * N6Tm2 * dN2T0 * dN7T * FP * N1Tm1 * ddN4T +
                          N3Tm1 * N6Tm2 * dN2T0 * dN7T * ddN1T0 * FP * ddN4T +
                          IA * dN6T * dN2T0 * N1Tm1 * N3Tm2 * ddN4T +
                          dN3T0 * N2Tm1 * dN6T * FA * ddN1T0 * N4Tm2 +
                          ddN7T * N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * FP +
                          ddN2T0 * ddN6T * dN3T * IP * dN0T0 * N1Tm1 * N4Tm2 -
                          ddN7T * ddN2T0 * dN6T * FP * N1Tm1 * N3Tm2 * dN4T0 +
                          ddN6T * dN3T * IP * N1Tm1 * N2Tm2 * ddN0T0 * dN4T0 +
                          ddN3T0 * dN4T * ddN6T * dN2T0 * N1Tm1 * MidP2 -
                          ddN3T0 * N4Tm1 * ddN6T * dN1T0 * FS * N2Tm2 +
                          N2Tm1 * dN6T * ddN3T * ddN1T0 * MidP2 * dN4T0 +
                          IA * dN6T * ddN3T * N1Tm1 * N2Tm2 * dN4T0 -
                          N2Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * MidP2 +
                          dN3T0 * dN4T * ddN6T * IA * N1Tm1 * N2Tm2 +
                          N3Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N2Tm2 * ddN4T +
                          N4Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * MidP2 -
                          dN3T0 * N4Tm1 * ddN6T * dN7T * ddN1T0 * FP * N2Tm2 -
                          dN4T * ddN2T0 * ddN6T * IP * dN0T0 * N1Tm1 * N3Tm2 +
                          ddN3T0 * N4Tm1 * dN6T * dN1T0 * FA * N2Tm2 +
                          ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * FA -
                          dN3T0 * dN4T * ddN2T0 * ddN6T * N1Tm1 * MidP2 +
                          dN6T * ddN4T0 * IP * ddN3T * dN0T0 * N1Tm1 * N2Tm2 +
                          dN3T0 * ddN2T0 * ddN6T * FS * N1Tm1 * N4Tm2 -
                          dN3T0 * dN4T * N6Tm2 * N2Tm1 * FA * ddN1T0 +
                          ddN3T0 * N6Tm2 * N2Tm1 * dN1T0 * dN7T * FP * ddN4T +
                          ddN7T * N3Tm1 * dN6T * dN2T0 * ddN1T0 * FP * N4Tm2 +
                          N6Tm2 * N2Tm1 * IA * dN3T * dN1T0 * ddN4T -
                          N4Tm1 * N6Tm2 * dN3T * dN2T0 * FA * ddN1T0 -
                          dN3T0 * ddN2T0 * ddN6T * dN7T * FP * N1Tm1 * N4Tm2 -
                          ddN2T0 * dN6T * IP * ddN3T * dN0T0 * N1Tm1 * N4Tm2 -
                          N2Tm1 * ddN6T * ddN4T0 * dN1T0 * FS * N3Tm2 -
                          N6Tm2 * N2Tm1 * ddN4T0 * dN1T0 * ddN3T * dN7T * FP +
                          ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * FA -
                          N2Tm1 * dN6T * IP * dN0T0 * ddN1T0 * N3Tm2 * ddN4T -
                          dN3T0 * dN4T * ddN6T * IP * N1Tm1 * N2Tm2 * ddN0T0 -
                          dN4T * ddN6T * IA * dN2T0 * N1Tm1 * N3Tm2 -
                          ddN7T * ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * FP -
                          ddN3T0 * ddN6T * dN7T * FP * N1Tm1 * N2Tm2 * dN4T0 -
                          dN3T0 * ddN2T0 * N6Tm2 * FS * N1Tm1 * ddN4T +
                          ddN7T * dN6T * ddN4T0 * dN2T0 * FP * N1Tm1 * N3Tm2 +
                          ddN2T0 * N4Tm1 * dN6T * dN1T0 * ddN3T * MidP2 -
                          N3Tm1 * IS * dN6T * ddN1T0 * N2Tm2 * ddN4T +
                          N6Tm2 * N2Tm1 * dN3T * FA * ddN1T0 * dN4T0 -
                          dN4T * N2Tm1 * ddN6T * IS * ddN1T0 * N3Tm2 +
                          dN4T * ddN6T * IP * dN2T0 * N1Tm1 * N3Tm2 * ddN0T0 -
                          dN3T0 * ddN7T * N2Tm1 * dN6T * ddN1T0 * FP * N4Tm2 -
                          dN4T * N6Tm2 * N2Tm1 * IP * ddN3T * dN0T0 * ddN1T0 +
                          N3Tm1 * ddN6T * ddN4T0 * dN1T0 * FS * N2Tm2 -
                          dN3T0 * IA * dN6T * N1Tm1 * N2Tm2 * ddN4T +
                          N2Tm1 * IA * dN6T * dN1T0 * ddN3T * N4Tm2 +
                          ddN3T0 * dN4T * ddN6T * dN1T0 * MidP1 * N2Tm2 -
                          ddN3T0 * dN6T * dN1T0 * MidP1 * N2Tm2 * ddN4T -
                          N3Tm1 * ddN6T * FS * ddN1T0 * N2Tm2 * dN4T0 +
                          ddN7T * N2Tm1 * dN6T * ddN1T0 * FP * N3Tm2 * dN4T0 +
                          ddN6T * dN3T * ddN1T0 * MidP1 * N2Tm2 * dN4T0 +
                          ddN3T0 * N4Tm1 * ddN6T * dN1T0 * dN7T * FP * N2Tm2 -
                          N2Tm1 * dN6T * IP * dN1T0 * ddN3T * N4Tm2 * ddN0T0 -
                          N2Tm1 * ddN6T * dN3T * IP * dN0T0 * ddN1T0 * N4Tm2 +
                          ddN6T * IA * dN3T * dN2T0 * N1Tm1 * N4Tm2 +
                          dN3T0 * ddN7T * N4Tm1 * dN6T * ddN1T0 * FP * N2Tm2 +
                          ddN7T * ddN2T0 * N6Tm2 * dN3T * FP * N1Tm1 * dN4T0 +
                          N4Tm1 * ddN6T * IA * dN3T * dN1T0 * N2Tm2 -
                          N3Tm1 * ddN2T0 * dN6T * dN1T0 * MidP2 * ddN4T -
                          dN4T * N2Tm1 * ddN6T * IP * dN1T0 * N3Tm2 * ddN0T0 +
                          dN3T0 * dN6T * ddN1T0 * MidP1 * N2Tm2 * ddN4T +
                          N3Tm1 * dN6T * FA * ddN1T0 * N2Tm2 * dN4T0 +
                          ddN2T0 * dN6T * IP * dN0T0 * N1Tm1 * N3Tm2 * ddN4T -
                          N3Tm1 * dN4T * ddN6T * IA * dN1T0 * N2Tm2) /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T);
  m_control_points[6] = -(ddN7T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * FP * N1Tm1 +
                          dN3T0 * ddN7T * dN4T * ddN2T0 * N5Tm2 * FP * N1Tm1 +
                          IP * dN5T * ddN3T * N1Tm1 * N2Tm2 * ddN0T0 * dN4T0 +
                          dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T * MidP2 -
                          N5Tm1 * dN2T0 * FS * ddN1T0 * N3Tm2 * ddN4T +
                          ddN3T0 * N5Tm1 * dN4T * dN1T0 * FA * N2Tm2 -
                          N3Tm1 * IP * dN5T * dN0T0 * ddN1T0 * N2Tm2 * ddN4T +
                          N4Tm1 * N5Tm2 * dN2T0 * ddN3T * dN7T * ddN1T0 * FP +
                          ddN3T0 * dN4T * N5Tm2 * dN2T0 * FA * N1Tm1 -
                          ddN7T * N4Tm1 * N5Tm2 * dN3T * dN2T0 * ddN1T0 * FP -
                          N4Tm1 * dN2T0 * dN5T * FA * ddN1T0 * N3Tm2 +
                          dN3T0 * dN4T * IP * N1Tm1 * ddN5T * N2Tm2 * ddN0T0 +
                          IS * ddN4T0 * dN5T * ddN3T * N1Tm1 * N2Tm2 -
                          dN3T0 * dN4T * IA * N1Tm1 * ddN5T * N2Tm2 -
                          N5Tm1 * dN4T * dN2T0 * ddN3T * ddN1T0 * MidP2 -
                          N5Tm1 * ddN2T0 * dN3T * dN1T0 * MidP2 * ddN4T -
                          N4Tm1 * dN3T * IP * dN0T0 * ddN1T0 * ddN5T * N2Tm2 -
                          N4Tm1 * IP * dN5T * dN1T0 * ddN3T * N2Tm2 * ddN0T0 -
                          dN4T * ddN2T0 * IS * N1Tm1 * N3Tm2 * ddN5T -
                          N2Tm1 * IS * dN5T * ddN1T0 * N3Tm2 * ddN4T -
                          dN3T0 * N5Tm1 * dN4T * FA * ddN1T0 * N2Tm2 +
                          N5Tm1 * dN3T * IP * dN0T0 * ddN1T0 * N2Tm2 * ddN4T +
                          dN2T0 * dN5T * ddN1T0 * N3Tm2 * MidP1 * ddN4T -
                          ddN7T * ddN2T0 * N4Tm1 * dN5T * dN1T0 * FP * N3Tm2 -
                          N3Tm1 * dN4T * N5Tm2 * dN2T0 * FA * ddN1T0 +
                          ddN3T0 * N2Tm1 * dN1T0 * dN7T * FP * ddN5T * N4Tm2 +
                          dN3T0 * ddN2T0 * dN5T * FA * N1Tm1 * N4Tm2 -
                          ddN7T * N5Tm1 * dN3T * ddN1T0 * FP * N2Tm2 * dN4T0 -
                          N5Tm2 * dN3T * dN2T0 * ddN1T0 * MidP1 * ddN4T -
                          ddN3T0 * N5Tm2 * dN2T0 * FS * N1Tm1 * ddN4T +
                          dN3T0 * ddN4T0 * FS * N1Tm1 * ddN5T * N2Tm2 -
                          N3Tm1 * ddN2T0 * dN5T * dN1T0 * FA * N4Tm2 +
                          ddN2T0 * N5Tm2 * dN3T * IP * dN0T0 * N1Tm1 * ddN4T -
                          ddN2T0 * N5Tm2 * dN3T * IS * N1Tm1 * ddN4T -
                          N2Tm1 * IP * dN5T * ddN3T * dN0T0 * ddN1T0 * N4Tm2 +
                          N2Tm1 * N5Tm2 * dN3T * IP * dN1T0 * ddN0T0 * ddN4T +
                          ddN4T0 * dN2T0 * dN5T * FA * N1Tm1 * N3Tm2 +
                          ddN3T0 * ddN7T * N4Tm1 * dN5T * dN1T0 * FP * N2Tm2 -
                          N3Tm1 * dN5T * FA * ddN1T0 * N2Tm2 * dN4T0 -
                          N5Tm1 * dN4T * ddN2T0 * dN1T0 * FA * N3Tm2 +
                          dN3T0 * dN4T * N2Tm1 * N5Tm2 * FA * ddN1T0 -
                          dN3T0 * dN5T * ddN1T0 * MidP1 * N2Tm2 * ddN4T +
                          N4Tm1 * IA * dN5T * dN1T0 * ddN3T * N2Tm2 -
                          dN3T0 * ddN7T * ddN2T0 * dN5T * FP * N1Tm1 * N4Tm2 -
                          N4Tm1 * N5Tm2 * dN2T0 * ddN3T * FS * ddN1T0 -
                          ddN3T0 * ddN7T * N2Tm1 * dN5T * dN1T0 * FP * N4Tm2 -
                          ddN3T0 * dN4T * N2Tm1 * N5Tm2 * dN1T0 * FA +
                          dN3T * ddN4T0 * IP * dN0T0 * N1Tm1 * ddN5T * N2Tm2 +
                          dN3T0 * N2Tm1 * N5Tm2 * dN7T * ddN1T0 * FP * ddN4T -
                          IA * dN2T0 * dN5T * N1Tm1 * N3Tm2 * ddN4T +
                          ddN3T0 * IP * dN5T * dN0T0 * N1Tm1 * N2Tm2 * ddN4T +
                          ddN3T0 * dN7T * FP * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                          N2Tm1 * ddN4T0 * dN1T0 * dN7T * FP * N3Tm2 * ddN5T +
                          ddN2T0 * dN3T * IS * N1Tm1 * ddN5T * N4Tm2 -
                          ddN7T * N2Tm1 * dN5T * ddN1T0 * FP * N3Tm2 * dN4T0 +
                          dN3T * IP * dN2T0 * N1Tm1 * ddN5T * N4Tm2 * ddN0T0 +
                          ddN3T0 * dN5T * dN1T0 * MidP1 * N2Tm2 * ddN4T -
                          dN4T * N2Tm1 * IA * dN1T0 * N3Tm2 * ddN5T +
                          IA * dN3T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                          N3Tm1 * ddN2T0 * N5Tm2 * dN1T0 * FS * ddN4T -
                          N5Tm1 * dN3T * ddN4T0 * dN1T0 * FA * N2Tm2 -
                          ddN7T * N5Tm1 * dN4T * dN2T0 * ddN1T0 * FP * N3Tm2 -
                          ddN2T0 * N5Tm2 * ddN3T * FS * N1Tm1 * dN4T0 -
                          dN3T0 * IP * dN5T * N1Tm1 * N2Tm2 * ddN0T0 * ddN4T -
                          N3Tm1 * dN2T0 * FS * ddN1T0 * ddN5T * N4Tm2 +
                          ddN3T0 * dN4T * IS * N1Tm1 * ddN5T * N2Tm2 +
                          N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T * MidP2 +
                          N3Tm1 * N5Tm2 * dN2T0 * FS * ddN1T0 * ddN4T -
                          N3Tm1 * ddN4T0 * dN1T0 * FS * ddN5T * N2Tm2 -
                          N3Tm1 * dN4T * IS * ddN1T0 * ddN5T * N2Tm2 -
                          N5Tm1 * dN3T * dN2T0 * FA * ddN1T0 * N4Tm2 -
                          N4Tm1 * IS * dN5T * ddN3T * ddN1T0 * N2Tm2 -
                          N5Tm1 * ddN2T0 * dN1T0 * dN7T * FP * N3Tm2 * ddN4T +
                          ddN2T0 * N4Tm1 * dN3T * dN1T0 * ddN5T * MidP2 +
                          ddN3T0 * dN2T0 * FS * N1Tm1 * ddN5T * N4Tm2 -
                          ddN2T0 * dN5T * FA * N1Tm1 * N3Tm2 * dN4T0 +
                          ddN7T * N3Tm1 * dN4T * N5Tm2 * dN2T0 * ddN1T0 * FP -
                          dN3T0 * N5Tm1 * dN7T * ddN1T0 * FP * N2Tm2 * ddN4T -
                          dN3T0 * N2Tm1 * N5Tm2 * FS * ddN1T0 * ddN4T -
                          ddN2T0 * dN5T * dN1T0 * N3Tm2 * MidP1 * ddN4T -
                          N5Tm1 * ddN2T0 * dN1T0 * ddN3T * FS * N4Tm2 -
                          ddN2T0 * IP * dN5T * dN0T0 * N1Tm1 * N3Tm2 * ddN4T +
                          dN4T * N2Tm1 * N5Tm2 * IP * ddN3T * dN0T0 * ddN1T0 +
                          N2Tm1 * IP * dN5T * dN1T0 * ddN3T * N4Tm2 * ddN0T0 +
                          dN3T0 * N4Tm1 * dN7T * ddN1T0 * FP * ddN5T * N2Tm2 -
                          N2Tm1 * N5Tm2 * dN3T * FA * ddN1T0 * dN4T0 +
                          N5Tm2 * ddN4T0 * dN2T0 * ddN3T * FS * N1Tm1 -
                          ddN2T0 * dN7T * FP * N1Tm1 * N3Tm2 * ddN5T * dN4T0 +
                          dN4T * ddN2T0 * N5Tm2 * IS * ddN3T * N1Tm1 +
                          N5Tm1 * dN4T * IP * dN1T0 * ddN3T * N2Tm2 * ddN0T0 +
                          dN3T0 * ddN7T * N5Tm1 * dN4T * ddN1T0 * FP * N2Tm2 +
                          N2Tm1 * IA * dN3T * dN1T0 * ddN5T * N4Tm2 -
                          N5Tm2 * dN3T * ddN4T0 * dN2T0 * FA * N1Tm1 -
                          ddN7T * ddN2T0 * N5Tm2 * dN3T * FP * N1Tm1 * dN4T0 +
                          ddN3T0 * dN5T * FA * N1Tm1 * N2Tm2 * dN4T0 +
                          ddN2T0 * N4Tm1 * dN1T0 * dN7T * FP * N3Tm2 * ddN5T +
                          N3Tm1 * dN4T * IP * dN0T0 * ddN1T0 * ddN5T * N2Tm2 -
                          ddN7T * N3Tm1 * dN2T0 * dN5T * ddN1T0 * FP * N4Tm2 +
                          ddN3T0 * dN4T * N2Tm1 * dN1T0 * ddN5T * MidP2 +
                          dN3T * ddN4T0 * dN1T0 * ddN5T * MidP1 * N2Tm2 +
                          dN4T * N2Tm1 * IP * dN1T0 * N3Tm2 * ddN5T * ddN0T0 +
                          ddN7T * N5Tm1 * dN4T * ddN2T0 * dN1T0 * FP * N3Tm2 -
                          N3Tm1 * IA * dN5T * dN1T0 * N2Tm2 * ddN4T -
                          N5Tm1 * ddN3T * FS * ddN1T0 * N2Tm2 * dN4T0 +
                          dN3T0 * N5Tm1 * FS * ddN1T0 * N2Tm2 * ddN4T -
                          ddN3T0 * dN4T * IP * dN0T0 * N1Tm1 * ddN5T * N2Tm2 +
                          N4Tm1 * dN3T * IS * ddN1T0 * ddN5T * N2Tm2 +
                          ddN7T * ddN2T0 * dN5T * FP * N1Tm1 * N3Tm2 * dN4T0 +
                          N5Tm1 * ddN3T * dN7T * ddN1T0 * FP * N2Tm2 * dN4T0 -
                          dN4T * N2Tm1 * IP * dN0T0 * ddN1T0 * N3Tm2 * ddN5T +
                          N2Tm1 * IP * dN5T * dN0T0 * ddN1T0 * N3Tm2 * ddN4T -
                          ddN3T0 * N2Tm1 * dN1T0 * FS * ddN5T * N4Tm2 +
                          dN4T * ddN2T0 * dN1T0 * N3Tm2 * ddN5T * MidP1 +
                          ddN7T * N3Tm1 * ddN2T0 * dN5T * dN1T0 * FP * N4Tm2 +
                          N5Tm2 * IA * dN3T * dN2T0 * N1Tm1 * ddN4T +
                          ddN2T0 * N4Tm1 * N5Tm2 * dN1T0 * ddN3T * FS +
                          ddN2T0 * IP * dN5T * ddN3T * dN0T0 * N1Tm1 * N4Tm2 -
                          IA * dN5T * ddN3T * N1Tm1 * N2Tm2 * dN4T0 +
                          ddN2T0 * N4Tm1 * dN5T * dN1T0 * FA * N3Tm2 -
                          N2Tm1 * N5Tm2 * dN3T * IP * dN0T0 * ddN1T0 * ddN4T -
                          N2Tm1 * dN5T * ddN3T * ddN1T0 * MidP2 * dN4T0 +
                          dN3T0 * dN4T * ddN2T0 * N1Tm1 * ddN5T * MidP2 -
                          dN3T0 * N4Tm1 * FS * ddN1T0 * ddN5T * N2Tm2 -
                          N5Tm1 * dN3T * IS * ddN1T0 * N2Tm2 * ddN4T +
                          N4Tm1 * N5Tm2 * dN3T * dN2T0 * FA * ddN1T0 +
                          dN3T0 * N2Tm1 * FS * ddN1T0 * ddN5T * N4Tm2 -
                          ddN4T0 * dN5T * dN1T0 * ddN3T * MidP1 * N2Tm2 -
                          dN4T * ddN2T0 * N5Tm2 * dN1T0 * ddN3T * MidP1 +
                          ddN7T * N5Tm1 * dN3T * ddN4T0 * dN1T0 * FP * N2Tm2 -
                          ddN3T0 * N5Tm1 * dN1T0 * FS * N2Tm2 * ddN4T +
                          N2Tm1 * ddN4T0 * dN1T0 * FS * N3Tm2 * ddN5T -
                          N4Tm1 * dN2T0 * dN7T * ddN1T0 * FP * N3Tm2 * ddN5T +
                          N5Tm1 * dN3T * dN2T0 * ddN1T0 * MidP2 * ddN4T +
                          ddN3T0 * N4Tm1 * dN1T0 * FS * ddN5T * N2Tm2 -
                          N2Tm1 * N5Tm2 * IA * dN3T * dN1T0 * ddN4T +
                          N3Tm1 * FS * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                          N3Tm1 * dN2T0 * dN7T * ddN1T0 * FP * ddN5T * N4Tm2 -
                          ddN2T0 * IS * dN5T * ddN3T * N1Tm1 * N4Tm2 -
                          dN3T0 * ddN2T0 * dN5T * N1Tm1 * MidP2 * ddN4T -
                          ddN3T0 * N2Tm1 * N5Tm2 * dN1T0 * dN7T * FP * ddN4T -
                          ddN3T0 * N2Tm1 * dN5T * dN1T0 * MidP2 * ddN4T -
                          ddN2T0 * dN3T * dN1T0 * ddN5T * N4Tm2 * MidP1 -
                          N2Tm1 * IA * dN5T * dN1T0 * ddN3T * N4Tm2 +
                          N5Tm1 * dN4T * IS * ddN3T * ddN1T0 * N2Tm2 +
                          ddN3T0 * N5Tm1 * dN1T0 * dN7T * FP * N2Tm2 * ddN4T +
                          dN3T0 * IA * dN5T * N1Tm1 * N2Tm2 * ddN4T +
                          N5Tm1 * ddN4T0 * dN1T0 * ddN3T * FS * N2Tm2 +
                          dN3T0 * ddN7T * N2Tm1 * dN5T * ddN1T0 * FP * N4Tm2 +
                          ddN7T * N5Tm1 * dN3T * dN2T0 * ddN1T0 * FP * N4Tm2 -
                          dN4T * N2Tm1 * N5Tm2 * IS * ddN3T * ddN1T0 +
                          N3Tm1 * ddN2T0 * dN1T0 * FS * ddN5T * N4Tm2 -
                          dN2T0 * dN5T * ddN3T * ddN1T0 * N4Tm2 * MidP1 -
                          N5Tm1 * ddN4T0 * dN1T0 * ddN3T * dN7T * FP * N2Tm2 +
                          N2Tm1 * N5Tm2 * ddN4T0 * dN1T0 * ddN3T * dN7T * FP +
                          N4Tm1 * dN3T * IP * dN1T0 * ddN5T * N2Tm2 * ddN0T0 +
                          ddN7T * N2Tm1 * N5Tm2 * dN3T * ddN1T0 * FP * dN4T0 +
                          dN4T * N2Tm1 * IS * ddN1T0 * N3Tm2 * ddN5T -
                          ddN2T0 * N4Tm1 * N5Tm2 * dN1T0 * ddN3T * dN7T * FP +
                          dN5T * ddN3T * ddN1T0 * MidP1 * N2Tm2 * dN4T0 +
                          N3Tm1 * IS * dN5T * ddN1T0 * N2Tm2 * ddN4T -
                          ddN3T0 * N4Tm1 * dN1T0 * dN7T * FP * ddN5T * N2Tm2 -
                          ddN2T0 * N4Tm1 * dN5T * dN1T0 * ddN3T * MidP2 -
                          ddN3T0 * dN2T0 * dN5T * FA * N1Tm1 * N4Tm2 -
                          N3Tm1 * dN4T * ddN2T0 * dN1T0 * ddN5T * MidP2 +
                          dN4T * ddN2T0 * IP * dN0T0 * N1Tm1 * N3Tm2 * ddN5T -
                          N3Tm1 * dN7T * ddN1T0 * FP * ddN5T * N2Tm2 * dN4T0 +
                          ddN7T * N2Tm1 * ddN4T0 * dN5T * dN1T0 * FP * N3Tm2 +
                          ddN4T0 * dN2T0 * dN7T * FP * N1Tm1 * N3Tm2 * ddN5T +
                          N3Tm1 * dN2T0 * dN5T * FA * ddN1T0 * N4Tm2 -
                          ddN7T * N2Tm1 * N5Tm2 * dN3T * ddN4T0 * dN1T0 * FP +
                          IP * dN2T0 * dN5T * N1Tm1 * N3Tm2 * ddN0T0 * ddN4T +
                          N5Tm1 * ddN2T0 * dN1T0 * ddN3T * dN7T * FP * N4Tm2 +
                          ddN2T0 * dN5T * ddN3T * N1Tm1 * MidP2 * dN4T0 -
                          N5Tm1 * dN2T0 * ddN3T * dN7T * ddN1T0 * FP * N4Tm2 +
                          N3Tm1 * ddN2T0 * dN5T * dN1T0 * MidP2 * ddN4T -
                          ddN2T0 * N4Tm1 * N5Tm2 * dN3T * dN1T0 * FA +
                          dN4T * N2Tm1 * N5Tm2 * IA * dN1T0 * ddN3T -
                          dN3T0 * ddN7T * N4Tm1 * dN5T * ddN1T0 * FP * N2Tm2 -
                          dN3T * IS * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 -
                          dN4T * N2Tm1 * N5Tm2 * IP * dN1T0 * ddN3T * ddN0T0 +
                          ddN7T * N3Tm1 * dN5T * ddN1T0 * FP * N2Tm2 * dN4T0 -
                          ddN7T * N3Tm1 * ddN4T0 * dN5T * dN1T0 * FP * N2Tm2 -
                          dN3T0 * ddN4T0 * dN7T * FP * N1Tm1 * ddN5T * N2Tm2 +
                          ddN3T0 * ddN7T * dN4T * N2Tm1 * N5Tm2 * dN1T0 * FP +
                          N5Tm1 * ddN2T0 * dN3T * dN1T0 * FA * N4Tm2 +
                          N4Tm1 * dN2T0 * FS * ddN1T0 * N3Tm2 * ddN5T -
                          N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T * MidP2 +
                          IA * dN2T0 * dN5T * ddN3T * N1Tm1 * N4Tm2 +
                          N5Tm1 * dN3T * FA * ddN1T0 * N2Tm2 * dN4T0 +
                          N2Tm1 * N5Tm2 * ddN3T * FS * ddN1T0 * dN4T0 +
                          dN3T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 * MidP1 -
                          dN3T * IP * N1Tm1 * ddN5T * N2Tm2 * ddN0T0 * dN4T0 +
                          ddN3T0 * ddN7T * dN2T0 * dN5T * FP * N1Tm1 * N4Tm2 -
                          ddN3T0 * dN4T * dN2T0 * N1Tm1 * ddN5T * MidP2 -
                          N3Tm1 * dN2T0 * dN5T * ddN1T0 * MidP2 * ddN4T -
                          dN4T * IP * dN2T0 * N1Tm1 * N3Tm2 * ddN5T * ddN0T0 -
                          dN3T * ddN1T0 * ddN5T * MidP1 * N2Tm2 * dN4T0 -
                          N5Tm1 * dN4T * IA * dN1T0 * ddN3T * N2Tm2 +
                          N2Tm1 * N5Tm2 * dN3T * ddN4T0 * dN1T0 * FA +
                          N2Tm1 * dN5T * FA * ddN1T0 * N3Tm2 * dN4T0 -
                          N5Tm1 * dN4T * IP * ddN3T * dN0T0 * ddN1T0 * N2Tm2 +
                          N2Tm1 * N5Tm2 * dN3T * IS * ddN1T0 * ddN4T +
                          N3Tm1 * ddN2T0 * N5Tm2 * dN1T0 * dN7T * FP * ddN4T +
                          ddN7T * ddN2T0 * N4Tm1 * N5Tm2 * dN3T * dN1T0 * FP +
                          dN3T0 * ddN2T0 * dN7T * FP * N1Tm1 * ddN5T * N4Tm2 +
                          N5Tm1 * ddN2T0 * dN1T0 * FS * N3Tm2 * ddN4T -
                          ddN7T * ddN4T0 * dN2T0 * dN5T * FP * N1Tm1 * N3Tm2 +
                          dN3T0 * ddN2T0 * N5Tm2 * FS * N1Tm1 * ddN4T -
                          N2Tm1 * N5Tm2 * ddN3T * dN7T * ddN1T0 * FP * dN4T0 -
                          dN3T0 * dN4T * N2Tm1 * ddN1T0 * ddN5T * MidP2 -
                          ddN2T0 * N4Tm1 * dN1T0 * FS * N3Tm2 * ddN5T +
                          dN3T0 * N4Tm1 * dN5T * FA * ddN1T0 * N2Tm2 -
                          ddN3T0 * ddN7T * dN5T * FP * N1Tm1 * N2Tm2 * dN4T0 +
                          ddN3T0 * N5Tm2 * dN2T0 * dN7T * FP * N1Tm1 * ddN4T -
                          ddN7T * N3Tm1 * dN4T * ddN2T0 * N5Tm2 * dN1T0 * FP -
                          N3Tm1 * N5Tm2 * dN2T0 * dN7T * ddN1T0 * FP * ddN4T -
                          ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 * MidP2 -
                          ddN4T0 * IP * dN5T * ddN3T * dN0T0 * N1Tm1 * N2Tm2 +
                          ddN3T0 * N2Tm1 * dN5T * dN1T0 * FA * N4Tm2 -
                          N3Tm1 * dN4T * IP * dN1T0 * ddN5T * N2Tm2 * ddN0T0 -
                          N2Tm1 * IP * dN5T * dN1T0 * N3Tm2 * ddN0T0 * ddN4T +
                          ddN7T * N4Tm1 * dN2T0 * dN5T * ddN1T0 * FP * N3Tm2 -
                          N2Tm1 * dN3T * IS * ddN1T0 * ddN5T * N4Tm2 -
                          ddN3T0 * FS * N1Tm1 * ddN5T * N2Tm2 * dN4T0 +
                          N5Tm1 * dN4T * ddN2T0 * dN1T0 * ddN3T * MidP2 -
                          ddN2T0 * dN3T * N1Tm1 * ddN5T * MidP2 * dN4T0 +
                          ddN2T0 * N5Tm2 * ddN3T * dN7T * FP * N1Tm1 * dN4T0 +
                          ddN3T0 * dN2T0 * dN5T * N1Tm1 * MidP2 * ddN4T +
                          ddN2T0 * FS * N1Tm1 * N3Tm2 * ddN5T * dN4T0 +
                          N3Tm1 * dN4T * dN2T0 * ddN1T0 * ddN5T * MidP2 -
                          dN3T0 * ddN7T * dN4T * N2Tm1 * N5Tm2 * ddN1T0 * FP -
                          dN3T0 * dN4T * ddN2T0 * N5Tm2 * FA * N1Tm1 -
                          N2Tm1 * dN3T * IP * dN1T0 * ddN5T * N4Tm2 * ddN0T0 -
                          ddN3T0 * N4Tm1 * dN5T * dN1T0 * FA * N2Tm2 +
                          ddN3T0 * N2Tm1 * N5Tm2 * dN1T0 * FS * ddN4T +
                          N5Tm1 * dN2T0 * dN7T * ddN1T0 * FP * N3Tm2 * ddN4T -
                          dN3T0 * N2Tm1 * dN5T * FA * ddN1T0 * N4Tm2 +
                          ddN2T0 * N5Tm2 * dN3T * FA * N1Tm1 * dN4T0 +
                          N2Tm1 * dN3T * IP * dN0T0 * ddN1T0 * ddN5T * N4Tm2 -
                          IA * dN3T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                          IP * dN2T0 * dN5T * ddN3T * N1Tm1 * N4Tm2 * ddN0T0 -
                          N2Tm1 * ddN4T0 * dN5T * dN1T0 * FA * N3Tm2 -
                          dN4T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T * MidP1 -
                          N4Tm1 * IA * dN3T * dN1T0 * ddN5T * N2Tm2 -
                          dN4T * N5Tm2 * IA * dN2T0 * ddN3T * N1Tm1 +
                          ddN2T0 * dN5T * dN1T0 * ddN3T * N4Tm2 * MidP1 +
                          N4Tm1 * IP * dN5T * ddN3T * dN0T0 * ddN1T0 * N2Tm2 +
                          dN3T0 * N2Tm1 * dN5T * ddN1T0 * MidP2 * ddN4T +
                          ddN2T0 * IS * dN5T * N1Tm1 * N3Tm2 * ddN4T -
                          ddN3T0 * dN2T0 * dN7T * FP * N1Tm1 * ddN5T * N4Tm2 -
                          ddN4T0 * dN2T0 * FS * N1Tm1 * N3Tm2 * ddN5T -
                          N5Tm2 * ddN4T0 * dN2T0 * ddN3T * dN7T * FP * N1Tm1 -
                          N5Tm1 * dN3T * IP * dN1T0 * N2Tm2 * ddN0T0 * ddN4T +
                          dN3T0 * dN4T * ddN1T0 * ddN5T * MidP1 * N2Tm2 -
                          dN4T * ddN2T0 * N5Tm2 * IP * ddN3T * dN0T0 * N1Tm1 -
                          dN3T0 * N2Tm1 * dN7T * ddN1T0 * FP * ddN5T * N4Tm2 -
                          dN3T0 * ddN4T0 * dN5T * FA * N1Tm1 * N2Tm2 +
                          N3Tm1 * dN4T * ddN2T0 * N5Tm2 * dN1T0 * FA -
                          N3Tm1 * ddN2T0 * dN1T0 * dN7T * FP * ddN5T * N4Tm2 +
                          N4Tm1 * dN2T0 * dN5T * ddN3T * ddN1T0 * MidP2 +
                          dN4T * N5Tm2 * IP * dN2T0 * ddN3T * N1Tm1 * ddN0T0 -
                          N5Tm2 * dN3T * IP * dN2T0 * N1Tm1 * ddN0T0 * ddN4T -
                          N2Tm1 * N5Tm2 * ddN4T0 * dN1T0 * ddN3T * FS -
                          ddN7T * N5Tm1 * ddN2T0 * dN3T * dN1T0 * FP * N4Tm2 +
                          dN3T0 * ddN7T * ddN4T0 * dN5T * FP * N1Tm1 * N2Tm2 -
                          dN3T0 * ddN2T0 * FS * N1Tm1 * ddN5T * N4Tm2 +
                          N2Tm1 * IS * dN5T * ddN3T * ddN1T0 * N4Tm2 -
                          N2Tm1 * FS * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                          N5Tm1 * IA * dN3T * dN1T0 * N2Tm2 * ddN4T +
                          N3Tm1 * ddN4T0 * dN5T * dN1T0 * FA * N2Tm2 +
                          N2Tm1 * dN3T * ddN1T0 * ddN5T * MidP2 * dN4T0 +
                          dN4T * N5Tm2 * dN2T0 * ddN3T * ddN1T0 * MidP1 -
                          ddN3T0 * ddN7T * dN4T * N5Tm2 * dN2T0 * FP * N1Tm1 -
                          ddN3T0 * IS * dN5T * N1Tm1 * N2Tm2 * ddN4T +
                          N3Tm1 * dN4T * IA * dN1T0 * ddN5T * N2Tm2 -
                          N4Tm1 * dN3T * dN2T0 * ddN1T0 * ddN5T * MidP2 -
                          ddN3T0 * dN4T * dN1T0 * ddN5T * MidP1 * N2Tm2 -
                          dN3T0 * ddN2T0 * N5Tm2 * dN7T * FP * N1Tm1 * ddN4T +
                          N3Tm1 * ddN4T0 * dN1T0 * dN7T * FP * ddN5T * N2Tm2 +
                          N2Tm1 * IA * dN5T * dN1T0 * N3Tm2 * ddN4T -
                          ddN2T0 * dN3T * IP * dN0T0 * N1Tm1 * ddN5T * N4Tm2 +
                          N2Tm1 * dN7T * ddN1T0 * FP * N3Tm2 * ddN5T * dN4T0 +
                          N5Tm1 * dN2T0 * ddN3T * FS * ddN1T0 * N4Tm2 +
                          dN4T * IA * dN2T0 * N1Tm1 * N3Tm2 * ddN5T +
                          N3Tm1 * IP * dN5T * dN1T0 * N2Tm2 * ddN0T0 * ddN4T +
                          ddN2T0 * N5Tm2 * dN3T * dN1T0 * MidP1 * ddN4T +
                          N5Tm1 * dN4T * dN2T0 * FA * ddN1T0 * N3Tm2 -
                          ddN3T0 * ddN7T * N5Tm1 * dN4T * dN1T0 * FP * N2Tm2) /
                        (N3Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 * dN4T0 +
                         N4Tm1 * N5Tm2 * dN6T * dN2T0 * ddN3T * ddN1T0 -
                         ddN2T0 * ddN6T * N5Tm2 * dN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * N6Tm2 * dN5T * N1Tm1 * ddN4T -
                         ddN2T0 * N4Tm1 * N6Tm2 * dN3T * dN1T0 * ddN5T -
                         N2Tm1 * N5Tm2 * dN6T * ddN3T * ddN1T0 * dN4T0 -
                         ddN3T0 * dN6T * dN2T0 * N1Tm1 * ddN5T * N4Tm2 -
                         ddN2T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N3Tm2 -
                         N4Tm1 * ddN6T * N5Tm2 * dN3T * dN2T0 * ddN1T0 -
                         dN3T0 * ddN2T0 * ddN6T * dN5T * N1Tm1 * N4Tm2 -
                         ddN3T0 * dN4T * N6Tm2 * N2Tm1 * dN1T0 * ddN5T -
                         dN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * ddN1T0 +
                         ddN3T0 * N6Tm2 * N2Tm1 * dN5T * dN1T0 * ddN4T +
                         N3Tm1 * ddN2T0 * N5Tm2 * dN6T * dN1T0 * ddN4T -
                         N5Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN3T -
                         dN3T0 * dN6T * ddN4T0 * N1Tm1 * ddN5T * N2Tm2 +
                         ddN2T0 * N6Tm2 * dN3T * N1Tm1 * ddN5T * dN4T0 +
                         N6Tm2 * ddN4T0 * dN2T0 * dN5T * ddN3T * N1Tm1 +
                         N5Tm1 * ddN6T * dN3T * dN2T0 * ddN1T0 * N4Tm2 +
                         dN6T * ddN4T0 * dN2T0 * N1Tm1 * N3Tm2 * ddN5T -
                         N3Tm1 * dN4T * ddN2T0 * ddN6T * N5Tm2 * dN1T0 +
                         N3Tm1 * dN6T * dN2T0 * ddN1T0 * ddN5T * N4Tm2 -
                         N3Tm1 * ddN2T0 * N6Tm2 * dN5T * dN1T0 * ddN4T +
                         N2Tm1 * N5Tm2 * dN6T * ddN4T0 * dN1T0 * ddN3T +
                         N5Tm1 * ddN2T0 * N6Tm2 * dN3T * dN1T0 * ddN4T +
                         N5Tm1 * dN6T * ddN3T * ddN1T0 * N2Tm2 * dN4T0 +
                         dN3T0 * dN4T * N6Tm2 * N2Tm1 * ddN1T0 * ddN5T -
                         dN3T0 * N6Tm2 * N2Tm1 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N6Tm2 * dN2T0 * dN5T * N1Tm1 * ddN4T +
                         ddN6T * N5Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 +
                         N5Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN4T +
                         ddN2T0 * ddN6T * dN5T * N1Tm1 * N3Tm2 * dN4T0 -
                         ddN2T0 * N6Tm2 * dN5T * ddN3T * N1Tm1 * dN4T0 +
                         dN3T0 * ddN2T0 * dN6T * N1Tm1 * ddN5T * N4Tm2 +
                         ddN3T0 * N2Tm1 * dN6T * dN1T0 * ddN5T * N4Tm2 +
                         N3Tm1 * N6Tm2 * dN2T0 * dN5T * ddN1T0 * ddN4T -
                         ddN3T0 * N4Tm1 * dN6T * dN1T0 * ddN5T * N2Tm2 +
                         N4Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN5T +
                         N4Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N3Tm2 +
                         N3Tm1 * ddN2T0 * ddN6T * dN5T * dN1T0 * N4Tm2 -
                         ddN3T0 * ddN6T * dN5T * N1Tm1 * N2Tm2 * dN4T0 -
                         N3Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 * dN4T0 +
                         N5Tm1 * ddN6T * dN3T * ddN4T0 * dN1T0 * N2Tm2 -
                         dN3T0 * ddN2T0 * N5Tm2 * dN6T * N1Tm1 * ddN4T -
                         N5Tm1 * ddN2T0 * ddN6T * dN3T * dN1T0 * N4Tm2 +
                         ddN3T0 * N5Tm1 * dN6T * dN1T0 * N2Tm2 * ddN4T -
                         ddN6T * ddN4T0 * dN2T0 * dN5T * N1Tm1 * N3Tm2 +
                         dN3T0 * N2Tm1 * N5Tm2 * dN6T * ddN1T0 * ddN4T +
                         ddN3T0 * dN4T * N2Tm1 * ddN6T * N5Tm2 * dN1T0 +
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN1T0 * dN4T0 -
                         N5Tm1 * ddN6T * dN3T * ddN1T0 * N2Tm2 * dN4T0 -
                         N3Tm1 * N5Tm2 * dN6T * dN2T0 * ddN1T0 * ddN4T +
                         ddN3T0 * N4Tm1 * ddN6T * dN5T * dN1T0 * N2Tm2 -
                         N3Tm1 * ddN2T0 * dN6T * dN1T0 * ddN5T * N4Tm2 -
                         N2Tm1 * dN6T * ddN4T0 * dN1T0 * N3Tm2 * ddN5T -
                         N5Tm2 * dN6T * ddN4T0 * dN2T0 * ddN3T * N1Tm1 +
                         ddN3T0 * N5Tm2 * dN6T * dN2T0 * N1Tm1 * ddN4T -
                         N5Tm1 * dN6T * dN2T0 * ddN3T * ddN1T0 * N4Tm2 +
                         N3Tm1 * dN4T * ddN6T * N5Tm2 * dN2T0 * ddN1T0 -
                         ddN3T0 * N2Tm1 * ddN6T * dN5T * dN1T0 * N4Tm2 +
                         dN3T0 * ddN6T * ddN4T0 * dN5T * N1Tm1 * N2Tm2 -
                         N2Tm1 * ddN6T * dN5T * ddN1T0 * N3Tm2 * dN4T0 +
                         dN3T0 * N5Tm1 * dN4T * ddN6T * ddN1T0 * N2Tm2 -
                         N6Tm2 * N2Tm1 * dN3T * ddN1T0 * ddN5T * dN4T0 +
                         N3Tm1 * dN6T * ddN4T0 * dN1T0 * ddN5T * N2Tm2 -
                         N6Tm2 * dN3T * ddN4T0 * dN2T0 * N1Tm1 * ddN5T -
                         ddN2T0 * N4Tm1 * N5Tm2 * dN6T * dN1T0 * ddN3T +
                         N3Tm1 * dN4T * ddN2T0 * N6Tm2 * dN1T0 * ddN5T -
                         N6Tm2 * N2Tm1 * ddN4T0 * dN5T * dN1T0 * ddN3T -
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * N3Tm2 * ddN4T -
                         ddN3T0 * dN4T * ddN6T * N5Tm2 * dN2T0 * N1Tm1 +
                         ddN2T0 * N5Tm2 * dN6T * ddN3T * N1Tm1 * dN4T0 +
                         ddN2T0 * N4Tm1 * N6Tm2 * dN5T * dN1T0 * ddN3T +
                         dN3T0 * dN4T * ddN2T0 * ddN6T * N5Tm2 * N1Tm1 -
                         N5Tm1 * dN4T * ddN6T * dN2T0 * ddN1T0 * N3Tm2 -
                         ddN3T0 * N5Tm1 * dN4T * ddN6T * dN1T0 * N2Tm2 +
                         ddN3T0 * dN4T * N6Tm2 * dN2T0 * N1Tm1 * ddN5T -
                         N4Tm1 * dN6T * dN2T0 * ddN1T0 * N3Tm2 * ddN5T -
                         N2Tm1 * ddN6T * N5Tm2 * dN3T * ddN4T0 * dN1T0 +
                         ddN2T0 * N4Tm1 * dN6T * dN1T0 * N3Tm2 * ddN5T +
                         dN3T0 * N2Tm1 * ddN6T * dN5T * ddN1T0 * N4Tm2 +
                         N5Tm1 * ddN2T0 * dN6T * dN1T0 * ddN3T * N4Tm2 +
                         ddN3T0 * dN6T * N1Tm1 * ddN5T * N2Tm2 * dN4T0 -
                         dN3T0 * N4Tm1 * ddN6T * dN5T * ddN1T0 * N2Tm2 -
                         ddN2T0 * dN6T * N1Tm1 * N3Tm2 * ddN5T * dN4T0 -
                         N5Tm1 * N6Tm2 * dN3T * dN2T0 * ddN1T0 * ddN4T -
                         N3Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N2Tm2 -
                         ddN3T0 * N2Tm1 * N5Tm2 * dN6T * dN1T0 * ddN4T +
                         ddN2T0 * N4Tm1 * ddN6T * N5Tm2 * dN3T * dN1T0 -
                         N3Tm1 * dN4T * N6Tm2 * dN2T0 * ddN1T0 * ddN5T -
                         dN3T0 * N5Tm1 * dN6T * ddN1T0 * N2Tm2 * ddN4T -
                         N5Tm1 * dN6T * ddN4T0 * dN1T0 * ddN3T * N2Tm2 +
                         N6Tm2 * N2Tm1 * dN5T * ddN3T * ddN1T0 * dN4T0 -
                         N3Tm1 * ddN6T * dN2T0 * dN5T * ddN1T0 * N4Tm2 +
                         N2Tm1 * ddN6T * ddN4T0 * dN5T * dN1T0 * N3Tm2 +
                         N5Tm1 * dN4T * ddN2T0 * ddN6T * dN1T0 * N3Tm2 +
                         N2Tm1 * dN6T * ddN1T0 * N3Tm2 * ddN5T * dN4T0 +
                         dN3T0 * N4Tm1 * dN6T * ddN1T0 * ddN5T * N2Tm2 -
                         dN3T0 * dN4T * ddN2T0 * N6Tm2 * N1Tm1 * ddN5T +
                         ddN3T0 * ddN6T * dN2T0 * dN5T * N1Tm1 * N4Tm2 +
                         N5Tm1 * dN4T * N6Tm2 * dN2T0 * ddN3T * ddN1T0 -
                         dN3T0 * N2Tm1 * dN6T * ddN1T0 * ddN5T * N4Tm2 -
                         N4Tm1 * N6Tm2 * dN2T0 * dN5T * ddN3T * ddN1T0 +
                         N6Tm2 * N2Tm1 * dN3T * ddN4T0 * dN1T0 * ddN5T);
  m_control_points[7] = FP;

  return;
}

void BSplinesFoot::GetParameters(double &FT, double &IP, double &FP,
                                 vector<double> &ToMP, vector<double> &MP) {
  FT = m_FT;
  FP = m_FP;
  IP = m_IP;
  ToMP = m_ToMP;
  MP = m_MP;
}

void BSplinesFoot::SetParametersWithoutMPAndToMP(double FT, double IP,
                                                 double FP, double IS,
                                                 double IA, double FS,
                                                 double FA) {
  m_FT = FT;

  m_IP = IP;
  m_IS = IS;
  m_IA = IA;

  m_FP = FP;
  m_FS = FS;
  m_FA = FA;
}

void BSplinesFoot::SetParametersWithInitFinalPose(double FT, double IP,
                                                  double FP,
                                                  std::vector<double> &ToMP,
                                                  std::vector<double> &MP) {
  // verify that each middle point has a reaching time parameter
  assert(ToMP.size() == MP.size());

  // save the parameters
  m_FT = FT;
  m_IP = IP;
  m_FP = FP;
  m_ToMP = ToMP;
  m_MP = MP;

  // initialize some variables
  std::deque<double> knot;
  std::vector<double> control_points;
  knot.clear();
  control_points.clear();

  // generation of the knot vector
  switch (ToMP.size()) {
  case 0:
    // set the first three knots to 0.0
    // the next one to 50% of the final time
    // and the last three to the final time
    for (unsigned int i = 0; i <= m_degree; i++) {
      knot.push_back(0.0);
    }

    for (unsigned int i = 0; i < (m_degree - 1); ++i) {
      knot.push_back((double)(i + 1) / ((double)m_degree));
    }

    for (unsigned int i = 0; i <= m_degree; i++) {
      knot.push_back(1);
    }

    // Set the first three control point
    // to the initial pos and the last three
    // to the final pos
    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_IP);

    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_FP);
    break;

  case 1:
    for (unsigned int i = 0; i <= m_degree; ++i) {
      knot.push_back(0.0);
    }

    for (unsigned int i = 1; i <= 3; ++i) {
      knot.push_back((double)i / 3.0 * m_ToMP[0] / m_FT);
    }

    for (unsigned int i = 0; i < 3; ++i) {
      knot.push_back((m_ToMP[0] + (double)i / 3 * (m_FT - m_ToMP[0])) / m_FT);
    }

    for (unsigned int i = 0; i <= m_degree; ++i) {
      knot.push_back(1);
    }

    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_IP);

    control_points.push_back(m_MP[0]);
    control_points.push_back(m_MP[0]);

    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_FP);

    break;

  case 2:
    for (unsigned int i = 0; i <= m_degree; ++i) {
      knot.push_back(0.0);
    }

    for (unsigned int i = 1; i <= 2; ++i) {
      knot.push_back((double)i / 2.0 * m_ToMP[0] / m_FT);
    }

    for (unsigned int i = 0; i < 2; ++i) {
      knot.push_back((m_ToMP[0] + m_ToMP[1]) * 0.5 / m_FT);
    }

    for (unsigned int i = 0; i < 2; ++i) {
      knot.push_back((m_ToMP[1] + (double)i / 2 * (m_FT - m_ToMP[1])) / m_FT);
    }

    for (unsigned int i = 0; i <= m_degree; ++i) {
      knot.push_back(1);
    }

    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_IP);

    control_points.push_back(m_MP[0]);
    control_points.push_back(m_MP[1]);

    for (unsigned int i = 0; i < m_degree; ++i)
      control_points.push_back(m_FP);

    break;
  } // end switch case

  SetKnotVector(knot);
  SetControlPoints(control_points);

  return;
}
