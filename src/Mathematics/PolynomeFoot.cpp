/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Olivier    Stasse
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
/* Polynomes object for generating foot trajectories. */
#include <iostream>
#include <vector>

#include <Debug.hh>
#include <Mathematics/PolynomeFoot.hh>

using namespace ::std;
using namespace ::PatternGeneratorJRL;

double PolynomeFoot::Compute(double t) {
  if (t >= FT_)
    return Polynome::Compute(FT_);
  else if (t <= 0.0)
    return Polynome::Compute(0.0);
  else
    return Polynome::Compute(t);
}

double PolynomeFoot::ComputeDerivative(double t) {
  if (t >= FT_)
    return Polynome::ComputeDerivative(FT_);
  else if (t <= 0.0)
    return Polynome::ComputeDerivative(0.0);
  else
    return Polynome::ComputeDerivative(t);
}

double PolynomeFoot::ComputeSecDerivative(double t) {
  if (t >= FT_)
    return Polynome::ComputeSecDerivative(FT_);
  else if (t <= 0.0)
    return Polynome::ComputeSecDerivative(0.0);
  else
    return Polynome::ComputeSecDerivative(t);
}

double PolynomeFoot::ComputeJerk(double t) {
  if (t >= FT_)
    return Polynome::ComputeJerk(FT_);
  else if (t <= 0.0)
    return Polynome::ComputeJerk(0.0);
  else
    return Polynome::ComputeJerk(t);
}

Polynome3::Polynome3(double FT, double FP) : PolynomeFoot(3, FT) {
  SetParameters(FT, FP);
}

Polynome3::Polynome3(double FT, double IP, double IS, double FP, double FS)
    : PolynomeFoot(3, FT) {
  SetParameters(FT, IP, IS, FP, FS);
}

void Polynome3::SetParameters(double FT, double FP) {
  Polynome3::SetParametersWithInitPosInitSpeed(FT, FP,
                                               /*InitPos*/ 0.0,
                                               /*InitSpeed*/ 0.0);
}

void Polynome3::SetParameters(double FT, double IP, double IS, double FP,
                              double FS) {
  FT_ = FT;
  FP_ = FP;

  double tmp;
  m_Coefficients[0] = IP;
  m_Coefficients[1] = IS;
  tmp = FT * FT;
  if (FT == 0.0) {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
  } else {
    m_Coefficients[2] = (3 * FP - 3 * IP - (2 * IS + FS) * FT) / tmp;
    m_Coefficients[3] = ((FS + IS) * FT + 2 * IP - 2 * FP) / (tmp * FT);
  }
}

void Polynome3::SetParametersWithInitPosInitSpeed(double FT, double FP,
                                                  double InitPos,
                                                  double InitSpeed) {
  FT_ = FT;
  FP_ = FP;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT * FT;
  if (FT == 0.0) {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
  } else {
    m_Coefficients[2] = (3 * FP - 3 * InitPos - 2 * InitSpeed * FT) / tmp;
    m_Coefficients[3] = (InitSpeed * FT + 2 * InitPos - 2 * FP) / (tmp * FT);
  }
}

void Polynome3::GetParametersWithInitPosInitSpeed(double &FT, double &FP,
                                                  double &InitPos,
                                                  double &InitSpeed) {
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
  FT = FT_;
  FP = FP_;
}

Polynome3::~Polynome3() {}

Polynome4::Polynome4(double FT, double MP, double FP) : PolynomeFoot(4, FT) {
  SetParameters(FT, MP, FP);
}

void Polynome4::SetParameters(double FT, double MP, double FP) {
  SetParametersWithInitPosInitSpeed(FT, MP,
                                    /*InitPos*/ 0.0,
                                    /*InitSpeed*/ 0.0,
                                    /*Final Pos*/ FP);
}

void Polynome4::SetParameters(double FT, double InitPos, double InitSpeed,
                              double InitAcc, double FinalSpeed,
                              double FinalAcc) {
  MP_ = 0.0; // default value
  FP_ = 0.0;

  FT_ = FT;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  m_Coefficients[2] = InitAcc / 2;
  tmp = FT * FT;
  if (tmp == 0.0) {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
  } else {
    m_Coefficients[3] = (-5 * InitAcc * FT - 2 * FinalAcc * FT -
                         6.0 * InitSpeed + 6.0 * FinalSpeed) /
                        (6 * tmp);
    tmp = tmp * FT;
    m_Coefficients[4] = (3 * InitAcc * FT + 2 * FinalAcc * FT +
                         4.0 * InitSpeed - 4.0 * FinalSpeed) /
                        (8 * tmp);
  }
}

void Polynome4::SetParametersWithInitPosInitSpeed(double FT, double MP,
                                                  double InitPos,
                                                  double InitSpeed, double FP) {
  FT_ = FT;
  MP_ = MP;
  FP_ = FP;
  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT * FT;
  if (tmp == 0.0) {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
  } else {
    m_Coefficients[2] =
        (-4.0 * InitSpeed * FT - 11.0 * InitPos + 16.0 * MP - 5 * FP) / tmp;
    tmp = tmp * FT;
    m_Coefficients[3] =
        (5.0 * InitSpeed * FT + 18.0 * InitPos - 32.0 * MP + 14 * FP) / tmp;
    tmp = tmp * FT;
    m_Coefficients[4] =
        (-2.0 * InitSpeed * FT - 8.0 * InitPos + 16.0 * MP - 8 * FP) / tmp;
  }
}
void Polynome4::GetParametersWithInitPosInitSpeed(double &FT, double &MP,
                                                  double &FP, double &InitPos,
                                                  double &InitSpeed) {
  FP = FP_;
  FT = FT_;
  MP = MP_;
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
}
Polynome4::~Polynome4() {}

Polynome5::Polynome5(double FT, double FP)
    : PolynomeFoot(5, FT), InitPos_(0.0), InitSpeed_(0.0), InitAcc_(0.0),
      FinalPos_(0.0), FinalSpeed_(0.0), FinalAcc_(0.0)

{
  SetParameters(FT, FP);
}

Polynome5::~Polynome5() {}

void Polynome5::SetParameters(double FT, double FP) {
  SetParameters(FT, FP, 0.0, 0.0, 0.0, 0.0);
}

void Polynome5::SetParametersWithInitPosInitSpeed(double FT, double FP,
                                                  double InitPos,
                                                  double InitSpeed) {
  SetParameters(FT, FP, InitPos, InitSpeed, 0.0, 0.0);
}

void Polynome5::GetParametersWithInitPosInitSpeed(double &FT, double &FP,
                                                  double &InitPos,
                                                  double &InitSpeed) {
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
  FT = FT_;
  FP = FinalPos_;
}

void Polynome5::SetParameters(double FT, double FP, double InitPos,
                              double InitSpeed, double InitAcc, double) {
  double tmp;
  m_Coefficients[0] = InitPos_ = InitPos;
  m_Coefficients[1] = InitSpeed_ = InitSpeed;
  m_Coefficients[2] = InitAcc / 2.0;
  InitAcc_ = InitAcc;
  FT_ = FT;
  FinalPos_ = FP;
  tmp = FT * FT * FT;
  if (tmp == 0.0) {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
  } else {
    m_Coefficients[3] = (-3.0 / 2.0 * InitAcc * FT * FT - 6.0 * InitSpeed * FT -
                         10.0 * InitPos + 10.0 * FP) /
                        tmp;
    tmp = tmp * FT;
    m_Coefficients[4] = (3.0 / 2.0 * InitAcc * FT * FT + 8.0 * InitSpeed * FT +
                         15.0 * InitPos - 15.0 * FP) /
                        tmp;
    tmp = tmp * FT;
    m_Coefficients[5] = (-1.0 / 2.0 * InitAcc * FT * FT - 3.0 * InitSpeed * FT -
                         6.0 * InitPos + 6.0 * FP) /
                        tmp;
  }
}

void Polynome5::SetParameters(double FT, double InitPos, double InitSpeed,
                              double InitAcc, double FinalPos,
                              double FinalSpeed, double FinalAcc) {
  double tmp;
  m_Coefficients[0] = InitPos_ = InitPos;
  m_Coefficients[1] = InitSpeed_ = InitSpeed;
  m_Coefficients[2] = InitAcc / 2.0;
  InitAcc_ = InitAcc;
  FT_ = FT;
  FinalPos_ = FinalPos;
  tmp = FT * FT * FT;
  if (tmp == 0.0) {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
  } else {
    m_Coefficients[3] = -(1.5 * InitAcc * FT * FT - 0.5 * FinalAcc * FT * FT +
                          6.0 * InitSpeed * FT + 4.0 * FinalSpeed * FT +
                          10.0 * InitPos - 10.0 * FinalPos) /
                        tmp;
    tmp = tmp * FT;
    m_Coefficients[4] =
        (1.5 * InitAcc * FT * FT - FinalAcc * FT * FT + 8.0 * InitSpeed * FT +
         7.0 * FinalSpeed * FT + 15.0 * InitPos - 15.0 * FinalPos) /
        tmp;
    tmp = tmp * FT;
    m_Coefficients[5] = -(0.5 * InitAcc * FT * FT - 0.5 * FinalAcc * FT * FT +
                          3.0 * InitSpeed * FT + 3.0 * FinalSpeed * FT +
                          6.0 * InitPos - 6.0 * FinalPos) /
                        tmp;
  }
}

Polynome6::Polynome6(double FT, double MP, double FP) : PolynomeFoot(6, FT) {
  SetParameters(FT, MP, FP);
}

void Polynome6::SetParameters(double FT, double MP, double FP) {
  SetParametersWithMiddlePos(FT, MP,
                             /*InitPos=*/0.0,
                             /*InitSpeed=*/0.0,
                             /*InitAcc=*/0.0, FP);
}

void Polynome6::SetParametersWithMiddlePos(double FT, double MP, double InitPos,
                                           double InitSpeed, double InitAcc,
                                           double FP) {
  FT_ = FT;
  MP_ = MP;
  FP_ = FP;
  InitPos_ = InitPos;
  InitSpeed_ = InitSpeed;
  InitAcc_ = InitAcc;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  m_Coefficients[2] = 0.5 * InitAcc;
  if (MP == 0.0 || FT == 0.0) {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
    m_Coefficients[6] = 0.0;
  } else {
    m_Coefficients[3] = -0.5 *
                        (5 * FT * FT * InitAcc + 32 * FT * InitSpeed -
                         128 * MP + 84 * InitPos + 44 * FP) /
                        (FT * FT * FT);
    m_Coefficients[4] = 0.5 *
                        (9 * FT * FT * InitAcc + 76 * FT * InitSpeed -
                         384 * MP + 222 * InitPos + 162 * FP) /
                        (FT * FT * FT * FT);
    m_Coefficients[5] = -0.5 *
                        (7 * FT * FT * InitAcc + 66 * FT * InitSpeed -
                         384 * MP + 204 * InitPos + 180 * FP) /
                        (FT * FT * FT * FT * FT);
    m_Coefficients[6] = (FT * FT * InitAcc + 10 * FT * InitSpeed - 64 * MP +
                         32 * InitPos + 32 * FP) /
                        (FT * FT * FT * FT * FT * FT);
  }
}

void Polynome6::GetParametersWithInitPosInitSpeed(double &TimeInterval,
                                                  double &MiddlePosition,
                                                  double &FinalPosition,
                                                  double &InitPosition,
                                                  double &InitSpeed) {
  TimeInterval = FT_;
  MiddlePosition = MP_;
  FinalPosition = FP_;
  InitPosition = InitPos_;
  InitSpeed = InitSpeed_;
}

Polynome6::~Polynome6() {}

Polynome7::Polynome7(double FT, double FP)
    : PolynomeFoot(7, FT), FP_(FP), InitPos_(0.0), InitSpeed_(0.0),
      InitAcc_(0.0)

{
  SetParameters(FT, FP);
}

void Polynome7::SetParameters(double FT, double FP) {
  SetParameters(FT, FP,
                /*InitPos*/ 0.0,
                /*InitSpeed*/ 0.0,
                /*InitAcc*/ 0.0,
                /*InitJerk*/ 0.0);
}

void Polynome7::SetParametersWithInitPosInitSpeed(double FT, double FP,
                                                  double InitPos,
                                                  double InitSpeed) {
  SetParameters(FT, FP, InitPos, InitSpeed, 0.0, 0.0);
}

void Polynome7::GetParametersWithInitPosInitSpeed(double &FT, double &FP,
                                                  double &InitPos,
                                                  double &InitSpeed) {
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
  FT = FT_;
  FP = FP_;
}

void Polynome7::SetParameters(double FT, double FP, double InitPos,
                              double InitSpeed, double InitAcc,
                              double InitJerk) {
  double tmp;
  FT_ = FT;
  FP_ = FP;
  m_Coefficients[0] = InitPos_ = InitPos;
  m_Coefficients[1] = InitSpeed_ = InitSpeed;
  m_Coefficients[2] = InitAcc / 2.0;
  InitAcc_ = InitAcc;
  m_Coefficients[3] = InitJerk / 6.0;
  InitJerk_ = InitJerk;

  tmp = FT * FT * FT * FT;
  if (tmp == 0.0) {
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
    m_Coefficients[6] = 0.0;
    m_Coefficients[7] = 0.0;
  } else {
    m_Coefficients[4] = -(2 * InitJerk * FT * FT * FT + 15 * InitAcc * FT * FT +
                          60 * InitSpeed_ * FT + 105 * InitPos_ - 105 * FP) /
                        (3 * tmp);
    tmp *= FT;
    m_Coefficients[5] = (InitJerk * FT * FT * FT + 10 * InitAcc * FT * FT +
                         45 * InitSpeed_ * FT + 84 * InitPos_ - 84 * FP) /
                        tmp;
    tmp *= FT;
    m_Coefficients[6] = -(4 * InitJerk * FT * FT * FT + 45 * InitAcc * FT * FT +
                          216 * InitSpeed_ * FT + 420 * InitPos_ - 420 * FP) /
                        (6 * tmp);
    tmp *= FT;
    m_Coefficients[7] = (InitJerk * FT * FT * FT + 12 * InitAcc * FT * FT +
                         60 * InitSpeed_ * FT + 120 * InitPos_ - 120 * FP) /
                        (6 * tmp);
  }
}

Polynome7::~Polynome7() {}
