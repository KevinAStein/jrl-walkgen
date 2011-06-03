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

#include <Mathematics/PolynomeFoot.h>



using namespace::std;
using namespace::PatternGeneratorJRL;

Polynome3::Polynome3(double FT, double FP) :Polynome(3)
{
  SetParameters(FT,FP);
}

void Polynome3::SetParameters(double FT, double FP)
{
  m_FT = FT;
  m_FP = FP;
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  m_Coefficients[2] = 3.0*FP/tmp;
  m_Coefficients[3] = -2.0*FP/(tmp*FT);
}

void Polynome3::SetParametersWithInitPosInitSpeed(double FT,
						  double FP,
						  double InitPos,
						  double InitSpeed)
{
  m_FT = FT;
  m_FP = FP;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  m_Coefficients[2] = (3*FP - 3*InitPos - 2*InitSpeed*FT)/tmp;
  m_Coefficients[3] = (InitSpeed*FT+ 2*InitPos - 2*FP)/(tmp*FT);
}

void Polynome3::GetParametersWithInitPosInitSpeed(double &FT,
						  double &FP,
						  double &InitPos,
						  double &InitSpeed)
{
  InitPos= m_Coefficients[0];
  InitSpeed= m_Coefficients[1];
  FT = m_FT;
  FP = m_FP;
}

Polynome3::~Polynome3()
{}

Polynome4::Polynome4(double FT, double FP) :Polynome(4)
{
  SetParameters(FT,FP);
}

void Polynome4::SetParameters(double FT, double MP)
{
  m_FT = FT;
  m_MP = MP;
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  m_Coefficients[2] = 16.0*MP/tmp;
  tmp=tmp*FT;
  m_Coefficients[3] = -32.0*MP/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = 16.0*MP/tmp;
}

void Polynome4::SetParametersWithInitPosInitSpeed(double FT,
						  double MP,
						  double InitPos,
						  double InitSpeed)
{
  m_FT = FT;
  m_MP = MP;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  m_Coefficients[2] = (-4.0*InitSpeed*FT - 11.0*InitPos + 16.0*MP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[3] = ( 5.0*InitSpeed*FT + 18.0*InitPos - 32.0*MP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = (-2.0*InitSpeed*FT - 8.0 *InitPos + 16.0*MP)/tmp;
}
void Polynome4::GetParametersWithInitPosInitSpeed(double &FT,
						  double &MP,
						  double &InitPos,
						  double &InitSpeed)
{
  FT = m_FT;
  MP = m_MP;
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
}
Polynome4::~Polynome4()
{}
Polynome5::Polynome5(double FT, double FP) :Polynome(5)
{
  SetParameters(FT,FP);
}

void Polynome5::SetParameters(double FT, double FP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  m_Coefficients[3] = 10*FP/tmp;
  tmp *=FT;
  m_Coefficients[4] = -15*FP/tmp;
  tmp*=FT;
  m_Coefficients[5] = 6*FP/tmp;
}

Polynome5::~Polynome5()
{}

Polynome6::Polynome6(double FT, double MP) :Polynome(6)
{
  SetParameters(FT,MP);
}

void Polynome6::SetParameters(double FT, double MP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  m_Coefficients[3] = 64*MP/tmp;
  tmp *=FT;
  m_Coefficients[4] = -192*MP/tmp;
  tmp *=FT;
  m_Coefficients[5] = 192*MP/tmp;
  tmp *=FT;
  m_Coefficients[6] = -64*MP/tmp;
}

Polynome6::~Polynome6()
{  
}

