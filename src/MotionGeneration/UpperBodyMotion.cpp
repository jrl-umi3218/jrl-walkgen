/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Francois Keith
 * Olivier Stasse
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

/*! This object generate all the values for the upperbody motion */
#include <iostream>
#include <fstream>
#include <MotionGeneration/UpperBodyMotion.hh>

using namespace::PatternGeneratorJRL;

UpperBodyMotion::UpperBodyMotion()
{

}

UpperBodyMotion::~UpperBodyMotion()
{

}



void UpperBodyMotion::GenerateDataFile(string aFileName, int LenghtDataArray)
{
  ofstream aof;



  aof.open(aFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {

      for(int i=0; i<LenghtDataArray; i++)
        {

#if 0
          aof         << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " //chest
                      << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " "  // head
                      << 14.813*M_PI/180.0  << " "
                      << -10.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " << -30.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " "  <<  0.0*M_PI/180.0 << " "
                      << 10.0*M_PI/180.0 << " "   //rarm
                      << 14.813*M_PI/180.0 << " " << 10.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " << -30.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0<< " "  <<  0.0*M_PI/180.0 << " "
                      << 10.0*M_PI/180.0<< " "  //larm
                      << -10.0*M_PI/180.0 << " " << 10.0*M_PI/180.0 << " "
                      << -10.0*M_PI/180.0 << " " << 10.0*M_PI/180.0 << " "
                      << -10.0*M_PI/180.0 << " " //rhand
                      << -10.0*M_PI/180.0  << " " << 10.0*M_PI/180.0 << " "
                      << -10.0*M_PI/180.0<< " " << 10.0*M_PI/180.0 << " "
                      << -10.0*M_PI/180.0<< " " //lhand
                      << endl;
#else
          aof         << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " //chest
                      << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " //head
                      << 0.0*M_PI/180.0  << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " "  <<  0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " "    //rarm
                      << 0.0*M_PI/180.0 << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0<< " "  <<  0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0<< " "     //larm
                      << 0.0*M_PI/180.0 << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0 << " " //rhand
                      << 0.0*M_PI/180.0  << " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0<< " " << 0.0*M_PI/180.0 << " "
                      << 0.0*M_PI/180.0<< " " //lhand
                      << endl;
#endif
        }

      aof.close();
    }

}


void UpperBodyMotion::ReadDataFile(string aFileName,
                                   Eigen::MatrixXd &UpperBodyAngles)
{

  std::ifstream aif;

  unsigned int NumberRows, NumberColumns;

  NumberRows = UpperBodyAngles.rows();
  NumberColumns = UpperBodyAngles.cols();

  double r;

  aif.open(aFileName.c_str(),std::ifstream::in);
  if (aif.is_open())
    {
      for (unsigned int i=0; i<NumberRows; i++)
        {
          for (unsigned int j=0; j<NumberColumns; j++)
            {
              aif >> r;
              UpperBodyAngles(i,j) = r;
            }
        }
      aif.close();
    }
  else
    std::cerr << "UpperBodyMotion - Unable to open " << aFileName << endl;

}

void UpperBodyMotion::WriteDataFile(string aFileName,
                                    Eigen::MatrixXd &UpperBodyAngles )
{
  ofstream aof;
  unsigned int NumberRows, NumberColumns;
  NumberRows = UpperBodyAngles.rows();
  NumberColumns = UpperBodyAngles.cols();

  aof.open(aFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {

      for (unsigned int i=0; i<NumberRows; i++)
        {
          for (unsigned int j=0; j<NumberColumns; j++)
            {
              aof << UpperBodyAngles(i,j)  << "\t";
            }
          aof << endl;
        }
      aof.close();
    }

}

