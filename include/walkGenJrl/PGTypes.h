/*! \file PGTypes.h
    \brief Defines basic types for the Humanoid Walking Pattern Generator.

    SVN Information:
   $Id$
   $Author$
   $Date$
   $Revision $
   $Source $
   $Log $


   Copyright (c) 2005-2008, 
   @author Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see license.txt for more information on license.

*/
#ifndef _PATTERN_GENERATOR_TYPES_H_
#define  _PATTERN_GENERATOR_TYPES_H_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

namespace PatternGeneratorJRL
{
  /// Structure to store the COM position computed by the preview control. 
  struct COMPosition_s
  {
    double x[3],y[3]; 
    double z[3];    	
    double yaw; // aka theta
    double pitch; // aka omega
    double roll; // aka hip 
     	
  };
  typedef struct COMPosition_s COMPosition;
  typedef struct COMPosition_s WaistState;

  //TODO 0: Maybe COMPosition_s should be replaced by COMState_s
  /// Structure to store the COM state computed by the preview control.
    struct COMState_s
    {
      double x[3],y[3],z[3];
      double yaw[3]; // aka theta
      double pitch[3]; // aka omega
      double roll[3]; // aka hip
    };
    typedef struct COMState_s COMState;

  /** Structure to store each foot position when the user is specifying 
      a sequence of relative positions. */
  struct RelativeFootPosition_s
  { 
    double sx,sy,theta;
    float SStime;
    float DStime;
    int stepType;     //1:normal walking 2:one step before opbstacle
                      //3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
    double DeviationHipHeight;      
   };
  typedef struct RelativeFootPosition_s RelativeFootPosition;

  /** Structure to store each of the ZMP value, with a given
      direction at a certain time. */
  struct ZMPPosition_s
  { 
    double px,py,pz;
    double theta;//For COM
    double time;
    int stepType;     //1:normal walking 2:one step before opbstacle
                      //3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
			//+10 if duoble support phase
			//*(-1) if right foot stance else left foot stance 
  };
  typedef struct ZMPPosition_s ZMPPosition;

  /// Structure to store the absolute foot position.
  struct FootAbsolutePosition_t
  { 
    /*! px, py in meters, theta in DEGREES. */
    double x,y,z, theta, omega, omega2; 
    /*! Speed of the foot. */
    double dx,dy,dz, dtheta, domega, domega2; 
    /*! Time at which this position should be reached. */
    double time;
    /*! 1:normal walking 2:one step before opbstacle
      3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
      +10 if double support phase
      (-1) if support foot  */
    int stepType;     
  };
  typedef struct FootAbsolutePosition_t FootAbsolutePosition;

  // Linear constraint.
  struct LinearConstraintInequality_s
  {
    MAL_MATRIX(A,double);
    MAL_MATRIX(B,double);
    MAL_VECTOR(Center,double);
    std::vector<int> SimilarConstraints;
    double StartingTime, EndingTime;
  };
  typedef struct LinearConstraintInequality_s 
    LinearConstraintInequality_t;
  
  /// Linear constraints with variable feet placement.
  struct LinearConstraintInequalityFreeFeet_s
  {
    MAL_MATRIX(D,double);
    MAL_MATRIX(Dc,double);
    unsigned int StepNumber;
  };
  typedef struct LinearConstraintInequalityFreeFeet_s 
    LinearConstraintInequalityFreeFeet_t;

  //State of the feet on the ground
  struct SupportFeet_s
  {
    double x,y,theta,StartTime;
    int SupportFoot;
  };
  typedef struct SupportFeet_s 
    SupportFeet_t;

  /// Structure to store the absolute reference.
  struct ReferenceAbsoluteVelocity_t
  { 
    /*! m/sec or degrees/sec */
    double x,y,z, theta; 

    /*! reference values for the whole preview window */
    MAL_VECTOR(RefVectorX,double);	
    MAL_VECTOR(RefVectorY,double);
    MAL_VECTOR(RefVectorTheta,double);
  };
  typedef struct ReferenceAbsoluteVelocity_t ReferenceAbsoluteVelocity;
  
};
#endif
