# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of jrl-walkgen.
# jrl-walkgen is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# jrl-walkgen is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# jrl-walkgen. If not, see <http://www.gnu.org/licenses/>.
INCLUDE(CTest)

# Import jrl-mal flags.
ADD_DEFINITIONS(${JRL_MAL_CFLAGS})

# Make sure private headers can be used.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

# Make the config_private.hh file available
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/private_cmake/config_private.hh.cmake
               ${CMAKE_BINARY_DIR}/include/${HEADER_DIR}/config_private.hh)

ADD_REQUIRED_DEPENDENCY("simple_humanoid_description >= 1.0.1")
ADD_OPTIONAL_DEPENDENCY("hrp2_14_description >= 1.0.2")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")
ADD_REQUIRED_DEPENDENCY("pinocchio >=  1.1.0")

#FIND_PACKAGE(YARP REQUIRED)
#INCLUDE_DIRECTORIES(SYSTEM ${YARP_INCLUDE_DIRS})



#################
## Generic test #
#################
#SET(GENERIC_TEST)
IF(GENERIC_TEST)
  ADD_EXECUTABLE(TestFootPrintPGInterface
    ../src/portability/gettimeofday.cc
    TestFootPrintPGInterfaceData.cpp
    CommonTools.cpp
    TestFootPrintPGInterface.cpp
    )

  TARGET_LINK_LIBRARIES(TestFootPrintPGInterface ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(TestFootPrintPGInterface jrl-mal)
  ADD_DEPENDENCIES(TestFootPrintPGInterface ${PROJECT_NAME})
ENDIF(GENERIC_TEST)

##########################
## Test Optimal Cholesky #
##########################
ADD_EXECUTABLE(TestOptCholesky
  TestOptCholesky.cpp
  ../src/Mathematics/OptCholesky.cpp
)
# Add test on the ricatti equation
ADD_TEST(TestOptCholesky TestOptCholesky)

##########################
## Test Bspline #
##########################
#TODO create a proper test for polynomes and bsplines
#ADD_EXECUTABLE(TestBsplines
#  TestBsplines.cpp
#  ../src/Mathematics/Bsplines.cpp
#  )

##########################
## Test Ricatti Equation #
##########################
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithoutInitialPose.datref)
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithInitialPose.datref)
ADD_EXECUTABLE(TestRiccatiEquation
  TestRiccatiEquation.cpp
  ../src/PreviewControl/OptimalControllerSolver.cpp
  )
# Need this dependency for the use of the fortran functions (it seems)
PKG_CONFIG_USE_DEPENDENCY(TestRiccatiEquation jrl-mal)
# Add test on the ricatti equation
ADD_TEST(TestRiccatiEquation TestRiccatiEquation)

################################################
## Generic Macro That Create a Boost Test Case #
################################################
MATH(EXPR BITS "8*${CMAKE_SIZEOF_VOID_P}")
MACRO(ADD_JRL_WALKGEN_TEST test_arg test_file_name)

  SET(test_name "${test_arg}${BITS}")

  CONFIG_FILES_CMAKE(${test_name}TestFGPI.datref)

  ADD_EXECUTABLE(${test_name}
    ${test_file_name}
    TestObject.cpp
    CommonTools.cpp
    ClockCPUTime.cpp
    ../src/portability/gettimeofday.cc
  )

  MESSAGE(STATUS "Prepare test : " ${test_name})
  SET(urdfpath
    ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/urdf/simple_humanoid.urdf
  )
  SET(srdfpath
    ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/srdf/simple_humanoid.srdf
  )

  LIST(APPEND LOGGING_WATCHED_VARIABLES urdfpath srdfpath)

  TARGET_LINK_LIBRARIES(${test_name} ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(${test_name} jrl-mal)

  ADD_TEST(${test_name} ${test_name} ${urdfpath} ${srdfpath})

ENDMACRO(ADD_JRL_WALKGEN_TEST)


#######################
## Test Morisawa 2007 #
#######################

#disabled as it fail : fix the code
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007OnLine TestMorisawa2007.cpp)

# the following disabled test case are just useful code for offline
# motion generation using the specialized inverse kinmatics of a robot
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007ShortWalk TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007Climbing TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007GoingDown TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007SteppingStones TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007WalkingOnBeam TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007GoThroughWall TestMorisawa2007.cpp)

####################
## Test Herdt 2010 #
####################

#ADD_JRL_WALKGEN_TEST(TestHerdt2010OnLine TestHerdt2010.cpp)
#ADD_JRL_WALKGEN_TEST(TestHerdt2010EmergencyStop TestHerdt2010.cpp)
#  CONFIG_FILES_CMAKE(${test_name}TestFGPI.datref)

ADD_EXECUTABLE(walkgentest
    walkgentest.cpp
#    TestObject.cpp
    CommonTools.cpp
    ClockCPUTime.cpp
    ../src/portability/gettimeofday.cc
  )

MESSAGE(STATUS "Prepare test : " "walgentest")
SET(urdfpath
  ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/urdf/simple_humanoid.urdf
)
SET(srdfpath
  ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/srdf/simple_humanoid.srdf
)

LIST(APPEND LOGGING_WATCHED_VARIABLES urdfpath srdfpath)

TARGET_LINK_LIBRARIES(walkgentest ${PROJECT_NAME}
                                 # ${YARP_LIBRARIES}
)
PKG_CONFIG_USE_DEPENDENCY(walkgentest jrl-mal)


############################
## Test Inverse Kinematics #
############################

#Disabled : fix the code to handl multiple inverse kinematics
# Todo compare halfsiting pose after forward and inverse kinematics.
#ADD_JRL_WALKGEN_TEST(TestInverseKinematics TestInverseKinematics.cpp)

###############################
## Test Kajita Dynamic Filter #
###############################

#TO BE FIXED with a proper inverse kinematics computed for the simple humanoid
#ADD_JRL_WALKGEN_TEST(TestKajitaDynamicFilter TestKajitaDynamicFilter.cpp)
#SET(DYNAMIC_FILTER) TO BE FIXED

#####################
## Test Kajita 2003 #
#####################

## These test is disabled for now as it fails.
## FIXME: fix the test and/or the implementation
#CONFIG_FILES_CMAKE(TestKajita2003StraightWalking${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003Circle${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq1${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq2${BITS}TestFGPI.datref)
#ADD_JRL_WALKGEN_TEST(TestKajita2003StraightWalking TestKajita2003.cpp)
#ADD_JRL_WALKGEN_TEST(TestKajita2003Circle          TestKajita2003.cpp)
#ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq1   TestKajita2003.cpp)
#ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq2   TestKajita2003.cpp)

##########################
## Read Novela Data 2011 #
##########################

#ADD_JRL_WALKGEN_TEST(ReadDataForNovela ReadDataForNovela.cpp)

################
# NMPC test #
################

# Disabled as the test fail : random results oscillating around mean behaviour
#ADD_JRL_WALKGEN_TEST(TestNaveau2015Online TestNaveau2015.cpp)

#####################
# Add user examples #
#####################
SET(EXAMPLES_SOURCES
  TestFootPrintPGInterfaceData.cpp
  TestFootPrintPGInterfaceData.h
  TestFootPrintPGInterface.cpp
  TestHerdt2010.cpp
  CommonTools.cpp
  CommonTools.hh
  examples.CMakeList
  )

INSTALL(FILES ${EXAMPLES_SOURCES}
  DESTINATION share/jrl/dynamics/examples/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )
