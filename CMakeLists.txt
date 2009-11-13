# Olivier Stasse, JRL, CNRS/AIST
# Creation: 05/11/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)
enable_language(CXX)

# Name of the project
#--------------------
SET(PROJECT_NAME walkGenJrl)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Humanoid Walking Pattern Generator (JRL)")

# Counter CMake mesures
#----------------------
IF(WIN32)
SET(CMAKE_STATIC_LIBRARY_PREFIX "")
SET(CMAKE_SHARED_LIBRARY_PREFIX "")
SET(CMAKE_SHARED_MODULE_PREFIX "")
ENDIF(WIN32)


# Debug mode for cmake.
#----------------------
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
#------------
SET(PROJECT_VERSION 3.0.0)

# Deal with OS specificities
#---------------------------
IF(WIN32)
  SET(LIBDIR_KW "/LIBPATH:")
  SET(LIBINCLUSION_KW "")
  SET(LIBINCLUSION_EXT ".lib")
ENDIF(WIN32)

IF(UNIX)
  SET(LIBDIR_KW "-L")
  SET(LIBDIR_RP "-Wl,-R")
  SET(LIBINCLUSION_KW "-l")
  SET(LIBINCLUSION_EXT "")
  SET(${PROJECT_NAME}_CXXFLAGS "-DUNIX ")
ENDIF(UNIX)

project(${PROJECT_NAME})

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build walkGenJrl with shared libraries." ON)

# where are located specific cmake modules
#-----------------------------------------
SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/CMakeModules)

# Check for required packages
# -------------------------
INCLUDE(FindPkgConfig)

SET(DYNJRLJAPAN_REQUIRED_VERSION 1.15)
pkg_check_modules(DYNJRLJAPAN REQUIRED dynamicsJRLJapan>=${DYNJRLJAPAN_REQUIRED_VERSION})
SET(PROJECT_REQUIREMENTS "dynamicsJRLJapan >= ${DYNJRLJAPAN_REQUIRED_VERSION}")

EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS  --variable=docdir dynamicsJRLJapan
  OUTPUT_VARIABLE _dynamicsJRLJapan_DOCDIR)

SET(${PROJECT_NAME}_src_CXXFLAGS "${${PROJECT_NAME}_src_CXXFLAGS} -I${${PROJECT_NAME}_SOURCE_DIR}")

IF(UNIX)
	SET(${PROJECT_NAME}_src_CXXFLAGS "-Wall ${${PROJECT_NAME}_src_CXXFLAGS}")
ENDIF(UNIX)

IF(WIN32)
	SET(${PROJECT_NAME}_src_CXXFLAGS "/D__STDC__ ${${PROJECT_NAME}_src_CXXFLAGS}")
ENDIF(WIN32)

FOREACH(option_cxxflags ${DYNJRLJAPAN_CFLAGS})
  SET(${PROJECT_NAME}_src_CXXFLAGS "${${PROJECT_NAME}_src_CXXFLAGS} ${option_cxxflags} ")
ENDFOREACH(option_cxxflags)

FOREACH(option_ldflags ${DYNJRLJAPAN_LDFLAGS})
  SET(${PROJECT_NAME}_src_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_src_LDFLAGS}")
ENDFOREACH(option_ldflags)

IF(UNIX)
  SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib/ ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_LDFLAGS}")
ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib/ ${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIBINCLUSION_EXT} ${${PROJECT_NAME}_LDFLAGS}")
ENDIF(WIN32)



# Optimisation flags when building with RELEASE
#----------------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "RELEASE")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET (${PROJECT_NAME}_CXXFLAGS "-O3 -funroll-loops -frerun-loop-opt -fschedule-insns2 -frerun-cse-after-loop -falign-functions -falign-labels -falign-loops -falign-jumps -fexpensive-optimizations ${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "RELEASE")

# To deal with the headers.
#--------------------------
add_subdirectory(src)
add_subdirectory(unitTesting)

# Prepare the documentation
# -------------------------
OPTION(GENERATE_DOC "Generate html documentation" ON)
IF (GENERATE_DOC)
  INCLUDE(FindDoxygen)
  IF(DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen found")
	ADD_SUBDIRECTORY(doc)
  ENDIF(DOXYGEN_FOUND)
ENDIF (GENERATE_DOC)

# Prepare description file for pkg-config.
#-----------------------------------------
SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")
SET(install_pkg_datarootdir "\${datarootdir}")

# Install the file in the appropriate place.
#-------------------------------------------
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

# Include CPack to build a beautiful package...
#----------------------------------------------
INCLUDE(CPack)






