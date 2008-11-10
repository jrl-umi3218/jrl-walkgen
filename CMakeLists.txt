# Olivier Stasse, JRL, CNRS/AIST
# Creation: 05/11/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)
enable_language(Fortran)

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
SET(PROJECT_VERSION 2.5.0)

# name of the project
SET(PROJECT_NAME walkGenJrl)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Humanoid Walking Pattern Generator (JRL)")
project(${PROJECT_NAME})

SET(CMAKE_INSTALL_PREFIX $ENV{ROBOTPKG_PATH})

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build walkGenJrl with shared libraries." ON)

# where are use-specific cmake modules
SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/CMakeModules)

# Check for required packages
# -------------------------
INCLUDE(FindPkgConfig)
pkg_check_modules(DYNJRLJAPAN REQUIRED dynamicsJRLJapan>=1.9.99 )

SET(PROJECT_REQUIREMENTS "dynamicsJRLJapan >= 1.9.99")

MESSAGE(STATUS "${DYNJRLJAPAN_CFLAGS}")

SET(${PROJECT_NAME}_CXXFLAGS "-Wall ${${PROJECT_NAME}_CXXFLAGS}")

FOREACH(option_cxxflags ${DYNJRLJAPAN_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_ldflags ${DYNJRLJAPAN_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

SET(${PROJECT_NAME}_LDFLAGS "-L${CMAKE_PREFIX_INSTALL}/lib/ -l${PROJECT_NAME}-${PROJECT_VERSION} ${${PROJECT_NAME}_LDFLAGS}")

#-------------------------
# Detect OmniOrb.
#------------------------- 
INCLUDE(${CMAKE_MODULE_PATH}/UseOmniORB4.cmake)
MESSAGE(STATUS "omniORB4: ${OMNIORB4_DIR}")

# Check for OpenHRP
# -----------------
INCLUDE(${CMAKE_MODULE_PATH}/FindOpenHRP.cmake)


# To deal with the headers.
#--------------------------
add_subdirectory(src)
add_subdirectory(plugin)
add_subdirectory(unitTesting)

# Prepare the documentation
# -------------------------
INCLUDE(FindDoxygen)
IF(DOXYGEN_FOUND)
  MESSAGE(STATUS "Doxygen found")
  add_subdirectory(doc)
ENDIF(DOXYGEN_FOUND)

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)






