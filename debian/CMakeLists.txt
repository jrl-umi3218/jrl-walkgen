# Copyright 2011, Olivier Stasse, JRL, CNRS/AIST
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

IF(HRP2_14_FOUND)
  SET(PACKAGE_HRP2_14_DATA "hrp2-14-data (>= 1.7.7),")
ELSE(HRP2_14_FOUND)
  SET(PACKAGE_HRP2_14_DATA "")
ENDIF(HRP2_14_FOUND)

MESSAGE(STATUS "PACKAGE_HRP2_14_DATA:" ${PACKAGE_HRP2_14_DATA})

# Yes I know people will scream at the modification of the source
# but there is little choice...
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/control.cmake
	${CMAKE_CURRENT_SOURCE_DIR}/control
	@ONLY)
