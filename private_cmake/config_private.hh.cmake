// Copyright (C) 2008-2015 LAAS-CNRS, JRL AIST-CNRS.
//
// This file is part of jrl-walkgen.
// jrl-walkgen is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// jrl-walkgen is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.

#ifndef JRL_WALKGEN_CONFIG_PRIVATE_HH
# define JRL_WALKGEN_CONFIG__PRIVATE_HH

// Package version (header).
# define JRL_WALKGEN_CONFIG_VERSION "@PROJECT_VERSION@"

// 
# if @WITH_HRP2_DYNAMICS@
# define WITH_HRP2DYNAMICS
# endif 
#endif //! JRL_WALKGEN_CONFIG_PRIVATE_HH
