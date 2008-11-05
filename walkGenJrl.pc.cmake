prefix=${CMAKE_INSTALL_PREFIX}
exec_prefix=${install_pkg_prefix}/bin
libdir=${install_pkg_prefix}/lib
includedir=${install_pkg_prefix}/include
datarootdir=${install_pkg_prefix}/share
docdir=${install_pkg_prefix}/doc/

Name: ${PROJECT_NAME}
Description: ${PROJECT_DESCRIPTION}
Version: ${PROJECT_VERSION}
Requires: ${PROJECT_REQUIREMENTS}
Libs:  ${${PROJECT_NAME}_LDFLAGS}
Cflags: -I${install_pkg_include_dir} ${${PROJECT_NAME}_CXXFLAGS}
