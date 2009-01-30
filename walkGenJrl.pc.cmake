prefix=${CMAKE_INSTALL_PREFIX}
exec_prefix=${install_pkg_prefix}/bin
libdir=${install_pkg_prefix}/lib
includedir=${install_pkg_prefix}/include
datarootdir=${install_pkg_prefix}/share
docdir=${install_pkg_datarootdir}/doc/${PROJECT_NAME}

Name: ${PROJECT_NAME}
Description: ${PROJECT_DESCRIPTION}
Version: ${PROJECT_VERSION}
Requires: ${PROJECT_REQUIREMENTS}
Libs:  ${LIBDIR_KW}${install_pkg_libdir} ${LIBINCLUSION_KW}${PROJECT_NAME}
Cflags: -I${install_pkg_include_dir} ${${PROJECT_NAME}_PC_CXXFLAGS}
