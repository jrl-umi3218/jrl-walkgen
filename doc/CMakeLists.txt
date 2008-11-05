# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST
SET(${${PROJECT_NAME}_SOURCE_DIR}/doc/docfiles
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/header.html
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/footer.html
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.css
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/pictures/footer.jpg
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/additionalDoc/package.h
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox
    )

# Configure package.dox file
CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox.cmake
  ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox
)


ADD_CUSTOM_COMMAND(OUTPUT ${${PROJECT_NAME}_SOURCE_DIR}/doc/html/index.html
                   COMMAND ${DOXYGEN_EXECUTABLE} ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox
                   DEPENDS ${docfiles}
)

ADD_CUSTOM_TARGET(doc ALL
                  DEPENDS ${${PROJECT_NAME}_SOURCE_DIR}/doc/html/index.html)

INSTALL(DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/doc/html
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}/)
INSTALL(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/doc/pictures
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}/html/)

