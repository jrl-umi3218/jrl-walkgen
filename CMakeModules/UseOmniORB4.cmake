  SET(omniORB4_include_DIR "")
  SET(omniORB4_link_DIR "")
  SET(omniORB4_link_FLAGS "")
  SET(omniORB4_cflags "")
  SET(omniORB4_IDLC "")


#PKGCONFIG(omniORB4 omniORB4_include_DIR omniORB4_link_DIR omniORB4_link_FLAGS omniORB4_cflags)
pkg_search_module(omniORB4 omniORB4)

FIND_PROGRAM(PKGCONFIG_EXECUTABLE NAMES pkg-config PATHS /usr/local/bin )

IF(PKGCONFIG_EXECUTABLE)
  EXEC_PROGRAM(${PKGCONFIG_EXECUTABLE} ARGS "omniORB4" --variable=omniidl
             OUTPUT_VARIABLE omniORB4_IDLC )

  #MESSAGE (STATUS "omniORB4_IDLC: ${omniORB4_IDLC}")
  EXEC_PROGRAM(${PKGCONFIG_EXECUTABLE} ARGS "omniORB4" --variable=prefix
             OUTPUT_VARIABLE OMNIORB4_DIR )

  EXEC_PROGRAM(${PKGCONFIG_EXECUTABLE} ARGS "omniORB4" --libs
             OUTPUT_VARIABLE omniORB4_link_FLAGS )
ENDIF(PKGCONFIG_EXECUTABLE)

# Try to find OMNIORB4_DIR
IF (NOT OMNIORB4_DIR)
  MESSAGE (STATUS "Did not find omniorb4_dir")
  IF(ROBOTPACKAGE_DIR)
    SET(OMNIORB4_DIR ${ROBOTPACKAGE_DIR})
  ELSE(ROBOTPACKAGE_DIR)
  # TODO: Should try to find it with Robotpackage. 
  # So far not that successfull.
  ENDIF(ROBOTPACKAGE_DIR)

  SET(omniORB4_include_DIR "${OMNIORB4_DIR}/include")
  SET(omniORB4_link_DIR "-L${OMNIORB4_DIR}/include")
  SET(omniORB4_link_FLAGS "-lomniORB4 -lomnithread -lomniDynamic4")
  SET(omniORB4_cflags "-DOMNIORB4 -I${OMNIORB4_DIR}/include -DOMNIORB4 -D__x86__ -DVERSION=2  -DNDEBUG")
  SET(omniORB4_IDLC "${OMNIORB4_DIR}/bin/omniidl")
ELSE(NOT OMNIORB4_DIR)
  MESSAGE (STATUS "Did found omniorb4_dir ${OMNIORB4_DIR}")
  IF (omniORB4_include_DIR)
    SET(omniORB4_cflags "-DOMNIORB4  -DNDEBUG -I${omniORB4_include_DIR}")
  ELSE(omniORB4_include_DIR)
    SET(omniORB4_cflags "-DOMNIORB4  -DNDEBUG")
  ENDIF(omniORB4_include_DIR)

ENDIF(NOT OMNIORB4_DIR)
	  
#MESSAGE(STATUS "OmniORB4_IDLC: ${omniORB4_IDLC}")
IF(OMNIORB4_DIR)


  SET(idlextension "\\.idl")
  MACRO(IDLFILERULE IDLFILE TargetFile_CPP TargetFile_Header IDL_WORKING_DIR IDL_INCLUDE_DIR)
    GET_FILENAME_COMPONENT(tidlfilename ${IDLFILE} NAME)
    
    #MESSAGE(STATUS "Please look at : ${IDL_WORKING_DIR}")
    #MESSAGE(STATUS "Please look at : ${IDL_INCLUDE_DIR}")	
    SET(tidlfilepath ${IDL_WORKING_DIR})
    IF (tidlfilepath)
      SET(tidlfilepath ${IDL_WORKING_DIR})
    ELSE(tidlfilepath)
      #MESSAGE(STATUS "Please look at : ${IDL_WORKING_DIR} is empty !")
      GET_FILENAME_COMPONENT(tidlfilepath ${IDLFILE} PATH)	
    ENDIF(tidlfilepath)

    #MESSAGE(STATUS "The idlpath: ${tidlfilepath}")	
    STRING(REGEX REPLACE "${idlextension}" "" idlfilename ${tidlfilename})
    #MESSAGE(STATUS "The idlfile: ${IDLFILE} ${idlfilename} ${TargetFile_CPP} ${TargetFile_Header}")
	
    SET(optionslcustomcommand "")
    IF (IDL_INCLUDE_DIR)	
      #MESSAGE(STATUS "inside useomniorb4 ${IDL_INCLUDE_DIR}")
      SET(optionslcustomcommand  ${optionslcustomcommand} -I${IDL_INCLUDE_DIR})
      FOREACH(lincdir ${ARGN})
        SET(optionslcustomcommand  ${optionslcustomcommand} -I${lincdir})
      ENDFOREACH(lincdir)
    ENDIF(IDL_INCLUDE_DIR)
    #MESSAGE(STATUS "optionslcustomcommand:" ${optionslcustomcommand})
    #ENDIF(EXISTS "${IDL_INCLUDE_DIR}")

    SET(commandtorealize ${omniORB4_IDLC} ${optionslcustomcommand} -bcxx -Wbh=\".h\" ${IDLFILE})
    #MESSAGE(STATUS "${TargetFile_CPP} ${TargetFile_Header} ${commandtorealize}")
    ADD_CUSTOM_COMMAND( OUTPUT ${TargetFile_CPP} ${TargetFile_Header} 
        COMMAND ${commandtorealize}
        WORKING_DIRECTORY ${tidlfilepath}
        MAIN_DEPENDENCY ${IDLFILE}
    )


    SET_SOURCE_FILES_PROPERTIES(${TargetFile_CPP} PROPERTIES MAIN_DEPENDENCY ${IDLFILE})
    #ADD_DEPENDENCIES(${TargetFile_CPP}  ${IDLFILE})
    SET_SOURCE_FILES_PROPERTIES(${TargetFile_Header} PROPERTIES MAIN_DEPENDENCY ${IDLFILE})
    #ADD_DEPENDENCIES(${TargetFile_Header}  ${IDLFILE})

  ENDMACRO(IDLFILERULE IDLFILE TargetFile_CPP TargetFile_Header)


  SET(OMNIORB4_FOUND TRUE)
ELSE(OMNIORB4_DIR)
  SET(OMNIORB4_FOUND FALSE)
ENDIF(OMNIORB4_DIR)