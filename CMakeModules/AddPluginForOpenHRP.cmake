# Olivier Stasse, JRL, CNRS/AIST
# Creation: 02/07/2008
# History:
#
# Copyright CNRS/AIST

MACRO(ADD_OPENHRP_PLUGIN PLUGINNAME PLUGINNAME_SRCS PLUGINCORBAIDL WORKINGDIRIDL PLUGINLINKS CORBAPLUGINDEPS)

GET_FILENAME_COMPONENT(PluginBaseName ${PLUGINNAME} NAME)
GET_FILENAME_COMPONENT(PluginBasePath ${PLUGINNAME} PATH)
GET_FILENAME_COMPONENT(PluginBaseNameIDL ${PLUGINCORBAIDL} NAME_WE)

SET(name_ofplugincorbaIDL ${PLUGINCORBAIDL})
#MESSAGE(STATUS ":name_ofplugincorbaIDL:" ${name_ofplugincorbaIDL})

SET(all_sourcefiles_for_plugin ${PLUGINNAME_SRCS})

# Build the path of the target
SET(${plugin_final_position} "${openhrp_final_plugin_path}/${PLUGINNAME}")

IF (NOT ${PLUGINNAME_SRCS})
   SET(PLUGINNAME_SRCS "${PluginBasePath}/${PluginBaseName}.cpp")
ENDIF (NOT ${PLUGINNAME_SRCS})

#MESSAGE(STATUS "Sources files for ${PLUGINNAME} : ${PLUGINNAME_SRCS} ")

SET(IDL_INCLUDE_DIR "${OPENHRP_HOME}/Common/corba" )
IF (OPENHRP_VERSION_3)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/DynamicsSimulator/corba)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/Controller/IOserver/corba)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/Controller/IOserver/plugin/SequencePlayer/corba)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/CollisionDetector/corba/)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/ViewSimulator/corba/)
  SET(IDL_INCLUDE_DIR ${IDL_INCLUDE_DIR} ${OPENHRP_HOME}/ModelLoader/corba/)
ENDIF(OPENHRP_VERSION_3)
#MESSAGE(STATUS "Idl include dir: ${IDL_INCLUDE_DIR} ")

IF   ( EXISTS "${name_ofplugincorbaIDL}" )

  SET(plugincorbaidl_CPP "${WORKINGDIRIDL}/${PluginBaseNameIDL}SK.cc")
  SET(plugincorbaidl_Header "${WORKINGDIRIDL}/${PluginBaseNameIDL}.h")
  IDLFILERULE(${PLUGINCORBAIDL} ${plugincorbaidl_CPP} ${plugincorbaidl_Header} ${WORKINGDIRIDL}
              ${IDL_INCLUDE_DIR})

  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${plugincorbaidl_CPP})
  
  IF (OPENHRP_VERSION_2)
    SET(CommonFileName common)
  ENDIF (OPENHRP_VERSION_2)

  IF (OPENHRP_VERSION_3)
    SET(CommonFileName OpenHRPCommon)
  ENDIF (OPENHRP_VERSION_3)

  # MESSAGE(STATUS "IDL Common file name : ${CommonFileName}.idl")
  SET(common_CPP "${WORKINGDIRIDL}/${CommonFileName}SK.cc")
  SET(common_Header "${WORKINGDIRIDL}/${CommonFileName}.h" )	
  
  IDLFILERULE(${OPENHRP_HOME}/Common/corba/${CommonFileName}.idl 
              ${common_CPP}
              ${common_Header} ${WORKINGDIRIDL} 
              ${IDL_INCLUDE_DIR})

  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${common_CPP})

  IF (OPENHRP_VERSION_3)
    SET(IDL_FILES_FOR_OPENHRP
        ${OPENHRP_HOME}/Controller/IOserver/corba/HRPcontroller.idl
	${OPENHRP_HOME}/ViewSimulator/corba/ViewSimulator.idl
	${OPENHRP_HOME}/DynamicsSimulator/corba/DynamicsSimulator.idl
	${OPENHRP_HOME}/ModelLoader/corba/ModelLoader.idl	
	${OPENHRP_HOME}/CollisionDetector/corba/CollisionDetector.idl	
     )
  ENDIF(OPENHRP_VERSION_3)

  IF (OPENHRP_VERSION_2)
    SET(IDL_FILES_FOR_OPENHRP
        ${OPENHRP_HOME}/Controller/IOserver/sys/plugin/dynamicsPlugin.idl
     )
  ENDIF(OPENHRP_VERSION_2)

    FOREACH( locidlfile ${IDL_FILES_FOR_OPENHRP})

      GET_FILENAME_COMPONENT(locIDLBaseName ${locidlfile} NAME)
      GET_FILENAME_COMPONENT(locIDLBasePath ${locidlfile} PATH)
      GET_FILENAME_COMPONENT(locIDLBaseNameWE ${locidlfile} NAME_WE)

      SET(locIDL_CPP "${WORKINGDIRIDL}/${locIDLBaseNameWE}SK.cc")
      SET(locIDL_Header "${WORKINGDIRIDL}/${locIDLBaseNameWE}.h" )	
  
      IDLFILERULE(${locidlfile} 
              ${locIDL_CPP}
              ${locIDL_Header} ${WORKINGDIRIDL} 
              ${IDL_INCLUDE_DIR})

      SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${locIDL_CPP})

    ENDFOREACH(locidlfile)


ELSE ( EXISTS "${name_ofplugincorbaIDL}" )
  MESSAGE(STATUS "${name_ofplugincorbaIDL} empty")
ENDIF( EXISTS "${name_ofplugincorbaIDL}" )

FOREACH( corbaplugin ${CORBAPLUGINDEPS})

  GET_FILENAME_COMPONENT(corbaplugin_basename ${corbaplugin} NAME_WE)
  SET(corbaplugin_CPP "${WORKINGDIRIDL}/${corbaplugin_basename}SK.cc")
  SET(corbaplugin_Header "${WORKINGDIRIDL}/${corbaplugin_basename}.h" )	
  IDLFILERULE(${corbaplugin}
              ${corbaplugin_CPP}
              ${corbaplugin_Header} ${WORKINGDIRIDL}  
              ${IDL_INCLUDE_DIR})
  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${corbaplugin_CPP})


ENDFOREACH(corbaplugin)
#MESSAGE(STATUS "all_sourcefiles_for_plugin : ${all_sourcefiles_for_plugin}")
#MESSAGE(STATUS "PluginName : ${PLUGINNAME}")
ADD_LIBRARY(${PluginBaseName} ${all_sourcefiles_for_plugin})

SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${${PROJECT_NAME}_SOURCE_DIR}/include")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} ${OPENHRP_CXX_FLAGS}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/include")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Common")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/common")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/sys/plugin")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${WORKINGDIRIDL}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} ${${PROJECT_NAME}_CXXFLAGS}")


SET(openhrp_plugin_ldflags "${openhrp_plugin_ldflags}  -L${${PROJECT_NAME}_BINARY_DIR}/src")


SET(openhrp_plugin_path "${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}/bin")
SET(openhrp_plugin_filenamepath "${openhrp_plugin_path}/${PluginBaseName}.so")

#MESSAGE(STATUS "openhrp_plugin_filenamepath ${openhrp_plugin_filenamepath}")
GET_TARGET_PROPERTY(PluginFinalFileNamePrefix ${PluginBaseName} PREFIX)
GET_TARGET_PROPERTY(PluginFinalFileNameSuffix ${PluginBaseName} SUFFIX)	              
#MESSAGE(STATUS "Prefix: ${PluginFinalFileNamePrefix} Suffix: ${PluginFinalFileNameSuffix}")

SET(loc_compile_flags "${omniORB4_cflags} ${openhrp_plugin_cflags}  ${${PROJECT_NAME}_src_CXXFLAGS}")

IF (OPENHRP_VERSION_2)
  SET(loc_compile_flags "${loc_compile_flags} -DOPENHRP_VERSION_2")
ENDIF (OPENHRP_VERSION_2)

IF (OPENHRP_VERSION_3)
  SET(loc_compile_flags "${loc_compile_flags} -DOPENHRP_VERSION_3 -I${OPENHRP_HOME}/DynamicsSimulator/server/")
ENDIF (OPENHRP_VERSION_3)

SET_TARGET_PROPERTIES(${PluginBaseName}
			PROPERTIES	
		        COMPILE_FLAGS "${loc_compile_flags}"
			LINK_FLAGS "${omniORB4_link_FLAGS} ${PLUGINLINKS}  ${${PROJECT_NAME}_src_LDFLAGS} ${openhrp_plugin_ldflags}"
			PREFIX ""
			SUFFIX ".so"
			LIBRARY_OUTPUT_DIRECTORY ${openhrp_plugin_path})

#SET(AlwaysCopyPlugin "${PluginBaseName}AlwaysCopyPlugin")
#ADD_CUSTOM_TARGET( ${AlwaysCopyPlugin} ALL
#	           ${CMAKE_COMMAND} -E copy ${LIBRARY_OUTPUT_PATH}/${PluginBaseName}.so ${openhrp_plugin_filenamepath}
#		   DEPENDS ${PluginBaseName}
#		   )

ENDMACRO(ADD_OPENHRP_PLUGIN)