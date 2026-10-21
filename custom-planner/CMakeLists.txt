qt_add_library(ChimeraPlannerModule STATIC)

set(planner_qml_files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BlockEditor.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BlockMap.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetBar.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SpeedOverlay.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetOverlay.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetSettings.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetSetup.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SurveyEditor.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SurveyOverlay.qml
)
qgc_set_qt_resource_alias(${planner_qml_files})
qt_add_qml_module(ChimeraPlannerModule
    URI Chimera.Planner
    VERSION 1.0
    RESOURCE_PREFIX /qml
    QML_FILES ${planner_qml_files}
    NO_PLUGIN
)

set(CUSTOM_DEFINITIONS QGC_CUSTOM_BUILD CUSTOMHEADER="CustomPlugin.h" CUSTOMCLASS=CustomPlugin CACHE INTERNAL "" FORCE)
set(QGC_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/planner.qrc CACHE STRING "Custom resources" FORCE)
set(CUSTOM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetPlanner.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FleetPlanner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Splitters/ControlPointSplitter.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ChimeraPlanner.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ChimeraPlanner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ChimeraMissionItem.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ChimeraMissionItem.h
    CACHE INTERNAL "" FORCE)
set(CUSTOM_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE INTERNAL "" FORCE)
set(CUSTOM_LIBRARIES ChimeraPlannerModule CACHE INTERNAL "" FORCE)

option(QGC_PLANNER_SMOKE_TEST "Enable the offline planner function check" OFF)
if(QGC_PLANNER_SMOKE_TEST)
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.cc
        DIRECTORY ${CMAKE_SOURCE_DIR}
        PROPERTIES COMPILE_DEFINITIONS QGC_PLANNER_SMOKE_TEST)
    set(CUSTOM_SOURCES ${CUSTOM_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/tests/PlannerSmoke.cc ${CMAKE_CURRENT_SOURCE_DIR}/tests/PlannerLogic.cc CACHE INTERNAL "" FORCE)
endif()
