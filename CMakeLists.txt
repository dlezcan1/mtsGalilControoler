#
# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (mts_galil_controller VERSION 0.0.0)

include_directories (include)

add_subdirectory (share)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES 
    cisstCommon
    cisstVector
    cisstOSAbstraction
    cisstMultiTask
    cisstParameterTypes
)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

set( catkin_REQUIRED_COMPONENTS 
    roscpp
    rospy
    std_msgs
    geometry_msgs
    cisst_msgs
    cisst_ros_bridge
)

find_package(catkin REQUIRED COMPONENTS
    ${catkin_REQUIRED_COMPONENTS}
)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES mtsGalilController
    CATKIN_DEPENDS ${catkin_REQUIRED_COMPONENTS}
)

set ( mtsGalilController_HEADER_FILES
    include/mtsGalilController/mtsGalilController.h
)
set ( mtsGalilController_SOURCE_FILES
    code/mtsGalilController.cpp
)

if (cisst_FOUND_AS_REQUIRED)

    # load cisst configuration
    include (${CISST_USE_FILE})

    if (NOT CISST_HAS_JSON)
        message( FATAL_ERROR "mtsGalilController requires cisst to be built with JSON" )
    endif ()

    # catkin/ROS paths
    cisst_set_output_path ()
    catkin_package(
        LIBRARIES mtsGalilController
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_REQUIRED_COMPONENTS}
    )

    # add all config files for this component
    cisst_add_config_files (mtsGalilController)
    
    add_library (mtsGalilController 
        ${mtsGalilController_SOURCE_FILES}
    )
    
    cisst_target_link_libraries (mtsGalilController ${REQUIRED_CISST_LIBRARIES})
    
    set_target_properties (mtsGalilController PROPERTIES
        VERSION ${mts_galil_controller_VERSION}
        FOLDER "mtsGalilController"
        PUBLIC_HEADER "${mtsGalilController_HEADER_FILES}"
    )
    target_include_directories(mtsGalilController PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries (mtsGalilController
        ${catkin_LIBRARIES}
        -lgclib
        -lgclibo                  
    )

    # Install target for headers and library
    install (
        DIRECTORY "${mts_galil_controller_SOURCE_DIR}/include/mtsGalilController"
        DESTINATION include
        COMPONENT mtsGalilController-dev
        PATTERN .svn EXCLUDE
    )

    install (
        TARGETS mtsGalilController 
        COMPONENT mtsGalilController
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
    )

    install(TARGETS mtsGalilController
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/mtsGalilController/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)