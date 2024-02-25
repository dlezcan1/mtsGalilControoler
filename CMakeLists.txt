#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawGalilController VERSION 0.1.0)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set (sawGalilController_INCLUDE_DIR "${sawGalilController_SOURCE_DIR}/include")
include_directories (${sawGalilController_INCLUDE_DIR})

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

# Find Galil gclib
find_package (gclib REQUIRED)

include_directories (${gclib_INCLUDE_DIR})
link_directories (${gclib_LIBRARY_DIR})

if (NOT WIN32)
    set (catkin_REQUIRED_COMPONENTS
         roscpp
         rospy
         std_msgs
         geometry_msgs
         cisst_msgs
         cisst_ros_bridge)

    find_package(catkin REQUIRED COMPONENTS ${catkin_REQUIRED_COMPONENTS})
    include_directories(${catkin_INCLUDE_DIRS})

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sawGalilController
        CATKIN_DEPENDS ${catkin_REQUIRED_COMPONENTS})

endif ()

if (cisst_FOUND_AS_REQUIRED)

    set (sawGalilController_HEADER_FILES
         "${sawGalilController_INCLUDE_DIR}/sawGalilController/mtsGalilController.h"
         "${sawGalilController_INCLUDE_DIR}/sawGalilController/sawGalilControllerExport.h")
    set (sawGalilController_SOURCE_FILES
         code/mtsGalilController.cpp)

    # load cisst configuration
    include (${CISST_USE_FILE})

    if (NOT CISST_HAS_JSON)
        message( FATAL_ERROR "sawGalilController requires cisst to be built with JSON" )
    endif ()

    # catkin/ROS paths
    cisst_set_output_path ()

    if (NOT WIN32)
        catkin_package(
            LIBRARIES sawGalilController
            INCLUDE_DIRS include
            CATKIN_DEPENDS ${catkin_REQUIRED_COMPONENTS}
        )

        # add all config files for this component
        cisst_add_config_files (sawGalilController)
    endif ()
    
   add_library (sawGalilController ${IS_SHARED}
                ${sawGalilController_HEADER_FILES}
                ${sawGalilController_SOURCE_FILES})
    
    cisst_target_link_libraries (sawGalilController ${REQUIRED_CISST_LIBRARIES})
    
    set_target_properties (sawGalilController PROPERTIES
        VERSION ${sawGalilController_VERSION}
        FOLDER "sawGalilController"
        PUBLIC_HEADER "${sawGalilController_HEADER_FILES}")

    target_include_directories(sawGalilController PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

    target_link_libraries (sawGalilController ${gclib_LIBRARIES})

    # Install target for headers and library
    install (
        DIRECTORY "${sawGalilController_SOURCE_DIR}/include/sawGalilController"
        DESTINATION include
        COMPONENT sawGalilController-dev
        PATTERN .svn EXCLUDE
    )

    install (
        TARGETS sawGalilController 
        COMPONENT sawGalilController
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
    )

    if (NOT WIN32)
        install(TARGETS sawGalilController
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

        install(DIRECTORY include/sawGalilController/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
    endif ()

else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
