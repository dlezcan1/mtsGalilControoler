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
project (mtsGalilController VERSION 0.0.0)

include_directories (include)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES 
    cisstCommon
    cisstVector
    cisstOSAbstraction
    cisstMultiTask
    cisstParameterTypes
)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

    # load cisst configuration
    include (${CISST_USE_FILE})

    if (NOT CISST_HAS_JSON)
        message( FATAL_ERROR "mtsGalilController requires cisst to be built with JSON" )
    endif ()

    # catkin/ROS paths
    cisst_set_output_path ()

    # add all config files for this component
    cisst_add_config_files (mtsGalilController)
    
    add_library (mtsGalilController 
        include/mtsGalilController/GalilControllerInterface.h
        code/GalilControllerInterface.cpp
        include/mtsGalilController/mtsGalilController.h
        code/mtsGalilController.cpp
    )
    add_executable(mtsGalilController_Test 
        code/main.cpp
    )
    target_link_libraries(mtsGalilController_Test mtsGalilController)
    
    cisst_target_link_libraries(mtsGalilController_Test ${REQUIRED_CISST_LIBRARIES})

    set_target_properties (mtsGalilController PROPERTIES
                            VERSION ${mtsGalilController_VERSION}
                            FOLDER "mtsGalilController"
    )
    target_link_libraries (mtsGalilController
        ${mtsGalilController_INCLUDE_DIR}
        -lgclib
        -lgclibo                  
    )
    cisst_target_link_libraries (mtsGalilController ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (
        DIRECTORY "${mtsGalilController_SOURCE_DIR}/include/mtsGalilController"
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

else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)