cmake_minimum_required (VERSION 3.10)
project (sawGalilController_config)

if (NOT WIN32)
    find_package (catkin QUIET)
    if (catkin_FOUND)
        catkin_package()
    endif ()
endif ()

install (
    DIRECTORY "${sawGalilController_SOURCE_DIR}"
    DESTINATION share/mtsGalilController
    COMPONENT mtsGalilController-Share
    PATTERN "CMakeLists.txt" EXCLUDE
    PATTERN ".svn" EXCLUDE
)
