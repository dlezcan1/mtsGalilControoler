cmake_minimum_required (VERSION 3.10)
project (mts_galil_controller_config)

find_package (catkin QUIET)
if (catkin_FOUND)
    catkin_package()
endif ()

install (
    DIRECTORY "${mts_galil_controller_SOURCE_DIR}"
    DESTINATION share/mtsGalilController
    COMPONENT mtsGalilController-Share
    PATTERN "CMakeLists.txt" EXCLUDE
    PATTERN ".svn" EXCLUDE
)