cmake_minimum_required(VERSION 3.0)
project(gtsam_gnss CXX C)

# Enforce c++17 standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(-DBOOST_ALL_NO_LIB)
  set(GTSAMCMakeTools_DIR "C:/Program Files (x86)/GTSAM/CMake/GTSAMCMakeTools")
  set(gtwrap_DIR "C:/Program Files (x86)/GTSAM/CMake/gtwrap")
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/GTSAM")
endif(MSVC)

# Find GTSAM components
find_package(GTSAM REQUIRED)
if(GTSAM_NOT_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(GTSAM_NOT_FOUND)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

# Import gtwrap package
find_package(gtwrap)

set(WRAP_PYTHON_VERSION
    "Default"
    CACHE STRING "The Python version to use for wrapping")
gtwrap_get_python_version(${WRAP_PYTHON_VERSION})

set(WRAP_TOOLBOX_INSTALL_PATH
    "${CMAKE_INSTALL_PREFIX}/gtsam_toolbox"
    CACHE
      PATH
      "Matlab toolbox destination")

# Include the Matlab related code
include(MatlabWrap)

# Include project headers
include_directories("${PROJECT_SOURCE_DIR}/src")

# Add sorce code directory
add_subdirectory(src)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# ##############################################################################
# Build MATLAB wrapper
matlab_wrap("gtsam_gnss.i" "${PROJECT_NAME}" "" "" "${mexFlags}" "${FilesToIgnore}" OFF)
message("========== Configuration Options ==========")
message(STATUS "Project:                  ${PROJECT_NAME}")
message(STATUS "MATLAB Root:              ${MATLAB_ROOT}")
message(STATUS "Mex Compiler:             ${Matlab_MEX_COMPILER}")
message("===========================================")
