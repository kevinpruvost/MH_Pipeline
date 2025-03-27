# CMakeList.txt : CMake project for VenomEngine, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
project(ThesisApplication_OrbbecSDK_Without_IMU_PnP)

set(CMAKE_CXX_STANDARD 20)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project (${PROJECT_NAME})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add source to this project's executable.
add_executable (${PROJECT_NAME}
    "main.cc"
    "../Application_OrbbecSDK/tools.cc"
    "../Application_OrbbecSDK/tools.h"
    "PointCloudConstruction.cc"
    "PointCloudConstruction.h"
)

# link Orbbec SDK
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE "${PROJECT_PATH}/include")

include("${OrbbecSDK_ROOT_DIR}/cmake/DetermineHostPlatform.cmake")

# Ensure the copy_orbbec_sdk_libs target runs after the main application build
add_dependencies(${PROJECT_NAME} copy_orbbec_sdk_libs)