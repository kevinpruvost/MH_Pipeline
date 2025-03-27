# CMakeList.txt : CMake project for VenomEngine, include source and define
# project specific logic here.
#
cmake_minimum_required(VERSION 3.8)

project("FaceReconstructionThesis")

set(CMAKE_DEBUG_POSTFIX "")

set(PROJECT_PATH ${CMAKE_SOURCE_DIR}/)
set(EXTERNAL_LIBS_PATH ${CMAKE_SOURCE_DIR}/lib/ExternalLibs/)
set(LIBS_PATH ${CMAKE_SOURCE_DIR}/lib/)

set(CMAKE_CXX_STANDARD 20)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR})

set(MAIN_APP_NAME "Application_OrbbecSDK")

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})

IF(UNIX)
    set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
ENDIF()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# find opencv
# set(OpenCV_DIR "your/path/to/opencv") # Alternative you can input cmake parameter from when cmake building
if(NOT DEFINED OpenCV_DIR)
  # Get OpenCV from system environment
  if(DEFINED ENV{OpenCV_INSTALL_PATH})
    set(OpenCV_DIR $ENV{OpenCV_INSTALL_PATH})
  else ()
    message("Install OpenCV, refer to the environment configuration tutorial")
  endif()
endif()
find_package(OpenCV REQUIRED)
if(DEFINED OpenCV_LIBS AND DEFINED OpenCV_INCLUDE_DIRS)
  set(OpenCV_FOUND TRUE)
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  set(OpenCV_FOUND FALSE)
endif()

add_subdirectory("./lib/")
# Find Open3D package if it provides Open3DConfig.cmake
list(APPEND CMAKE_PREFIX_PATH "./lib/Open3D/CMake")
# find_package(Open3D REQUIRED)
# find PCL
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

set(OrbbecSDK_ROOT_DIR ${LIBS_PATH}/OrbbecSDK)
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(UNIX)
#linux system set sample rpath
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:${CMAKE_INSTALL_RPATH}:${OrbbecSDK_LIBRARY_DIRS}")
    if(NOT APPLE)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()
endif()

include_directories("./include/")

# Executable
add_subdirectory(${PROJECT_PATH}/Application_OrbbecSDK)
add_subdirectory(${PROJECT_PATH}/Application_OrbbecSDK_WithoutIMU_PnP)
add_subdirectory(${PROJECT_PATH}/Application_OrbbecSDK_WithoutIMU_PnP_Batched)
# add_subdirectory(${PROJECT_PATH}/Application_OrbbecSDK_WithoutIMU_ICP)
add_subdirectory(${PROJECT_PATH}/Application_OFRC)
# add_subdirectory(${PROJECT_PATH}/PointCloudRegistration)