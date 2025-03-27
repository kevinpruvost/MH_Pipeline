# CMakeList.txt : CMake project for VenomEngine, include source and define
# project specific logic here.
#
cmake_minimum_required(VERSION 3.1.15)

project("Libraries")

set(PROJECT_PATH ${CMAKE_SOURCE_DIR}/)

set(CMAKE_CXX_STANDARD 20)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(BUILD_SHARED_LIBS ON)

add_subdirectory("./OrbbecSDK")
