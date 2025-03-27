
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Open3DConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 OLD)
endif()

include(CMakeFindDependencyMacro)
include(FindPackageHandleStandardArgs)

foreach(dep IN ITEMS )
    find_dependency(${dep})
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/Open3DTargets.cmake")

foreach(dep IN ITEMS Open3D::3rdparty_glew;Open3D::3rdparty_glfw;Open3D::3rdparty_fmt;Open3D::3rdparty_filament;Open3D::3rdparty_opengl)
    if(TARGET ${dep})
        get_property(inc TARGET ${dep} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
        if(inc)
            set_property(TARGET Open3D::Open3D APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${inc})
        endif()
        unset(inc)
        get_property(def TARGET ${dep} PROPERTY INTERFACE_COMPILE_DEFINITIONS)
        if(def)
            set_property(TARGET Open3D::Open3D APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS ${def})
        endif()
        unset(def)
    endif()
endforeach()

set_and_check(Open3D_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
set(Open3D_LIBRARIES Open3D::Open3D)
set(Open3D_VERSION "0.18.0")
set(Open3D_CONFIG ${CMAKE_CURRENT_LIST_FILE})

find_package_handle_standard_args(Open3D CONFIG_MODE)
