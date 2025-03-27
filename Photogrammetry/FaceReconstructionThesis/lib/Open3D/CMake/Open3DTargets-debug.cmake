#----------------------------------------------------------------
# Generated CMake target import file for configuration "DEBUG".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Open3D::tbb" for configuration "DEBUG"
set_property(TARGET Open3D::tbb APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Open3D::tbb PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/debug/tbb12_debug.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/debug/tbb12_debug.dll"
  )

list(APPEND _cmake_import_check_targets Open3D::tbb )
list(APPEND _cmake_import_check_files_for_Open3D::tbb "${_IMPORT_PREFIX}/lib/debug/tbb12_debug.lib" "${_IMPORT_PREFIX}/bin/debug/tbb12_debug.dll" )

# Import target "Open3D::Open3D" for configuration "DEBUG"
set_property(TARGET Open3D::Open3D APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Open3D::Open3D PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/debug/Open3D.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "Open3D::tbb"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/debug/Open3D.dll"
  )

list(APPEND _cmake_import_check_targets Open3D::Open3D )
list(APPEND _cmake_import_check_files_for_Open3D::Open3D "${_IMPORT_PREFIX}/lib/debug/Open3D.lib" "${_IMPORT_PREFIX}/bin/debug/Open3D.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
