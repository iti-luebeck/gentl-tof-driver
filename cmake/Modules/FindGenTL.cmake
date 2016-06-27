###############################################################################
# Find QVTK
#
# This sets the following variables:
# QVTK_FOUND - True if QVTK was found.
# QVTK_INCLUDE_DIR - Directory containing the QVTK include files.
# QVTK_LIBRARY - QVTK library.
# if QVTK_FOUND then QVTK_INCLUDE_DIR is appended to VTK_INCLUDE_DIRS and 
# QVTK_LIBRARY is appended to QVTK_LIBRARY_DIR

find_library (GENTL_LIBRARY GenApi_gcc_v3_0_Basler_pylon_v5_0 PATHS /opt/BaslerToF/lib /opt/BaslerToF/lib64)
find_library (GENTL_GCBASE_LIBRARY GCBase_gcc_v3_0_Basler_pylon_v5_0 PATHS /opt/BaslerToF/lib /opt/BaslerToF/lib64)
#find_library (GENTL_GX_LIBRARY gxapi PATHS /opt/BaslerToF/lib /opt/BaslerToF/lib64)
#find_library (GENTL_PYLON_BASE_LIBRARY pylonbase PATHS /opt/BaslerToF/lib /opt/BaslerToF/lib64)
#find_library (GENTL_PYLON_UTILITY_LIBRARY pylonutility PATHS /opt/BaslerToF/lib /opt/BaslerToF/lib64)
find_library (DL_LIBRARY dl)
find_path (GENTL_INCLUDE_DIR GenICam.h PATHS /opt/BaslerToF/include)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GenTL DEFAULT_MSG GENTL_LIBRARY GENTL_INCLUDE_DIR)

if(NOT GENTL_FOUND)
  message (gentl not found)
else(NOT GENTL_FOUND)
  get_filename_component (GENTL_LIBRARY_DIR ${GENTL_LIBRARY} PATH)
  set (GENTL_LIBRARIES ${GENTL_LIBRARY} ${GENTL_GCBASE_LIBRARY} ${DL_LIBRARY})# ${GENTL_GX_LIBRARY} ${GENTL_PYLON_BASE_LIBRARY} ${GENTL_PYLON_UTILITY_LIBRARY}")
  set (GENTL_LIBRARY_DIRS ${GENTL_LIBRARY_DIR})
  set (GENTL_INCLUDE_DIRS ${GENTL_INCLUDE_DIR})
endif(NOT GENTL_FOUND)

message(WARNING, "cmake GENTL_LIBRARIES: " ${GENTL_LIBRARIES})
