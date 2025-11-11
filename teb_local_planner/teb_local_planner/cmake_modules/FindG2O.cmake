cmake_minimum_required(VERSION 3.16)

# Headers
find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/base_vertex.h
  HINTS /usr/local/include /usr/local/include/g2o /usr/include /usr/include/g2o)

# helper to try a library and append if it exists
function(_try_lib _var _name)
  find_library(${_var} NAMES ${_name}
    HINTS /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib/x86_64-linux-gnu
    PATH_SUFFIXES lib x86_64-linux-gnu lib/x86_64-linux-gnu)
  if(NOT ${_var} STREQUAL "${_var}-NOTFOUND")
    list(APPEND G2O_LIBRARIES ${${_var}})
    set(G2O_LIBRARIES "${G2O_LIBRARIES}" PARENT_SCOPE)
  endif()
endfunction()

# core (bắt buộc)
_try_lib(G2O_CORE_LIBRARY g2o_core)

# các lib thường dùng (tự thêm nếu có)
_try_lib(G2O_STUFF_LIBRARY           g2o_stuff)
_try_lib(G2O_TYPES_SLAM2D_LIBRARY    g2o_types_slam2d)
_try_lib(G2O_TYPES_SLAM3D_LIBRARY    g2o_types_slam3d)
_try_lib(G2O_TYPES_SIM3_LIBRARY      g2o_types_sim3)
_try_lib(G2O_SOLVER_EIGEN_LIBRARY    g2o_solver_eigen)
_try_lib(G2O_SOLVER_CSPARSE_LIBRARY  g2o_csparse_extension)
_try_lib(G2O_SOLVER_CHOLMOD_LIBRARY  g2o_solver_cholmod)

set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O DEFAULT_MSG
  G2O_INCLUDE_DIR G2O_CORE_LIBRARY)

mark_as_advanced(
  G2O_INCLUDE_DIR
  G2O_CORE_LIBRARY
  G2O_STUFF_LIBRARY
  G2O_TYPES_SLAM2D_LIBRARY
  G2O_TYPES_SLAM3D_LIBRARY
  G2O_TYPES_SIM3_LIBRARY
  G2O_SOLVER_EIGEN_LIBRARY
  G2O_SOLVER_CSPARSE_LIBRARY
  G2O_SOLVER_CHOLMOD_LIBRARY)
