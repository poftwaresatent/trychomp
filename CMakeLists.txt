cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)

project (trychomp)

set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

### check_cxx_compiler_flag (-std=c++11 CXX_FLAG_cxx11)
### if (CXX_FLAG_cxx11)
###   add_definitions (-std=c++11)
### endif (CXX_FLAG_cxx11)
### check_cxx_compiler_flag (-stdlib=libc++ CXX_FLAG_cxx11lib)
### if (CXX_FLAG_cxx11lib)
###   add_definitions (-stdlib=libc++)
### endif (CXX_FLAG_cxx11lib)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

# if (WIN32)
#   add_definitions (-DWIN32)
# endif (WIN32)
# 
# if (CMAKE_SYSTEM_NAME MATCHES Linux)
#   add_definitions (-DLINUX)
# endif (CMAKE_SYSTEM_NAME MATCHES Linux)
# 
if (APPLE)
  add_definitions (-DOSX)
  include_directories (
    /opt/local/include
    /opt/local/include/eigen3
    )
  list (APPEND CMAKE_REQUIRED_INCLUDES
    /opt/local/include
    /opt/local/include/eigen3
    )
endif (APPLE)
# 
# if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
#   add_definitions (-DOPENBSD)
#   list (APPEND CMAKE_REQUIRED_INCLUDES
#     /usr/local/include
#     )
#   include_directories (
#     /usr/local/include
#     )
#   link_directories (/usr/local/lib)
# endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)

include (CheckIncludeFileCXX)

check_include_file_cxx (Eigen/Core HAVE_EIGEN3)
if (HAVE_EIGEN3)
  message ("*** found Eigen3 (required)")
else (HAVE_EIGEN3)
  message (FATAL_ERROR "Please install Eigen3.")
endif (HAVE_EIGEN3)

find_package(GTK2 REQUIRED gtk)
if (GTK2_FOUND)
  message ("*** found GTK2 (required)")
  include_directories (${GTK2_INCLUDE_DIRS})
else (GTK2_FOUND)
  message (FATAL_ERROR "Please install GTK2 development files.")
endif (GTK2_FOUND)

##################################################

add_executable (pp2d pp2d.cpp gfx.cpp)
target_link_libraries (pp2d ${GTK2_LIBRARIES})

add_executable (pprrr2d pprrr2d.cpp gfx.cpp)
target_link_libraries (pprrr2d ${GTK2_LIBRARIES})

add_executable (conjgrad conjgrad.cpp)
