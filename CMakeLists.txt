cmake_minimum_required(VERSION 3.15)

project(mdk-rockchip VERSION 0.33.1 DESCRIPTION "mdk rockchip plugin")
set(PROJECT_VERSION_TWEAK 0)

set(MDKSDK "${CMAKE_BINARY_DIR}/mdk-sdk" CACHE STRING "libmdk SDK dir")

if(NOT CMAKE_PROJECT_NAME STREQUAL mdk) # not build in source tree
    list(APPEND CMAKE_MODULE_PATH ${MDKSDK}/lib/cmake)
endif()
find_package(MDK)

set(CMAKE_CXX_STANDARD 26)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

check_library_exists(rockchip_mpp mpp_create "" HAVE_MPP)
if(NOT HAVE_MPP)
  message("rockchip_mpp library not found")
  return()
endif()

setup_mdk_plugin(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    MppDecoder.cpp
)

include(FindPkgConfig)
pkg_check_modules(PKG_LIBDRM libdrm)
if(PKG_LIBDRM_FOUND)
  if(CMAKE_CROSSCOMPILING)
    string(REPLACE "-I/usr" "-I=/usr" PKG_LIBDRM_CFLAGS "${PKG_LIBDRM_CFLAGS}") # -I/usr/include/libdrm
  endif()
  string(REPLACE ";" " " PKG_LIBDRM_CFLAGS "${PKG_LIBDRM_CFLAGS}")
  add_compile_options(${PKG_LIBDRM_CFLAGS})
  set_property(SOURCE MppDecoder.cpp APPEND PROPERTY COMPILE_FLAGS "${PKG_LIBDRM_CFLAGS}")
  #list(APPEND EXTRA_DYLIBS ${PKG_LIBDRM_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC rockchip_mpp)
