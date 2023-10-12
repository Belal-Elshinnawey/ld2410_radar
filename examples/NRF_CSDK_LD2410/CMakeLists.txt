cmake_minimum_required(VERSION 3.20.0)
set(LIB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/ld2410_radar)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(radar53)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

zephyr_get_include_directories_for_lang_as_string(       C includes)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C definitions)
zephyr_get_compile_options_for_lang_as_string(           C options)
set(external_project_cflags
  "${includes} ${definitions} ${options} ${system_includes}"
  )
include(ExternalProject)

set(ld2410_radar_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/ld2410_radar)
set(ld2410_radar_build_dir ${CMAKE_CURRENT_BINARY_DIR}/ld2410_radar)

set(LD2410_RADAR_LIB_DIR     ${ld2410_radar_build_dir}/lib)
set(LD2410_RADAR_INCLUDE_DIR ${ld2410_radar_src_dir}/src)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

ExternalProject_Add(
  ld2410_radar_project                
  PREFIX     ${ld2410_radar_build_dir}
  SOURCE_DIR ${ld2410_radar_src_dir}
  BINARY_DIR ${ld2410_radar_src_dir}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND
  ${submake}
  PREFIX=${ld2410_radar_build_dir}
  CC=${CMAKE_C_COMPILER}
  AR=${CMAKE_AR}
  CFLAGS=${external_project_cflags}
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${LD2410_RADAR_LIB_DIR}/libld2410_radar.a
)
add_library(ld2410_radar_lib STATIC IMPORTED GLOBAL)
add_dependencies(
  ld2410_radar_lib
  ld2410_radar_project
)
set_target_properties(ld2410_radar_lib PROPERTIES IMPORTED_LOCATION             ${LD2410_RADAR_LIB_DIR}/libld2410_radar.a)
set_target_properties(ld2410_radar_lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LD2410_RADAR_INCLUDE_DIR})

target_link_libraries(app PUBLIC ld2410_radar_lib)
