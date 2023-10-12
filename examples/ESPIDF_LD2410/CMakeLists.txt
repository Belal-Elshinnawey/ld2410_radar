cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tkrm_lock_v3)

littlefs_create_partition_image(data data)
