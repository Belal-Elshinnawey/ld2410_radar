
set(CMAKE_OBJECT_PATH_MAX 400)

if((CONFIG_BOARD_RADAR53_CPUAPP OR CONFIG_BOARD_RADAR53_CPUAPP_NS)
  AND CONFIG_BOARD_ENABLE_CPUNET)
  zephyr_library()
  zephyr_library_sources(nrf5340_cpunet_reset.c)

  if(CONFIG_BUILD_WITH_TFM)
    zephyr_library_include_directories(
      $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
    )
  endif()
endif()
