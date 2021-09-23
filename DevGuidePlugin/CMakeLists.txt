option(BUILD_DEVGUIDE_PLUGIN "Building a sample plugin of the plugin development guide" OFF)
if(NOT BUILD_DEVGUIDE_PLUGIN)
  return()
endif()

choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)
target_link_libraries(CnoidDevGuidePlugin CnoidBodyPlugin)
