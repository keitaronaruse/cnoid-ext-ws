option(BUILD_PCD_SCENE_PLUGIN "Building PCD scene plugin" OFF)
if(NOT BUILD_PCD_SCENE_PLUGIN)
  return()
endif()

choreonoid_add_plugin(CnoidPCDScenePlugin PCDScenePlugin.cpp)