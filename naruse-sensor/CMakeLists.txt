find_package(PCL REQUIRED common io surface features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources omni-d435-save-pcd-controller.cpp)
set(target OmniD435SavePCDController)
choreonoid_add_simple_controller(OmniD435SavePCDController omni-d435-save-pcd-controller.cpp)

#set(sources d435-save-pcd-controller.cpp)
#set(target D435SavePCDController)
#choreonoid_add_simple_controller(D435SavePCDController d435-save-pcd-controller.cpp)

#set(sources naruse-d435-save.cpp)
#set(target NaruseD435SaveController)
#choreonoid_add_simple_controller(NaruseD435SaveController naruse-d435-save.cpp)

#set(sources naruse-pt-d435-save.cpp)
#set(target NarusePTD435SaveController)
#choreonoid_add_simple_controller(NarusePTD435SaveController naruse-pt-d435-save.cpp)

#target_link_libraries(${target} ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} pcl_io)
target_link_libraries(${target} ${PCL_LIBRARIES} )
