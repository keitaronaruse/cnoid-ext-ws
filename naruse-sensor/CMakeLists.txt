find_package(PCL REQUIRED common io surface features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources light-on-controller.cpp)
set(target LightOnController)
choreonoid_add_simple_controller(LightOnController light-on-controller.cpp)

#set(sources d435-save-pcd-controller.cpp)
#set(target D435SavePCDController)
#choreonoid_add_simple_controller(D435SavePCDController d435-save-pcd-controller.cpp)

# set(sources pt-d435-save-pcd-controller.cpp)
# set(target PTD435SavePCDController)
# choreonoid_add_simple_controller(PTD435SavePCDController pt-d435-save-pcd-controller.cpp)

#set(sources omni-d435-save-pcd-controller.cpp)
#set(target OmniD435SavePCDController)
#choreonoid_add_simple_controller(OmniD435SavePCDController omni-d435-save-pcd-controller.cpp)

target_link_libraries(${target} ${PCL_LIBRARIES} )
