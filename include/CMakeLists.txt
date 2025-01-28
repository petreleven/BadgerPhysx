add_library(LocusMathFunctions LocusMathFunctions.cpp)
add_library(Core Core.cpp)
add_library(ForceGenerator ForceGenerator.cpp)
add_library(RigidBody RigidBody.cpp)
add_library(World World.cpp)
target_link_libraries(LocusMathFunctions PUBLIC Core)
target_link_libraries(ForceGenerator PUBLIC Core) 
target_link_libraries(RigidBody PUBLIC Core) 
target_link_libraries(World PUBLIC Core) 
target_link_libraries(World PUBLIC ForceGenerator) 
target_link_libraries(World PUBLIC RigidBody) 



