cmake_minimum_required(VERSION 2.8.3)
project(karto_scan_matcher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(PkgConfig REQUIRED)
# Use Thread-Building-Blocks [OpenKarto]
pkg_check_modules(TBB tbb)
if(TBB_FOUND)
      add_definitions(-DUSE_TBB)
  endif(TBB_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES karto_scan_matcher
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(karto_scan_matcher
    src/Any.cpp              
    src/RigidBodyTransform.cpp
    src/Event.cpp            
    src/Exception.cpp        
    src/Geometry.cpp         
    src/Grid.cpp             
    src/GridIndexLookup.cpp  
    src/Identifier.cpp        
    src/Logger.cpp            
    src/Meta.cpp              
    src/MetaAttribute.cpp     
    src/MetaClass.cpp         
    src/MetaClassManager.cpp  
    src/MetaEnum.cpp         
    src/MetaEnumHelper.cpp   
    src/MetaEnumManager.cpp  
    src/Module.cpp           
    src/Mutex.cpp            
    src/Object.cpp           
    src/Objects.cpp        
    src/OccupancyGrid.cpp  
    src/OpenMapper.cpp     
    src/Parameter.cpp      
    src/PoseTransform.cpp  
    src/Referenced.cpp     
    src/Sensor.cpp
    src/SensorData.cpp
    src/SensorRegistry.cpp
    src/String.cpp
    src/StringHelper.cpp
)

if(TBB_FOUND)
      target_link_libraries(karto_scan_matcher ${TBB_LIBRARIES})
endif(TBB_FOUND)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(karto_scan_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(karto_scan_matcher_node src/karto_scan_matcher_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(karto_scan_matcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(karto_scan_matcher_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS karto_scan_matcher 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_karto_scan_matcher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
