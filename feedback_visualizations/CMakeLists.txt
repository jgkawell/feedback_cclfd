# URL , CMakeLists Tutorial: http://wiki.ros.org/catkin/CMakeLists.txt

# Your CMakeLists.txt file MUST follow this format otherwise your packages will not build correctly. The order in the configuration DOES count.

#  1. Required CMake Version (cmake_minimum_required)
cmake_minimum_required(VERSION 2.6)

#  2. Package Name (project())
project(feedback_visualizations)
#  2.1. Project-specific definitions
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#  2.2. Module path - Important?
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#  2.3. Set the appropriate compiler flags
SET( GCC_PTHREAD_FLAGS "-lpthread -pthread" ) # This is needed for LIBIGL when using 'catkin_make' , works with plain cmake/make
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PTHREAD_FLAGS}" ) # Append the flags                 ^^^-- *shrug* ---^^^



# === ROS SETUP ============================================================================================================================

#  3. Find other CMake/Catkin packages needed for build (find_package())
find_package(catkin REQUIRED cmake_modules) # DO THIS FIRST # 2018-05-27

#~ find_package(Eigen REQUIRED) # 2018-05-27

find_package(catkin REQUIRED COMPONENTS
  # ~ ROS Packages ~
  roslib
  roscpp #- C++
  rospy # - Python!
  tf2 # --- tf is deprecated
  tf2_ros # This is separate from vanilla tf2!!
  # ~ ROS Messages ~
  message_generation # This should be first?
  std_msgs # --------- Primitive messages
  visualization_msgs # RViz
  geometry_msgs # ---- Vectors, Quaternions, Poses
  sensor_msgs 
  # ~ Local Packages ~
  # YOUR PKGs HERE
)

#  4. Enable Python module support (catkin_python_setup())
#~ catkin_python_setup() # Use only if there are python packages to *install*, Not necessary for scripts

#  5. Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())

#  6. Invoke message/service/action generation (generate_messages())
generate_messages(DEPENDENCIES 
	std_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
)

#  7. Specify package build info export (catkin_package())
catkin_package(CATKIN_DEPENDS 
	# message_generation
	message_runtime 
	std_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS cmake_modules roscpp std_msgs visualization_msgs roslib
)

#  8. Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
        # include/
        ${catkin_INCLUDE_DIRS}
        #~ ${Eigen_INCLUDE_DIRS} # 2018-05-27
        #~ ${Eigen3_INCLUDE_DIRS} # NOTE: Eigen3 currently not in use for this project
)

#  9. Tests to build (catkin_add_gtest())

# 10. Install rules (install()) 
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ___ END ROS ______________________________________________________________________________________________________________________________


# === PROJECT SPECIFIC =====================================================================================================================

#~ include_directories( cpp_helpers ) # Helper Library
#~ include_directories( src ) # ------- Project specific sources

#~ set( CPP_HELP_CPP cpp_helpers/Cpp_Helpers.cpp ) # Shortcut for the "Cpp_Helpers" source files
#~ set( ROS_HELP_CPP cpp_helpers/ROS_Helpers.cpp ) # Shortcut for the "ROS_Helpers" source files

# ___ END SPECIFIC _________________________________________________________________________________________________________________________


# === PROJECT EXECUTABLES ==================================================================================================================

# ~~~ <EXEC NAME HERE> ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#~ add_executable( execName 
			    #~ <Paths_to_All_C_and_CPP_SOURCES>
			    #~ ${PKG_SOURCE_VARS} )

#~ target_link_libraries( execName
					   #~ ${PKG_LIB_VARS}
					   #~ ${catkin_LIBRARIES} )

# ___ END EXECUTABLES _________________________________________________________________________________________________________________________