cmake_minimum_required(VERSION 2.8.3)
project(darkroom)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rviz tf tf_conversions visualization_msgs
		roboy_communication_middleware common_utilities )

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)

# protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS protobuf/lighthouse.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities )

include_directories(
		include/
        mavmap/src
        ${catkin_INCLUDE_DIRS}
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ncurses_INCLUDE_DIRS}
		../common_utilities/include
)
#==============================================================================
# Application
#==============================================================================

add_library( UDPSocket src/UDPSocket.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries( UDPSocket ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} )

add_library( PoseMinimizer src/PoseMinimizer.cpp src/LighthousePoseMinimizer.cpp src/LighthousePoseMinimizer2.cpp)
target_link_libraries( PoseMinimizer ${catkin_LIBRARIES} )

add_library( Sensor src/Sensor.cpp )
target_link_libraries( Sensor ${catkin_LIBRARIES} )

add_library( TrackedObject src/TrackedObject.cpp mavmap/src/base3d/triangulation.cc mavmap/src/base3d/projection.cc
		epnp/epnp.cpp mavmap/src/base3d/p3p.cc mavmap/src/util/math.cc src/ParticleFilter.cpp)
add_dependencies(TrackedObject roboy_communication_simulation_gencpp)
target_link_libraries( TrackedObject ${yaml-cpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} UDPSocket PoseMinimizer Sensor )

qt5_wrap_cpp(MOC_FILES include/darkroom/DarkRoom.hpp include/darkroom/TrackedObjectDelegate.hpp)
add_library( darkroom src/DarkRoom.cpp src/TrackedObjectDelegate.cpp ${MOC_FILES} src/Triangulation.cpp src/ParticleFilter.cpp)
target_link_libraries( darkroom ${QT_LIBRARIES} ${catkin_LIBRARIES} TrackedObject rviz_visualization)

add_executable( CommunicationTest src/CommunicationTest.cpp)
target_link_libraries( CommunicationTest ${catkin_LIBRARIES} UDPSocket )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        darkroom
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS
		CommunicationTest
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
