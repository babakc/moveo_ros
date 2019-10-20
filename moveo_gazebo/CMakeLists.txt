cmake_minimum_required(VERSION 2.8.3)
project(moveo_gazebo)


find_package(catkin REQUIRED)


#find_package(catkin REQUIRED COMPONENTS
#  gazebo_ros
#  moveo_urdf    # required to install .rviz model
#)

#catkin_package(DEPENDS gazebo_ros)


catkin_package()

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION $ENV{HOME}/.gazebo/models
)

