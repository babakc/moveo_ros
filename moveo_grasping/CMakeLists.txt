cmake_minimum_required(VERSION 2.8.3)
project(moveo_grasping)


find_package(catkin REQUIRED)


#find_package(catkin REQUIRED COMPONENTS
#  gazebo_ros
#  moveo_urdf    # required to install .rviz model
#)

#catkin_package(DEPENDS gazebo_ros)


catkin_package()

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
