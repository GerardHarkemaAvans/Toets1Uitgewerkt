cmake_minimum_required(VERSION 2.8.3)
project(casus_support)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()

# Install all the model files
install(DIRECTORY urdf/models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}meshes/models)

