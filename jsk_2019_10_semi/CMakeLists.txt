cmake_minimum_required(VERSION 2.8.3)
project(jsk_2019_10_semi)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

