cmake_minimum_required(VERSION 2.8.3)
project(bsv_effectors)

find_package(catkin REQUIRED
  rospy
  bsv_msg)

catkin_package(CATKIN_DEPENDS xacro)

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

