cmake_minimum_required(VERSION 3.0.2)
project(vive_teleop)

find_package(catkin REQUIRED COMPONENTS

)
catkin_package(
  CATKIN_DEPENDS actionlib)

catkin_install_python(PROGRAMS scripts/gamepad_teleop.py
                               scripts/incrementer_server.py
                               scripts/vive_teleop.py
                               scripts/ee_publisher.py
                               scripts/data_recorder.py
                               scripts/vive_pose_mapping.py
                               scripts/tiago_arm_position_control.py
                               scripts/tiago_arm_gamepad_control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
