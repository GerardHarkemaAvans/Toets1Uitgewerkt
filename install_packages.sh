#!/bin/bash

echo "Installing ROS Tools"
sudo apt update
sudo apt install 	python-catkin-tools \
		ros-melodic-moveit \
		joint-state-publisher \
		ros-melodic-joint-state-publisher-gui \
		ros-melodic-ros-control \
		ros-melodic-ros-controllers \
		gazebo9-plugin-base \
		libsdformat6 \
		libsdl-image1.2-dev \
		libnlopt-dev \
		ros-melodic-flexbe-behavior-engine \
		ros-melodic-nodelet \


DOWNLOAD_DIRECTORY=../downloads
PACKAGE_DIRECTORY=../src/rospackages

#flexbe app
echo "Installing Flexbe"
DIRECTORY=$PACKAGE_DIRECTORY/flexbe_app
if [ ! -d  "$DIRECTORY" ]; then
	git clone https://github.com/FlexBE/flexbe_app.git $PACKAGE_DIRECTORY/flexbe_app
fi

#mkdir Downloads


# Universal Robots
echo "Installing Universal Robot"
UNIVERSAL_ROBOT_FILE=$DOWNLOAD_DIRECTORY/universal_robots/melodic-devel.zip
UNIVERSAL_ROBOT_PACKAGES="ur_bringup ur_description ur_gazebo"
if [ ! -f "$UNIVERSAL_ROBOT_FILE" ]; then
    wget https://github.com/ros-industrial/universal_robot/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/universal_robots

fi
for val in $UNIVERSAL_ROBOT_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/universal_robot-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/universal_robots/melodic-devel.zip universal_robot-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done

# ecl core
echo "Installing ecl Core"
ECL_CORE_FILE=$DOWNLOAD_DIRECTORY/ecl_core/0.62-melodic.zip
ECL_CORE_PACKAGES="ecl_concepts ecl_eigen ecl_exceptions ecl_mpl ecl_threads ecl_time ecl_type_traits ecl_utilities"
if [ ! -f "$ECL_CORE_FILE" ]; then
    wget https://github.com/stonier/ecl_core/archive/release/0.62-melodic.zip -P $DOWNLOAD_DIRECTORY/ecl_core
fi
for val in $ECL_CORE_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/ecl_core-release-0.62-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/ecl_core/0.62-melodic.zip ecl_core-release-0.62-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing ecl Tools"
ECL_TOOLS_FILE=$DOWNLOAD_DIRECTORY/ecl_tools/0.61-melodic.zip
ECL_TOOLS_PACKAGES="ecl_build ecl_license"
if [ ! -f "$ECL_TOOLS_FILE" ]; then
    wget https://github.com/stonier/ecl_tools/archive/release/0.61-melodic.zip -P $DOWNLOAD_DIRECTORY/ecl_tools
fi
for val in $ECL_TOOLS_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/ecl_tools-release-0.61-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/ecl_tools/0.61-melodic.zip ecl_tools-release-0.61-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing ecl Lite"
ECL_LITE_FILE=$DOWNLOAD_DIRECTORY/ecl_lite/0.61-melodic.zip
ECL_LITE_PACKAGES="ecl_config ecl_errors ecl_time_lite"
if [ ! -f "$ECL_LITE_FILE" ]; then
    wget https://github.com/stonier/ecl_lite/archive/release/0.61-melodic.zip -P $DOWNLOAD_DIRECTORY/ecl_lite
fi
for val in $ECL_LITE_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/ecl_lite-release-0.61-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/ecl_lite/0.61-melodic.zip ecl_lite-release-0.61-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing turtlebot"
TURTLEBOT_FILE=$DOWNLOAD_DIRECTORY/turtlebot/melodic.zip
TURTLEBOT_PACKAGES="turtlebot_bringup turtlebot_description turtlebot_teleop"
if [ ! -f "$TURTLEBOT_FILE" ]; then
    wget https://github.com/turtlebot/turtlebot/archive/melodic.zip -P $DOWNLOAD_DIRECTORY/turtlebot
fi
for val in $TURTLEBOT_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/turtlebot-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/turtlebot/melodic.zip turtlebot-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing turtlebot gazebo"
TURTLEBOT_GAZEBO_FILE=$DOWNLOAD_DIRECTORY/turtlebot-gazebo/melodic.zip
TURTLEBOT_GAZEBO_PACKAGES="turtlebot_gazebo"
if [ ! -f "$TURTLEBOT_GAZEBO_FILE" ]; then
    wget https://github.com/turtlebot/turtlebot_simulator/archive/melodic.zip -P $DOWNLOAD_DIRECTORY/turtlebot-gazebo
fi
for val in $TURTLEBOT_GAZEBO_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/turtlebot_simulator-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/turtlebot-gazebo/melodic.zip turtlebot_simulator-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing turtlebot rviz launchers"
TURTLEBOT_RVIZ_FILE=$DOWNLOAD_DIRECTORY/turtlebot-rviz/indigo.zip
TURTLEBOT_RVIZ_PACKAGES="turtlebot_rviz_launchers"
if [ ! -f "$TURTLEBOT_RVIZ_FILE" ]; then
    wget https://github.com/turtlebot/turtlebot_viz/archive/indigo.zip -P $DOWNLOAD_DIRECTORY/turtlebot-rviz
fi
for val in $TURTLEBOT_RVIZ_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/turtlebot_viz-indigo/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/turtlebot-rviz/indigo.zip turtlebot_viz-indigo/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing turtlebot apps"
TURTLEBOT_APPS_FILE=$DOWNLOAD_DIRECTORY/turtlebot-apps/indigo.zip
TURTLEBOT_APPS_PACKAGES="turtlebot_navigation"
if [ ! -f "$TURTLEBOT_APPS_FILE" ]; then
    wget https://github.com/turtlebot/turtlebot_apps/archive/indigo.zip -P $DOWNLOAD_DIRECTORY/turtlebot-apps
fi
for val in $TURTLEBOT_APPS_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/turtlebot_apps-indigo/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/turtlebot-apps/indigo.zip turtlebot_apps-indigo/$val/* -d $PACKAGE_DIRECTORY
	fi
done

echo "Installing joy"
JOYSTICK_FILE=$DOWNLOAD_DIRECTORY/joystick_drivers/melodic-devel.zip
JOYSTICK_PACKAGES="joy"
if [ ! -f "$JOYSTICK_FILE" ]; then
    wget https://github.com/ros-drivers/joystick_drivers/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/joystick_drivers
fi
for val in $JOYSTICK_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/joystick_drivers-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $DOWNLOAD_DIRECTORY/joystick_drivers/melodic-devel.zip joystick_drivers-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing yocs"
YOCS_FILE=$DOWNLOAD_DIRECTORY/yocs/0.8-melodic.zip
YOCS_PACKAGES="yocs_cmd_vel_mux yocs_controllers yocs_velocity_smoother yocs_msgs"
if [ ! -f "$YOCS_FILE" ]; then
    wget https://github.com/yujinrobot/yujin_ocs/archive/release/0.8-melodic.zip -P $DOWNLOAD_DIRECTORY/yocs
fi
for val in $YOCS_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/yujin_ocs-release-0.8-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $YOCS_FILE yujin_ocs-release-0.8-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing yocs msgs"
YOCS_MSGS_FILE=$DOWNLOAD_DIRECTORY/yocs_msgs/0.7-melodic.zip
if [ ! -f "$YOCS_MSGS_FILE" ]; then
    wget https://github.com/yujinrobot/yocs_msgs/archive/release/0.7-melodic.zip -P $DOWNLOAD_DIRECTORY/yocs_msgs
fi
DIRECTORY=$PACKAGE_DIRECTORY/yocs_msgs-release-0.7-melodic
if [ ! -d  "$DIRECTORY" ]; then
	unzip $YOCS_MSGS_FILE yocs_msgs-release-0.7-melodic/* -d $PACKAGE_DIRECTORY
fi



echo "Installing kobuki"
KOBUKI_FILE=$DOWNLOAD_DIRECTORY/kobuki/melodic.zip
KOBUKI_PACKAGES="kobuki_bumper2pc kobuki_description kobuki_safety_controller"
if [ ! -f "$KOBUKI_FILE" ]; then
    wget https://github.com/yujinrobot/kobuki/archive/melodic.zip -P $DOWNLOAD_DIRECTORY/kobuki
fi
for val in $KOBUKI_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/kobuki-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $KOBUKI_FILE kobuki-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing kobuki desktop"
KOBUKI_FILE=$DOWNLOAD_DIRECTORY/kobuki-desktop/melodic.zip
KOBUKI_PACKAGES="kobuki_gazebo_plugins"
if [ ! -f "$KOBUKI_FILE" ]; then
    wget https://github.com/yujinrobot/kobuki_desktop/archive/melodic.zip -P $DOWNLOAD_DIRECTORY/kobuki-desktop
fi
for val in $KOBUKI_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/kobuki_desktop-melodic/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $KOBUKI_FILE kobuki_desktop-melodic/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing kobuki msgs"
KOBUKI_MSGS_FILE=$DOWNLOAD_DIRECTORY/kobuki_msgs/melodic.zip
#KOBUKI_MSGS_PACKAGES="kobuki_msgs"
if [ ! -f "$KOBUKI_MSGS_FILE" ]; then
    wget https://github.com/yujinrobot/kobuki_msgs/archive/melodic.zip -P $DOWNLOAD_DIRECTORY/kobuki_msgs
fi
DIRECTORY=$PACKAGE_DIRECTORY/kobuki_msgs-melodic
if [ ! -d  "$DIRECTORY" ]; then
	unzip $KOBUKI_MSGS_FILE kobuki_msgs-melodic/* -d $PACKAGE_DIRECTORY
fi


echo "Installing navigation"
NAVIGATION_FILE=$DOWNLOAD_DIRECTORY/navigation/melodic-devel.zip
NAVIGATION_PACKAGES="move_base fake_localization voxel_grid rotate_recovery navfn nav_core map_server dwa_local_planner costmap_2d clear_costmap_recovery base_local_planner amcl"
if [ ! -f "$NAVIGATION_FILE" ]; then
    wget https://github.com/ros-planning/navigation/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/navigation
fi
for val in $NAVIGATION_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/navigation-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $NAVIGATION_FILE navigation-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing navigation msgs"
NAVIGATION_MSGS_FILE=$DOWNLOAD_DIRECTORY/navigation_msgs/ros1.zip
NAVIGATION_MSGS_PACKAGES="move_base_msgs"
if [ ! -f "$NAVIGATION_MSGS_FILE" ]; then
    wget https://github.com/ros-planning/navigation_msgs/archive/ros1.zip -P $DOWNLOAD_DIRECTORY/navigation_msgs
fi
for val in $NAVIGATION_MSGS_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/navigation_msgs-ros1/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $NAVIGATION_MSGS_FILE navigation_msgs-ros1/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing tf2"
NAVIGATION_FILE=$DOWNLOAD_DIRECTORY/tf2/melodic-devel.zip
NAVIGATION_PACKAGES="tf2_sensor_msgs"
if [ ! -f "$NAVIGATION_FILE" ]; then
    wget https://github.com/ros/geometry2/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/tf2
fi
for val in $NAVIGATION_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/geometry2-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $NAVIGATION_FILE geometry2-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing depth image to laserscan"
IM_TO_LASER_FILE=$DOWNLOAD_DIRECTORY/depthimage_to_laserscan/melodic-devel.zip
if [ ! -f "$IM_TO_LASER_FILE" ]; then
    wget https://github.com/ros-perception/depthimage_to_laserscan/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/depthimage_to_laserscan
fi
DIRECTORY=$PACKAGE_DIRECTORY/depthimage_to_laserscan-melodic-devel
if [ ! -d  "$DIRECTORY" ]; then
	unzip $IM_TO_LASER_FILE depthimage_to_laserscan-melodic-devel/* -d $PACKAGE_DIRECTORY
fi

echo "Installing trac_ik"
TRAC_IK_FILE=$DOWNLOAD_DIRECTORY/trac_ik/eb11e334494a.zip
TRAC_IK_PACKAGES="trac_ik_kinematics_plugin  trac_ik_lib trac_ik"
if [ ! -f "$TRAC_IK_FILE" ]; then
    wget https://bitbucket.org/traclabs/trac_ik/get/eb11e334494a.zip -P $DOWNLOAD_DIRECTORY/trac_ik
fi
for val in $TRAC_IK_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/traclabs-trac_ik-eb11e334494a/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $TRAC_IK_FILE traclabs-trac_ik-eb11e334494a/$val/* -d $PACKAGE_DIRECTORY
	fi
done

echo "Installing moveit python"
MOVEIT_PYTHON_FILE=$DOWNLOAD_DIRECTORY/moveit_python/master.zip
if [ ! -f "$MOVEIT_PYTHON_FILE" ]; then
    wget https://github.com/mikeferguson/moveit_python/archive/master.zip -P $DOWNLOAD_DIRECTORY/moveit_python
fi
DIRECTORY=$PACKAGE_DIRECTORY/moveit_python-master
if [ ! -d  "$DIRECTORY" ]; then
	unzip $MOVEIT_PYTHON_FILE moveit_python-master/* -d $PACKAGE_DIRECTORY
fi


echo "Installing slam gmapping"
SLAM_GMAPPING_FILE=$DOWNLOAD_DIRECTORY/slam_gmapping/melodic-devel.zip
SLAM_GMAPPING_PACKAGES="gmapping"
if [ ! -f "$SLAM_GMAPPING_FILE" ]; then
    wget https://github.com/ros-perception/slam_gmapping/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/slam_gmapping
fi
for val in $SLAM_GMAPPING_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/slam_gmapping-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $SLAM_GMAPPING_FILE slam_gmapping-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done

echo "Installing openslam gmapping"
MOVEIT_PYTHON_FILE=$DOWNLOAD_DIRECTORY/openslam-gmapping/melodic-devel.zip
if [ ! -f "$MOVEIT_PYTHON_FILE" ]; then
    wget https://github.com/ros-perception/openslam_gmapping/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/openslam-gmapping
fi
DIRECTORY=$PACKAGE_DIRECTORY/openslam_gmapping-melodic-devel
if [ ! -d  "$DIRECTORY" ]; then
	unzip $MOVEIT_PYTHON_FILE openslam_gmapping-melodic-devel/* -d $PACKAGE_DIRECTORY
fi


echo "Installing PR2"
PR2_FILE=$DOWNLOAD_DIRECTORY/pr2/melodic-devel.zip
PR2_PACKAGES="pr2_description"
if [ ! -f "$PR2_FILE" ]; then
    wget https://github.com/PR2/pr2_common/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/pr2
fi
for val in $PR2_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/pr2_common-melodic-devel/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $PR2_FILE pr2_common-melodic-devel/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing robot pose efk"
ROBOT_POSE_EFK_FILE=$DOWNLOAD_DIRECTORY/robot_pose_efk/master.zip
if [ ! -f "$ROBOT_POSE_EFK_FILE" ]; then
    wget https://github.com/ros-planning/robot_pose_ekf/archive/master.zip -P $DOWNLOAD_DIRECTORY/robot_pose_efk
fi
DIRECTORY=$PACKAGE_DIRECTORY/robot_pose_ekf-master
if [ ! -d  "$DIRECTORY" ]; then
	unzip $ROBOT_POSE_EFK_FILE robot_pose_ekf-master/* -d $PACKAGE_DIRECTORY
fi


echo "Installing ROS bridge"
ROS_BRIDGE_FILE=$DOWNLOAD_DIRECTORY/rosbridge/develop.zip
ROS_BRIDGE_PACKAGES="rosapi rosbridge_library rosbridge_msgs rosbridge_server"
if [ ! -f "$ROS_BRIDGE_FILE" ]; then
    wget https://github.com/RobotWebTools/rosbridge_suite/archive/develop.zip -P $DOWNLOAD_DIRECTORY/rosbridge
fi
for val in $ROS_BRIDGE_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/rosbridge_suite-develop/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $ROS_BRIDGE_FILE rosbridge_suite-develop/$val/* -d $PACKAGE_DIRECTORY
	fi
done


echo "Installing telop twist keyboard"
TELOP_TWIST_KEYBOARD_FILE=$DOWNLOAD_DIRECTORY/telop_twist_keyboard/master.zip
if [ ! -f "$TELOP_TWIST_KEYBOARD_FILE" ]; then
    wget https://github.com/ros-teleop/teleop_twist_keyboard/archive/master.zip -P $DOWNLOAD_DIRECTORY/telop_twist_keyboard
fi
DIRECTORY=$PACKAGE_DIRECTORY/teleop_twist_keyboard-master
if [ ! -d  "$DIRECTORY" ]; then
	unzip $TELOP_TWIST_KEYBOARD_FILE teleop_twist_keyboard-master/* -d $PACKAGE_DIRECTORY
fi

echo "Installing convex decompoistion"
CONFEX_DECOMP_FILE=$DOWNLOAD_DIRECTORY/convex_decomposition/melodic-devel.zip
if [ ! -f "$CONFEX_DECOMP_FILE" ]; then
    wget https://github.com/ros/convex_decomposition/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/convex_decomposition
fi
DIRECTORY=$PACKAGE_DIRECTORY/convex_decomposition-melodic-devel
if [ ! -d  "$DIRECTORY" ]; then
	unzip $CONFEX_DECOMP_FILE convex_decomposition-melodic-devel/* -d $PACKAGE_DIRECTORY
fi

echo "Installing rosauth"
ROSAUTH_FILE=$DOWNLOAD_DIRECTORY/rosauth/develop.zip
if [ ! -f "$ROSAUTH_FILE" ]; then
    wget https://github.com/GT-RAIL/rosauth/archive/develop.zip -P $DOWNLOAD_DIRECTORY/rosauth
fi
DIRECTORY=$PACKAGE_DIRECTORY/rosauth-develop
if [ ! -d  "$DIRECTORY" ]; then
	unzip $ROSAUTH_FILE rosauth-develop/* -d $PACKAGE_DIRECTORY
fi


echo "Installing bfl"
BFL_FILE=$DOWNLOAD_DIRECTORY/bfl/bfl.zip
if [ ! -f "$BFL_FILE" ]; then
    wget https://github.com/ros-gbp/bfl-release/archive/rpm/melodic/bfl.zip -P $DOWNLOAD_DIRECTORY/bfl
fi
DIRECTORY=$PACKAGE_DIRECTORY/bfl-release-rpm-melodic-bfl
if [ ! -d  "$DIRECTORY" ]; then
	unzip $BFL_FILE bfl-release-rpm-melodic-bfl/* -d $PACKAGE_DIRECTORY
fi


echo "Installing ivcon"
IVCON_FILE=$DOWNLOAD_DIRECTORY/ivcon/melodic-devel.zip
if [ ! -f "$IVCON_FILE" ]; then
    wget https://github.com/ros/ivcon/archive/melodic-devel.zip -P $DOWNLOAD_DIRECTORY/ivcon
fi
DIRECTORY=$PACKAGE_DIRECTORY/ivcon-melodic-devel
if [ ! -d  "$DIRECTORY" ]; then
	unzip $IVCON_FILE ivcon-melodic-devel/* -d $PACKAGE_DIRECTORY
fi


echo "Installing turtlebot create"
TURTLEBOT_CREATE_FILE=$DOWNLOAD_DIRECTORY/turtlebot_create/master.zip
TURTLEBOT_CREATE_PACKAGES="create_description"
if [ ! -f "$TURTLEBOT_CREATE_FILE" ]; then
    wget https://github.com/turtlebot/turtlebot_create/archive/master.zip -P $DOWNLOAD_DIRECTORY/turtlebot_create
fi
for val in $TURTLEBOT_CREATE_PACKAGES; do
	DIRECTORY=$PACKAGE_DIRECTORY/turtlebot_create-master/$val
	if [ ! -d  "$DIRECTORY" ]; then
		unzip $TURTLEBOT_CREATE_FILE turtlebot_create-master/$val/* -d $PACKAGE_DIRECTORY
	fi
done




#rm -r $DOWNLOAD_DIRECTORY
echo "Done installing"



