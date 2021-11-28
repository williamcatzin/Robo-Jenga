# Filename: .bashrc
# Description: Sources in on the class MASTER version for settings information
# 
# Please (DO NOT) edit this file unless you are sure of what you are doing.
# This file and other dotfiles have been written to work with each other.
# Any change that you are not sure off can break things in an unpredicatable
# ways.

# Set the Class MASTER variable and source the class master version of .cshrc

[[ -z ${MASTER} ]] && export MASTER=${LOGNAME%-*}
[[ -z ${MASTERDIR} ]] && export MASTERDIR=$(eval echo ~${MASTER})

# Set up class wide settings
for file in ${MASTERDIR}/adm/bashrc.d/* ; do [[ -x ${file} ]] && . "${file}"; done

# Set up local settings
for file in ${HOME}/bashrc.d/* ; do [[ -x ${file} ]] && . "${file}"; done



# If you are working with the Baxter or Sawyer packages, uncomment the line below
# to source the .bashrc file  that sets up ROS with the Baxter and Sawyer packages:
source /scratch/shared/baxter_ws/devel/setup.bash

# Otherwise, uncomment the line below to source the .bashrc file
# that sets up ROS without the Baxter and Sawyer packages:
# source /opt/ros/kinetic/setup.bash

##############################
# Configuring the IP Address #
##############################

# Run the following command on a terminal:
# cat /etc/hosts
# to see the list of IP addresses for the workstations and robots.

# If you are working on a workstation in the lab, uncomment the line
# below to automatically set the ROS hostname as the current workstation IP address:
export ROS_HOSTNAME=$(hostname --short).local

# Uncomment the line below with the correct IP address for the robot that you will be using.
# For example, export ROS_MASTER_URI=http://192.168.1.111:11311 to work with Black Turtlebot.
export ROS_MASTER_URI=http://ayrton.local:11311
