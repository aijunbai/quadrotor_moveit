#!/bin/bash - 
#===============================================================================
#
#          FILE: fly.sh
# 
#         USAGE: ./fly.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 07/23/2015 23:41
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

rostopic pub -v -1 /command/twist geometry_msgs/TwistStamped \
    "{header: {stamp: now, frame_id: \"map\"}, twist: {linear: {x: 0, y: 0, z: 1.0}, angular: {x: 0, y: 0, z: 0}}}"

rostopic pub -v -1 /command/twist geometry_msgs/TwistStamped \
    "{header: {stamp: now, frame_id: \"map\"}, twist: {linear: {x: 0, y: 0, z: 0.0}, angular: {x: 0, y: 0, z: 1.0}}}"


