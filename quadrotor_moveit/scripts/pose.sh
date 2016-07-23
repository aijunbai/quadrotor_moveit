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

for z in 1.0 3.0 5.0; do
    for y in -4.0 -2.0 0.0 2.0 4.0; do
        for x in -4.0 -2.0 0.0 2.0 4.0; do
            rostopic pub -v -1 /command/pose geometry_msgs/PoseStamped \
                "{header: {stamp: now, frame_id: \"map\"}, pose: {position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {x: ${x}, y: ${y}, z: ${z}, w: 1.0}}}"
            sleep 10
        done
    done
done




