#!/bin/bash

# To be run to pull
# In order to run:
#   1. Make the file an executable using command "chmod +x ~/catkin_ws/src/drone_swarm/scripts/push.sh"
#   2. Run the file using command "bash ~/catkin_ws/src/drone_swarm/scripts/push.sh"

# Navigate to the source directory
cd ~/catkin_ws/src

# Pull changes from the master branch
git push

# Optional: print a success message
echo "Workspace pushed successfully!"