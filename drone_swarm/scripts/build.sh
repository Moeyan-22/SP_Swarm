#!/bin/bash

# To be run to build after pulling
# In order to run:
#   1. Make the file an executable using command "chmod +x ~/catkin_ws/src/drone_swarm/scripts/build.sh"
#   2. Run the file using command "bash ~/catkin_ws/src/drone_swarm/scripts/build.sh"

# Build the workspace
cd ~/catkin_ws/
catkin build

# Source the setup script
source devel/setup.bash

# Optional: print a success message
echo "Workspace built successfully!"