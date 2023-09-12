CONFIG_FILE=workspace.rosinstall
ROSINSTALLFILE=$(cd $(dirname $CONFIG_FILE) && pwd -P)/$CONFIG_FILE

cd ..
if [ -d "catkin_ws" ]
then
  echo "Removing old catkin workspace..."
  rm -rf catkin_ws
fi

mkdir -p catkin_ws/src
catkin init --workspace catkin_ws
cd catkin_ws
wstool init src $ROSINSTALLFILE

# Create simbolic link of the packages in src
echo "Creating simbolic link of the packages..."
cd src
ln -s ../../controller
ln -s ../../planning
ln -s ../../waypoint_msgs
cd ..

# Build
echo "Building packages..."
catkin_make
return_value=$?
if [ ${return_value} -eq 1 ]
then
  echo -e "\033[31mBuild failed!!!\033[0m"
  exit 1
fi
echo -e "\033[32mBuild successful!!!\033[0m"