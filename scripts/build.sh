cd ..
cd catkin_ws
rm -rf build devel
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