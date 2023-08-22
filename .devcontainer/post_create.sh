source ~/.bashrc
source ~/convenience.sh

echo -e "\e[36mInitializing dev environment...\e[0m"   
run_in_directory "catkin build" "$CATKIN_WS_PATH" # Build catkin workspace
source $CATKIN_WS_PATH/devel/setup.bash # Source catkin workspace
abl # Build Arduino libraries
ac # Compile Arduino sketch
echo -e "\e[36mDev environment initialized, compiled catkin_ws, arduino rosserial libraries, and arduino sketch.\e[0m"