# Source ROS
source /opt/ros/noetic/setup.bash
# Source overlay IF it exists
if [ -f ~/catkin_ws/devel/setup.bash ]; then
  source ~/catkin_ws/devel/setup.bash
fi

run_in_directory() {
  local command="$1"
  local directory="$2"

  # Store the current working directory
  local current_dir="$(pwd)"

  # Change to the specified directory
  cd "$directory" || return

  # Run the command
  eval "$command"

  # Return to the original working directory
  cd "$current_dir" || return
}

# Aliases

# Generic utils
alias refreshenv='source ~/.bashrc' # Refresh environment
alias editbashrc='nano ~/.bashrc && refreshenv' # Edit bashrc
alias cdrepo='cd $RBC_REPO' # CD to repo
alias cdws='cd $CATKIN_WS_PATH' # CD to workspace

# Catkin
alias cb='run_in_directory "catkin build" "$CATKIN_WS_PATH" && refreshenv' # Catkin Build and refresh environment

# Arduino
alias ac='run_in_directory "arduino-cli compile --fqbn arduino:avr:mega" "$RBC_REPO/controller"' # Arduino Compile
alias au='run_in_directory "arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega" "$RBC_REPO/controller"' # Arduino Upload
alias acu='ac && au' # Arduino Compile and Upload
alias abl='rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries' # Arduino Build Libraries
alias acm='cb && abl && ac' # Arduino Compile Macro (Compiles all dependencies, compiles the sketch)

# Launch files
alias rbcl='roslaunch robocock' # Robocock Launch
alias cst='rbcl controller_stress_test.launch' # Controller stress test
alias g='rbcl gazebo.launch' # Gazebo
alias m='rbcl model.launch' # Model
alias mrc='rbcl motor_ros_control.launch' # Motor ROS Control
alias mt='rbcl motor_tune.launch' # Motor Tune

# Git
alias update='run_in_directory "git pull && refreshenv" "$RBC_REPO"' # Update repo
alias discard_changes='run_in_directory "git reset --hard HEAD" "$RBC_REPO"' # Discard changes
alias commit='run_in_directory "git add . && git commit -m" "$RBC_REPO"' # Commit changes

# Check if ~/.dev has init.lock file. If it doesnt, initialize and create the file, otherwise do nothing
if [ ! -f ~/.dev/init.lock ]; then
  # Echo in cyan initializing dev environment
  echo -e "\e[36mInitializing dev environment...\e[0m"   
  run_in_directory "catkin build" "$CATKIN_WS_PATH" # Build catkin workspace
  source $CATKIN_WS_PATH/devel/setup.bash # Source catkin workspace
  abl # Build Arduino libraries
  ac # Compile Arduino sketch
  touch ~/.dev/init.lock
  echo -e "\e[36mDev environment initialized, compiled catkin_ws, arduino rosserial libraries, and arduino sketch.\e[0m"
fi

# Convenience alias for setting remote or local ROS_MASTER_URI

export DEV_MASTER_URI="http://localhost:11311"
export DEV_BOT_HOSTNAME="rbc.local"
export DEV_BOT_MASTER_URI="http://$DEV_BOT_HOSTNAME:11311"

export LOCAL_IP=$(hostname -I | awk '{print $1}')


alias setdevmaster='export ROS_MASTER_URI=$DEV_MASTER_URI && export ROS_HOSTNAME=$LOCAL_IP'
alias setbotmaster='export ROS_MASTER_URI=$DEV_BOT_MASTER_URI && export ROS_HOSTNAME=$LOCAL_IP'
alias checkmaster='echo Hostname: $ROS_HOSTNAME, Master URI: $ROS_MASTER_URI'