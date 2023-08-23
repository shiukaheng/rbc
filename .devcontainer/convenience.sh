#!/bin/bash

# Source ROS
source /opt/ros/noetic/setup.bash
# Source overlay IF it exists
if [ -f ~/catkin_ws/devel/setup.bash ]; then
  source ~/catkin_ws/devel/setup.bash
fi

pexport() {
    # Check for the correct number of arguments
    if [[ $# -ne 2 ]]; then
        echo "Usage: pexport VARNAME VALUE"
        return 1
    fi

    local var_name="$1"
    local value="$2"
    local bashrc="$HOME/.dev/.bashrc_vars"
    local comment="# pexport generated"

    # If bashrc does not exist, create it
    if [ ! -f "$bashrc" ]; then
        touch "$bashrc"
    fi

    # Remove the previous pexported line if it exists
    grep -v "export $var_name=" "$bashrc" | grep -v "$var_name=.*$comment" > "${bashrc}.tmp"
    mv "${bashrc}.tmp" "$bashrc"

    # Add the new pexport line
    echo "export $var_name=\"$value\" $comment" >> "$bashrc"

    echo "Variable $var_name has been set to \"$value\" and pexported."
    source "$bashrc"
}

punset() {
    # Check for the correct number of arguments
    if [[ $# -ne 1 ]]; then
        echo "Usage: punset VARNAME"
        return 1
    fi

    local var_name="$1"
    local bashrc="$HOME/.dev/.bashrc_vars"
    local comment="# pexport generated"

    # If bashrc does not exist, create it
    if [ ! -f "$bashrc" ]; then
        touch "$bashrc"
    fi

    # Remove the pexported line
    grep -v "export $var_name=" "$bashrc" | grep -v "$var_name=.*$comment" > "${bashrc}.tmp"
    mv "${bashrc}.tmp" "$bashrc"

    echo "Variable $var_name has been unset and removed from pexported variables."
    source "$bashrc"
}

# run_in_directory() {
#   local command_to_run="$1"
#   local directory="$2"
#   shift 2
#   (
#     cd "$directory"
#     "$command_to_run" "$@"
#   )
# }

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

check_var_not_empty() { # Takes in a variable name, value, command to run if not empty, and command to run if empty
  local var_name="$1"
  local var_value="$2"
  local not_empty_command="$3"
  local empty_command="$4"

  if [ -z "$var_value" ]; then
    echoColor yellow "Warning: Variable $var_name is empty"
    if [ -n "$empty_command" ]; then
    eval "$empty_command"
    fi
  else
    if [ -n "$not_empty_command" ]; then
    eval "$not_empty_command"
    fi
  fi
}

# Aliases

# Generic utils
alias refreshenv='source ~/.bashrc' # Refresh environment
alias editbashrc='nano ~/.bashrc && refreshenv' # Edit bashrc
alias cdrepo='cd $RBC_REPO' # CD to repo
alias cdr='cdrepo'
alias cdws='cd $CATKIN_WS_PATH' # CD to workspace
alias cdw='cdws'
# Catkin
alias cb='run_in_directory catkin "$CATKIN_WS_PATH" build && refreshenv'
alias ccp='run_in_directory "catkin_create_pkg" "$CATKIN_WS_PATH/src"' # Catkin Create Package

# Patch files with string replacement
function patch() {
  local file="$1"
  local search="$2"
  local replace="$3"
  sed -i "s/$search/$replace/g" "$file"
}

# Arduino
alias ac='run_in_directory "arduino-cli compile --fqbn arduino:avr:mega" "~/rbc/controller"' # Arduino Compile
alias au='run_in_directory "arduino-cli upload -p $ARDUINO_PORT --fqbn arduino:avr:mega" "$RBC_REPO/controller"' # Arduino Upload
alias acu='ac && au' # Arduino Compile and Upload
alias patch_rosserial_arduino_port='patch "~/Arduino/libraries/ros_lib/ArduinoHardware.h" "iostream = &Serial;" "iostream = \\&Serial3;"' # Patch rosserial_arduino port
alias abl='rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries && patch_rosserial_arduino_port' # Arduino Build Libraries
alias acm='cb && abl && ac' # Arduino Compile Macro (Compiles all dependencies, compiles the sketch)

# Launch files

# Git
alias update='run_in_directory "git pull && refreshenv" "$RBC_REPO"' # Update repo
alias discard_changes='run_in_directory "git reset --hard HEAD" "$RBC_REPO"' # Discard changes
alias commit='run_in_directory "git add . && git commit -m" "$RBC_REPO"' # Commit changes

# Convenience alias for setting remote or local ROS_MASTER_URI

alias setdevmaster="pexport ROS_MASTER_URI $DEV_MASTER_URI && pexport ROS_HOSTNAME \$(hostname).local && pexport RBC_MASTER 'local'"
alias sdm='setdevmaster'
alias setbotmaster="pexport ROS_MASTER_URI $DEV_BOT_MASTER_URI && pexport ROS_HOSTNAME \$(hostname).local && pexport RBC_MASTER 'bot'"
alias sbm='setbotmaster'
alias checkmaster='echo Hostname: $ROS_HOSTNAME, Master URI: $ROS_MASTER_URI'
alias cm='checkmaster'

# Usage: echoColor <color> <message>
# Colors: black, red, green, yellow, blue, magenta, cyan, white
function echoColor() {
    local color=$1
    local message=$2
    local colorCode

    case $color in
        black)
            colorCode=0
            ;;
        red)
            colorCode=1
            ;;
        green)
            colorCode=2
            ;;
        yellow)
            colorCode=3
            ;;
        blue)
            colorCode=4
            ;;
        magenta)
            colorCode=5
            ;;
        cyan)
            colorCode=6
            ;;
        white)
            colorCode=7
            ;;
        bright-black)
            colorCode=8
            ;;
        bright-red)
            colorCode=9
            ;;
        bright-green)
            colorCode=10
            ;;
        bright-yellow)
            colorCode=11
            ;;
        bright-blue)
            colorCode=12
            ;;
        bright-magenta)
            colorCode=13
            ;;
        bright-cyan)
            colorCode=14
            ;;
        bright-white)
            colorCode=15
            ;;
        *)
            echo "Invalid color: $color"
            return 1
            ;;
    esac
    echo -e "\033[38;5;${colorCode}m${message}\033[0m"
}

# Convenience function for piping to echoColor
# Usage: <command> | echoColorPipe <color>
function echoColorPipe() {
    local color=$1
    local message
    while read message; do
        echoColor $color "$message"
    done
}

# Convenience function for adding style to text
# Usage: echoStyle <style> <message>
# Styles: bold, italic, underline, blink, inverse, hidden, strikethrough
function echoStyle() {
    local style=$1
    local message=$2
    local styleCode
    case $style in
        bold)
            styleCode=1
            ;;
        italic)
            styleCode=3
            ;;
        underline)
            styleCode=4
            ;;
        blink)
            styleCode=5
            ;;
        inverse)
            styleCode=7
            ;;
        hidden)
            styleCode=8
            ;;
        strikethrough)
            styleCode=9
            ;;
        *)
            echo "Invalid style: $style"
            return 1
            ;;
    esac
    echo -e "\033[${styleCode}m${message}\033[0m"
}

# Convenience function for piping to echoStyle
# Usage: <command> | echoStylePipe <style>
function echoStylePipe() {
    local style=$1
    local message
    while read message; do
        echoStyle $style "$message"
    done
}

# Function to emulate Dockerfile RUN command, and if it returns a code of 0, append it to the end of $DOCKERFILE
# Otherwise, ask the user if they want to add to the Dockerfile

append_line() {
    # Arguments:
    # $1: File
    # $2: Line

    file="$1"
    line="$2"

    # Check if file exists
    if [ ! -f "$file" ]; then
        echo "File $file does not exist."
        return 1
    fi

    # Check if file ends with a newline
    last_char=$(tail -c 1 "$file")
    if [ "$last_char" != "" ]; then
        # File does not end with newline. Append newline first
        echo "" >> "$file"
    fi

    # Append line
    echo "$line" >> "$file"
}

function RUN() {
    local cmd="$@"
    cmd=${cmd/apt-get /\/usr\/local\/bin\/apt-get-wrapper.sh }
    eval "$cmd"
    local status=$?
    if [ $status -ne 0 ]; then
        echoColor "red" "The command failed with exit code $status." >&2
        echoColor "red" "Do you want to add this command to the Dockerfile? (y/n)" >&2
        read -n 1 -r
        echo >&2
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            append_line "$DOCKERFILE" "RUN $cmd"
        fi
    else
        echoColor "green" "The command succeeded with exit code $status. Adding to Dockerfile."
        append_line "$DOCKERFILE" "RUN $cmd"
    fi
    return $status
}

# Function to generate uninstallation commands for common install commands
# Usage: SUGGEST_UNINSTALL <command>
function SUGGEST_UNINSTALL() {
    local command="$@"
    local uninstall_command

    # Check if the command starts with sudo
    local sudo_prefix=""
    if [[ $command == sudo\ * ]]; then
        sudo_prefix="sudo "
        command=${command#sudo }  # Remove sudo from command
    fi

    if [[ $command == apt\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
    elif [[ $command == apt-get\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
    elif [[ $command == yum\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == dnf\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == zypper\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == apk\ add* ]]; then
        uninstall_command=$(echo "$command" | sed 's/add/del/')
    elif [[ $command == pip\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == pip3\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == npm\ install\ -g* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == gem\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == cargo\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == /usr/local/bin/apt-get-wrapper.sh\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
        uninstall_command=${uninstall_command//apt-get-wrapper.sh/apt-get}
    fi

    uninstall_command="${sudo_prefix}${uninstall_command}"
    echo "$uninstall_command"
}

function UNRUN() {
    # If no arguments are given
    if [ $# -eq 0 ]; then
        # Look for the last "RUN " prefixed line of the Dockerfile
        local last_run_command=$(tac $DOCKERFILE | grep -m1 -oP '^RUN \K.*')
        
        # If there is a last RUN command
        if [[ -n $last_run_command ]]; then
            echoColor "yellow" "The last RUN command in the Dockerfile is: $last_run_command"
            echoColor "yellow" "Do you want to UNRUN this command? (y/n)"
            read -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                UNRUN $last_run_command
            fi
        else
            echoColor "red" "No RUN command found in the Dockerfile."
        fi
    else
        local command="RUN $@"
        local line_num
        line_num=$(grep -n -F -- "$command" $DOCKERFILE | cut -d: -f1)
        if [ -n "$line_num" ]; then
            sed -i "${line_num}d" $DOCKERFILE
            echoColor "green" "Command removed from Dockerfile."

            local uninstall_command=$(SUGGEST_UNINSTALL "$@")
            if [[ -n $uninstall_command ]]; then
                echoColor "yellow" "A corresponding uninstall command was found: $uninstall_command"
                echoColor "yellow" "Running uninstall command..."
                $uninstall_command
            else
                echoColor "green" "Please manually remove any files created by this command, or run 'docker-compose build --no-cache' to rebuild the image."
            fi
        else
            echoColor "red" "Command not found in Dockerfile."
        fi
    fi
}

function rbcinfo() {
    # Show what device we are on
    echo -n "$(echoColor white "Device: ")"
    if [ "$DEV_ENV" == "0" ]; then
        echoColor "blue" "$(echoStyle bold "RoboCapture Raspberry Pi")"
    elif [ "$DEV_ENV" == "1" ]; then
        echoColor "green" "$(echoStyle bold "RoboCapture Dev Environment")"
    else
        # if $DEV_ENV not set, echo Device: Unknown
        if [ -z "$DEV_ENV" ]; then
            echoColor "red" "$(echoStyle bold "Unknown, \$DEV_ENV is not set, should not happen!")"
        else
            echoColor "red" "$(echoStyle bold "Unknown ($DEV_ENV)")"
        fi
    fi
    # Show if we are on bot or local
    echo -n "$(echoColor white "Master: ")"
    if [ "$RBC_MASTER" == "bot" ]; then
        echoColor "blue" "$(echoStyle bold "Bot")"
    elif [ "$RBC_MASTER" == "local" ]; then
        echoColor "green" "$(echoStyle bold "Local")"
    else
        if [ -z "$RBC_MASTER" ]; then
            echoColor "red" "$(echoStyle bold "Unknown, \$RBC_MASTER is not set")"
        else
            echoColor "red" "$(echoStyle bold "Unknown ($RBC_MASTER)")"
        fi
    fi
    # Display ROS_MASTER_URI
    local uri_line="$(echoColor white "ROS_MASTER_URI: ")"
    echo -n "$uri_line"
    if [ -z "$ROS_MASTER_URI" ]; then
        echoColor "red" "$(echoStyle bold "Not set")"
    else
        echoColor "green" "$(echoStyle bold "$ROS_MASTER_URI")"
    fi

    # Display ROS_IP
    local ip_line="$(echoColor white "ROS_IP: ")"
    echo -n "$ip_line"
    if [ -z "$ROS_IP" ]; then
        if [ -z "$ROS_HOSTNAME" ]; then
            echoColor "red" "$(echoStyle bold "Not set")"
        else
            echo "Not set, using ROS_HOSTNAME"
        fi
    else
        echoColor "green" "$(echoStyle bold "$ROS_IP")"
    fi

    # Display ROS_HOSTNAME
    local hostname_ros_line="$(echoColor white "ROS_HOSTNAME: ")"
    echo -n "$hostname_ros_line"
    if [ -z "$ROS_HOSTNAME" ]; then
        if [ -z "$ROS_IP" ]; then
            echoColor "red" "$(echoStyle bold "Not set")"
        else
            echo "Not set, using ROS_IP"
        fi
    else
        echoColor "green" "$(echoStyle bold "$ROS_HOSTNAME")"
    fi
    # Check if roscore is running on the master by trying to run rostopic list
    local roscore_line="$(echoColor white "roscore status: ")"
    echo -n "$roscore_line"
    echoColor "yellow" "$(echoStyle blink "Checking master status...")"
    if rostopic list &> /dev/null; then
        # Remove last line (the checking master status message)
        tput cuu 1 && tput el
        # Echo that it is running
        echo -n "$roscore_line"
        echoColor "green" "$(echoStyle bold "Running")"
    else
        # Remove last line (the checking master status message)
        tput cuu 1 && tput el
        # Echo that it is not running
        echo -n "$roscore_line"
        echoColor "red" "$(echoStyle bold "Not running")"
    fi
    # Check if bot hostname is reachable (ping)
    local hostname_line="$(echoColor white "Bot status ($DEV_BOT_HOSTNAME): ")"
    echo -n "$hostname_line"
    echoColor "yellow" "$(echoStyle blink "Pinging...")"

    if ping -w 1 "$DEV_BOT_HOSTNAME" &> /dev/null; then
        # Remove last line (the pinging message)
        tput cuu 1 && tput el

        # Echo that it is reachable
        echo -n "$hostname_line"
        echoColor "green" "$(echoStyle bold "Online on $(getent hosts "$DEV_BOT_HOSTNAME" | awk '{ print $1 }')")"
    else
        # Remove last line (the pinging message)
        tput cuu 1 && tput el

        # Echo that it is not reachable
        echo -n "$hostname_line"
        echoColor "red" "$(echoStyle bold "Unreachable")"
    fi
}

# Remote bot utilities
alias botsh='ssh $DEV_BOT_USER@$DEV_BOT_HOSTNAME' # SSH into bot. Technically, can be used like botdo, but it does not support the aliases defined in .bashrc.
alias bsh='botsh'

# Make bot run command with all environmental variables
function botdo() {
    local command="$@"
    # Early terminate if no command is given
    if [ -z "$command" ]; then
        echo "Usage: botdo <command>"
        return 1
    fi
    ssh -t $DEV_BOT_USER@$DEV_BOT_HOSTNAME "bash -i -c 'shopt -s expand_aliases; source ~/.bashrc; $command'"
}
alias bd='botdo'

function alldo() {
    local command="$@"
    if [ -z "$command" ]; then
        echo "Usage: alldo <command>"
        return 1
    fi
    # Run command locally, then run command on bot
    echoColor yellow "Running command locally..."
    eval "$command"
    echoColor yellow "Running command on bot..."
    botdo "$command"
}
alias ad='alldo'

alias botsync="rsync -avz ~/rbc/ $DEV_BOT_USER@$DEV_BOT_HOSTNAME:~/rbc/ --delete" # Directly sync bot repo with local repo without github
alias bs='botsync'

alias cbs="botsync && alldo cb" # Catkin build on both local and bot; catkin build sync

