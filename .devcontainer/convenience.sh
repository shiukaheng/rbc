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
alias cdws='cd $CATKIN_WS_PATH' # CD to workspace

# Catkin
alias cb='run_in_directory "catkin build" "$CATKIN_WS_PATH" && refreshenv' # Catkin Build and refresh environment

# Patch files with string replacement
function patch() {
  local file="$1"
  local search="$2"
  local replace="$3"
  sed -i "s/$search/$replace/g" "$file"
}

# Arduino
export ARDUINO_PORT="/dev/ttyACM0" # Arduino port
alias ac='run_in_directory "arduino-cli compile --fqbn arduino:avr:mega" "$RBC_REPO/controller"' # Arduino Compile
alias au='run_in_directory "arduino-cli upload -p $ARDUINO_PORT --fqbn arduino:avr:mega" "$RBC_REPO/controller"' # Arduino Upload
alias acu='ac && au' # Arduino Compile and Upload
alias patch_rosserial_arduino_port='patch "/root/Arduino/libraries/ros_lib/ArduinoHardware.h" "iostream = &Serial;" "iostream = \\&Serial3;"' # Patch rosserial_arduino port
alias abl='rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries && patch_rosserial_arduino_port' # Arduino Build Libraries
alias acm='cb && abl && ac' # Arduino Compile Macro (Compiles all dependencies, compiles the sketch)

# Launch files

# Git
alias update='run_in_directory "git pull && refreshenv" "$RBC_REPO"' # Update repo
alias discard_changes='run_in_directory "git reset --hard HEAD" "$RBC_REPO"' # Discard changes
alias commit='run_in_directory "git add . && git commit -m" "$RBC_REPO"' # Commit changes

# Convenience alias for setting remote or local ROS_MASTER_URI

export DEV_MASTER_URI="http://localhost:11311"
export DEV_BOT_HOSTNAME="rbc.local"
export DEV_BOT_MASTER_URI="http://$DEV_BOT_HOSTNAME:11311"

export LOCAL_IP=$(hostname -I | awk '{print $1}')


alias setdevmaster='export ROS_MASTER_URI=$DEV_MASTER_URI && export ROS_HOSTNAME=$LOCAL_IP'
alias setbotmaster='export ROS_MASTER_URI=$DEV_BOT_MASTER_URI && export ROS_HOSTNAME=$LOCAL_IP'
alias checkmaster='echo Hostname: $ROS_HOSTNAME, Master URI: $ROS_MASTER_URI'

# Convenience function for echoing with color
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
        *)
            echo "Invalid color: $color"
            return 1
            ;;
    esac
    echo -e "\033[3${colorCode}m${message}\033[0m"
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
    "$@"
    local status=$?
    local cmd="$*"
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

# Autocompletion for UNRUN
function _UNRUN() {
    local curr_arg;
    COMPREPLY=()
    curr_arg=${COMP_WORDS[COMP_CWORD]}
    local dockerfile_contents=$(grep "^RUN " $REPOSITORY/.devcontainer/Dockerfile | sed 's/^RUN //')

    # Join the matched commands into a single string
    local matches=$(compgen -W "${dockerfile_contents}" -- $curr_arg)
    if [ -n "$matches" ]; then
        COMPREPLY=( $(echo "${matches[*]}" | tr ' ' '\n') )
    fi
}

complete -F _UNRUN UNRUN