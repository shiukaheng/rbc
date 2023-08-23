depends_func echo_info

function log() {
    rosrun rbc_core rosout_display.py
}

function dev() {
    log &             # start log in the background
    local log_pid=$!  # get the PID of the log process

    roslaunch rbc_dev dev.launch

    # After roslaunch completes, kill the log process
    kill -9 $log_pid 2>/dev/null
}

export -f dev
export -f log