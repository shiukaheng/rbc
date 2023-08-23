# Depends on .bashrc for variables
depends_func run_in_dir
depends_var CATKIN_WS_PATH

function cb() {
    run_in_dir "$CATKIN_WS_PATH" catkin build "$@"
    refreshenv
}

function cpp() {
    run_in_dir "$CATKIN_WS_PATH" catkin_create_pkg "$@"
    refreshenv
}

export -f cb
export -f cpp