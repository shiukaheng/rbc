# Where things are located on the system (relative paths should mean it works on the container and the Pi)
export WORKSPACE_REPO=~/workspace
export DOCKERFILE=$WORKSPACE_REPO/.devcontainer/Dockerfile

# Persistent variables
# If ~/.penv/persistent_vars.sh does not exist, create it
if [ ! -f ~/.penv/persistent_vars.sh ]; then
    mkdir -p ~/.penv
    touch ~/.penv/persistent_vars.sh
fi
export PERSISTENT_FILE=$(realpath ~/.penv/persistent_vars.sh)