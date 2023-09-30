#!/bin/bash

source ~/workspace/.devcontainer/env_vars.sh

function import() {
    source $WORKSPACE_REPO/.devcontainer/convenience/$1.sh
}

# Basic utilities
import refreshenv
import logging
import depends
import run_in_dir
import persistent_exports
import echo_color

# Dockerfile emulation
import dockerfile_emu