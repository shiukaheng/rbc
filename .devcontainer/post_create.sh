source ~/.bashrc
source ~/convenience.sh

# Generate ssh keys if they don't exist at ~/.ssh/id_rsa
if [ ! -f ~/.ssh/id_rsa ]; then
    echo -e "\e[36mGenerating ssh keys...\e[0m"
    ssh-keygen -t rsa -b 4096 -C "rbc@devcontainer" -f ~/.ssh/id_rsa -q -N ""
    echo -e "\e[36mSSH keys generated.\e[0m"
fi

# Add ssh keys to ssh-agent
ssh-add ~/.ssh/id_rsa

echo -e "\e[36mInitializing dev environment...\e[0m"   

# cb # Catkin build
cd $CATKIN_WS_PATH
catkin build
# abl # Build Arduino libraries
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries && patch_rosserial_arduino_port
# ac # Compile Arduino sketch
cd $RBC_REPO/controller
arduino-cli compile --fqbn arduino:avr:mega

echo -e "\e[36mDev environment initialized, compiled catkin_ws, arduino rosserial libraries, and arduino sketch.\e[0m"