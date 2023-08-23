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
cb # Catkin build
abl # Build Arduino libraries
ac # Compile Arduino sketch
echo -e "\e[36mDev environment initialized, compiled catkin_ws, arduino rosserial libraries, and arduino sketch.\e[0m"