# Generate ssh keys if they don't exist at ~/.ssh/id_rsa
if [ ! -f ~/.ssh/id_rsa ]; then
    echo -e "\e[36mGenerating ssh keys...\e[0m"
    ssh-keygen -t rsa -b 4096 -C "rbc@devcontainer" -f ~/.ssh/id_rsa -q -N ""
    echo -e "\e[36mSSH keys generated.\e[0m"
fi

# Add ssh keys to ssh-agent
ssh-add ~/.ssh/id_rsa

# Run acm
/bin/bash -c "source ~/rbc/.devcontainer/convenience.sh && acm"