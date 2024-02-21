# Make link from ~/gra/ros to ~/catkin_ws/src
if [ ! -d "$HOME/catkin_ws/src" ]; then
    ln -s ~/gra/ros ~/catkin_ws/src
fi

# # Generate ssh keys if they don't exist at ~/.ssh/id_rsa
# if [ ! -f ~/.ssh/id_rsa ]; then
#     echo -e "\e[36mGenerating ssh keys...\e[0m"
#     ssh-keygen -t rsa -b 4096 -C "gra@devcontainer" -f ~/.ssh/id_rsa -q -N ""
#     echo -e "\e[36mSSH keys generated.\e[0m"
# fi

source ~/convenience.sh -i
setdevmaster
cb
# bash -ic "source ~/.bashrc && setbotmaster; acm"
