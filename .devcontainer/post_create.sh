mkdir -p ~/catkin_ws
# Make link from ~/gra/ros to ~/catkin_ws/src
if [ ! -d "$HOME/catkin_ws/src" ]; then
    ln -s ~/gra/ros ~/catkin_ws/src
fi

# 3D pointcloud slam
# cd ~/catkin_ws/src
# git clone https://github.com/koide3/ndt_omp.git
# git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
# git clone https://github.com/koide3/hdl_graph_slam

# # VSLAM
# cd ~
# git clone https://github.com/RainerKuemmerle/g2o.git
# cd g2o
# mkdir build
# cd build
# cmake ..
# make

# cd ~
# git clone https://github.com/stella-cv/FBoW.git
# cd FBoW && mkdir build && cd build
# cmake .. -DBUILD_TESTS=ON -DBUILD_UTILS=ON
# make

mkdir -p ~/lib
cd ~/lib
git clone --recursive --depth 1 https://github.com/stella-cv/stella_vslam.git
rosdep install -y -i --from-paths ~/lib
cd ~/lib/stella_vslam
mkdir -p ~/lib/stella_vslam/build
cd ~/lib/stella_vslam/build
source /opt/ros/${ROS_DISTRO}/setup.bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j
sudo make install

cd ~
git clone --recursive -b ros https://github.com/stella-cv/stella_vslam_ros.git
cd stella_vslam_ros
git submodule update --init --recursive
ln -s ~/stella_vslam_ros ~/catkin_ws/src/stella_vslam_ros



# # Generate ssh keys if they don't exist at ~/.ssh/id_rsa
# if [ ! -f ~/.ssh/id_rsa ]; then
#     echo -e "\e[36mGenerating ssh keys...\e[0m"
#     ssh-keygen -t rsa -b 4096 -C "gra@devcontainer" -f ~/.ssh/id_rsa -q -N ""
#     echo -e "\e[36mSSH keys generated.\e[0m"
# fi

# Create devcontainer.metadata from template if it doesn't exist
if [ ! -f ~/gra/.devcontainer/devcontainer.metadata ]; then
    echo -e "\e[36mCreating devcontainer.metadata ...\e[0m"
    cp ~/gra/.devcontainer/devcontainer.metadata_template ~/gra/.devcontainer/devcontainer.metadata
fi

source ~/convenience.sh -i
setdevmaster
cb

# bash -ic "source ~/.bashrc && setbotmaster; acm"

python3 /root/gra/ros/ackermann_vehicle_navigation/scripts/numba_precompile.py
