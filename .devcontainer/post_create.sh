mkdir -p ~/catkin_ws
# Make link from ~/gra/ros to ~/catkin_ws/src
if [ ! -d "$HOME/catkin_ws/src" ]; then
    ln -s ~/gra/ros ~/catkin_ws/src
fi

# ov2slam
# cd ~/catkin_ws/src/
# git clone https://github.com/ov2slam/ov2slam.git
# cd ~/catkin_ws/src/ov2slam
# chmod +x build_thirdparty.sh
# ./build_thirdparty.sh

# 3D pointcloud SLAM
cd ~/catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam
git -C hdl_graph_slam apply ~/gra/.devcontainer/patches/hdl_graph_slam.patch

# 2D Laser Odometry and SLAM
cd ~/catkin_ws/src
git clone https://github.com/ugsfume/rf2o_laser_odometry.git

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
CMAKE_ARGS=""
# ndt_omp tries to compile with SSE optimisations, which are only available on x86, unless
# the following option is set.
CMAKE_ARGS+="-DBUILD_WITH_MARCH_NATIVE=ON"
# The CMake version installed by the Dockerfile has a bug in its FindOpenMP.cmake.  Work
# around this by avoiding having to find OpenMP manually at all.
CMAKE_ARGS+=" -DOPENMP_C_FLAGS=-fopenmp -DOPENMP_CXX_FLAGS=-fopenmp"
cb --cmake-args $CMAKE_ARGS

# bash -ic "source ~/.bashrc && setbotmaster; acm"

python3 /root/gra/ros/ackermann_vehicle_navigation/scripts/numba_precompile.py
