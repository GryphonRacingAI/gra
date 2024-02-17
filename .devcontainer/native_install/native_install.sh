python3 .devcontainer/native_install/docker2sh.py .devcontainer/Dockerfile > installation_script_generated.sh
cp .devcontainer/apt-get-wrapper.sh /usr/local/bin
chmod +x /usr/local/bin/apt-get-wrapper.sh
chmod +x installation_script_generated.sh
./installation_script_generated.sh
mkdir $HOME/catkin_ws
source bot_scripts/install.sh