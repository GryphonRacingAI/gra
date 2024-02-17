#!/bin/bash
# Dynamically create the roscore.service file
echo "Generating roscore.service for user $USER..."
cat > roscore.service <<EOL
[Unit]
Description=roscore service
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -ic 'source ~/.bashrc && roscore'
Restart=always
User=$USER

[Install]
WantedBy=multi-user.target
EOL

# Check if roscore.service exists before copying
if [ ! -f roscore.service ]; then
    echo -e "${YELLOW}Warning: roscore.service does not exist in current directory. Skipping...${RESET}"
else
    echo "Copying roscore.service to /etc/systemd/system/"
    sudo cp roscore.service /etc/systemd/system/

    echo "Reloading systemd to recognize the service..."
    sudo systemctl daemon-reload

    echo "Enabling the roscore.service to start on boot..."
    sudo systemctl enable roscore.service   

    echo "Starting the roscore.service..."
    sudo systemctl start roscore.service
fi