# Update function
rbc-update() {
    # Save current working directory
    CWD=$(pwd)
    cd ~/rbc
    # Abandon all changes
    git reset --hard
    # Pull latest changes
    git pull
    # Pull latest docker images
    docker pull --platform shiukaheng/rbc:latest
    # Build docker images
    cd ~/rbc/.devcontainer
    docker-compose -f docker-compose.rpi.yml build
    # Restore working directory
    cd $CWD
}

# Launch function
rbc-launch() {
    # Save current working directory
    CWD=$(pwd)
    cd ~/rbc/.devcontainer
    # Docker-compose docker-compose.rpi.yml
    docker-compose -f docker-compose.rpi.yml up -d
    # Restore working directory
    cd $CWD
}