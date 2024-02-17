#!/bin/bash

DAY_IN_SECONDS=86400
FILE="/var/cache/apt/pkgcache.bin"

function update_packages()
{   
    # Add RealSense source
    mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list
    
    apt-get update
}

if [ ! -f "$FILE" ] | [ ! "$(ls -A /var/lib/apt/lists)" ]; then
    update_packages
else
    CACHE_AGE=$(($(date +%s) - $(stat -c %Y "$FILE")))
    
    if [ "$CACHE_AGE" -gt "$DAY_IN_SECONDS" ]; then
        update_packages
    fi
fi

apt-get install -y "$@"