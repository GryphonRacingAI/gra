#!/bin/bash

DAY_IN_SECONDS=86400
FILE="/var/cache/apt/pkgcache.bin"

function update_packages()
{    
    apt-get update
}

if [ ! -f "$FILE" ] | [ ! "$(ls -A /var/lib/apt/lists)" ]; then
    update_packages
else
    CACHE_AGE=$($(date +%s) - $(stat -c %Y "$FILE"))
    
    if [ "$CACHE_AGE" -gt "$DAY_IN_SECONDS" ]; then
        update_packages
    fi
fi

apt-get install -y "$@"