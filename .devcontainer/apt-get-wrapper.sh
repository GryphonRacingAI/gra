#!/bin/bash

DAY_IN_SECONDS=86400
FILE="/var/cache/apt/pkgcache.bin"

if [ ! -f "$FILE" ]; then
    apt-get update
else
    CACHE_AGE=$(($(date +%s) - $(stat -c %Y "$FILE")))
    
    if [ "$CACHE_AGE" -gt "$DAY_IN_SECONDS" ]; then
        apt-get update
    fi
fi

apt-get "$@"