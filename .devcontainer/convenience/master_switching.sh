depends_func pexport

function setdevmaster() {
    pexport ROS_MASTER_URI "$DEV_MASTER_URI" && pexport ROS_HOSTNAME $DEV_BOT_HOSTNAME && pexport GRA_MASTER 'local'
}

function sdm() {
    setdevmaster
}

function setbotmaster() {
    pexport ROS_MASTER_URI "$DEV_BOT_MASTER_URI" && pexport ROS_HOSTNAME "$(hostname).local" && pexport GRA_MASTER 'bot'
}
function sbm() {
    setbotmaster
}

function checkmaster() {
    echo "Hostname: $ROS_HOSTNAME, Master URI: $ROS_MASTER_URI"
}
function cm() {
    checkmaster
}

export -f setdevmaster
export -f sdm
export -f setbotmaster
export -f sbm
export -f checkmaster
export -f cm