depends_var DEV_ENV
depends_func echoColor

function rbcinfo() {
    # Show what device we are on
    echo -n "$(echoColor white "Device: ")"
    if [ "$DEV_ENV" == "0" ]; then
        echoColor "blue" "$(echoStyle bold "RoboCapture Raspberry Pi")"
    elif [ "$DEV_ENV" == "1" ]; then
        echoColor "green" "$(echoStyle bold "RoboCapture Dev Environment")"
    else
        # if $DEV_ENV not set, echo Device: Unknown
        if [ -z "$DEV_ENV" ]; then
            echoColor "red" "$(echoStyle bold "Unknown, \$DEV_ENV is not set, should not happen!")"
        else
            echoColor "red" "$(echoStyle bold "Unknown ($DEV_ENV)")"
        fi
    fi
    # Show if we are on bot or local
    echo -n "$(echoColor white "Master: ")"
    if [ "$RBC_MASTER" == "bot" ]; then
        echoColor "blue" "$(echoStyle bold "Bot")"
    elif [ "$RBC_MASTER" == "local" ]; then
        echoColor "green" "$(echoStyle bold "Local")"
    else
        if [ -z "$RBC_MASTER" ]; then
            echoColor "red" "$(echoStyle bold "Unknown, \$RBC_MASTER is not set, please run setdevmaster or setbotmaster")"
        else
            echoColor "red" "$(echoStyle bold "Unknown ($RBC_MASTER)")"
        fi
    fi
    # Display ROS_MASTER_URI
    local uri_line="$(echoColor white "ROS_MASTER_URI: ")"
    echo -n "$uri_line"
    if [ -z "$ROS_MASTER_URI" ]; then
        echoColor "red" "$(echoStyle bold "Not set")"
    else
        echoColor "green" "$(echoStyle bold "$ROS_MASTER_URI")"
    fi

    # Display ROS_IP
    local ip_line="$(echoColor white "ROS_IP: ")"
    echo -n "$ip_line"
    if [ -z "$ROS_IP" ]; then
        if [ -z "$ROS_HOSTNAME" ]; then
            echoColor "red" "$(echoStyle bold "Not set")"
        else
            echo "Not set, using ROS_HOSTNAME"
        fi
    else
        echoColor "green" "$(echoStyle bold "$ROS_IP")"
    fi

    # Display ROS_HOSTNAME
    local hostname_ros_line="$(echoColor white "ROS_HOSTNAME: ")"
    echo -n "$hostname_ros_line"
    if [ -z "$ROS_HOSTNAME" ]; then
        if [ -z "$ROS_IP" ]; then
            echoColor "red" "$(echoStyle bold "Not set")"
        else
            echo "Not set, using ROS_IP"
        fi
    else
        echoColor "green" "$(echoStyle bold "$ROS_HOSTNAME")"
    fi
    # Check if roscore is running on the master by trying to run rostopic list
    local roscore_line="$(echoColor white "roscore status: ")"
    echo -n "$roscore_line"
    echoColor "yellow" "$(echoStyle blink "Checking master status...")"
    if rostopic list &> /dev/null; then
        # Remove last line (the checking master status message)
        tput cuu 1 && tput el
        # Echo that it is running
        echo -n "$roscore_line"
        echoColor "green" "$(echoStyle bold "Running")"
    else
        # Remove last line (the checking master status message)
        tput cuu 1 && tput el
        # Echo that it is not running
        echo -n "$roscore_line"
        echoColor "red" "$(echoStyle bold "Not running")"
    fi
    # Check if bot hostname is reachable (ping)
    local hostname_line="$(echoColor white "Bot status ($DEV_BOT_HOSTNAME): ")"
    echo -n "$hostname_line"
    echoColor "yellow" "$(echoStyle blink "Pinging...")"

    if ping -w 1 "$DEV_BOT_HOSTNAME" &> /dev/null; then
        # Remove last line (the pinging message)
        tput cuu 1 && tput el

        # Echo that it is reachable
        echo -n "$hostname_line"
        echoColor "green" "$(echoStyle bold "Online on $(getent hosts "$DEV_BOT_HOSTNAME" | awk '{ print $1 }')")"
    else
        # Remove last line (the pinging message)
        tput cuu 1 && tput el

        # Echo that it is not reachable
        echo -n "$hostname_line"
        echoColor "red" "$(echoStyle bold "Unreachable")"
    fi
}

export -f rbcinfo