function log() {
    rosrun rbc_core rosout_display.py
}

function dev() {
    roslaunch rbc_dev dev.launch
}

export -f dev
export -f log