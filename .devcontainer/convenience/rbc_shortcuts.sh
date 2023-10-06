function log() {
    rosrun gra_core rosout_display.py
}

function dev() {
    roslaunch gra_dev dev.launch
}

export -f dev
export -f log