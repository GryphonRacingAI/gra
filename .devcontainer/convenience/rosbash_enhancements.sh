depends_func cbs

launch() {
    # Run roslaunch in the background and suppress its output
    roslaunch "$@" > /dev/null 2>&1 &
    roslaunch_pid=$!

    # This trap will handle Ctrl-C to ensure both processes receive the SIGINT signal
    trap "kill -INT $roslaunch_pid" INT

    # Run rosout_display in the foreground
    rosrun rbc_core rosout_display.py
    display_pid=$!

    # If the rosrun command ends first (or for any other reasons),
    # we should send SIGINT to roslaunch to ensure it ends gracefully as well
    kill -INT $roslaunch_pid

    # Cleanup trap
    trap - INT
}

complete -o filenames -F _roscomplete_launch launch

export -f launch