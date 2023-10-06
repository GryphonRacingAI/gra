depends_var DEV_BOT_USER
depends_var DEV_BOT_HOSTNAME
depends_func echoColor

function botsh() {
    ssh "$DEV_BOT_USER@$DEV_BOT_HOSTNAME"
}

function bsh() {
    botsh
}

# Make bot run command with all environmental variables
function botdo() {
    local command="$@"
    # Early terminate if no command is given
    if [ -z "$command" ]; then
        echo "Usage: botdo <command>"
        return 1
    fi
    ssh -t $DEV_BOT_USER@$DEV_BOT_HOSTNAME "bash -i -c 'shopt -s expand_aliases; source ~/.bashrc; $command'"
}

function bd() {
    botdo "$@"
}

function alldo() {
    local command="$@"
    if [ -z "$command" ]; then
        echo "Usage: alldo <command>"
        return 1
    fi
    # Run command locally, then run command on bot
    echoColor yellow "Running command locally..."
    eval "$command"
    echoColor yellow "Running command on bot..."
    botdo "$command"
}

function ad() {
    alldo "$@"
}

function botsync() {
    rsync -avz ~/rbc/ "$DEV_BOT_USER@$DEV_BOT_HOSTNAME:~/rbc/" --delete
}

function bs() {
    botsync
}

function cbs() {
    botsync && alldo cb
}