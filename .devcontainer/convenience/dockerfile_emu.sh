depends_func echoColor

append_line() {
    # Arguments:
    # $1: File
    # $2: Line

    file="$1"
    line="$2"

    # Check if file exists
    if [ ! -f "$file" ]; then
        echo "File $file does not exist."
        return 1
    fi

    # Check if file ends with a newline
    last_char=$(tail -c 1 "$file")
    if [ "$last_char" != "" ]; then
        # File does not end with newline. Append newline first
        echo "" >> "$file"
    fi

    # Append line
    echo "$line" >> "$file"
}

function RUN() {
    local cmd="$@"
    cmd=${cmd/apt-get /\/usr\/local\/bin\/apt-get-wrapper.sh }
    eval "$cmd"
    local status=$?
    if [ $status -ne 0 ]; then
        echoColor "red" "The command failed with exit code $status." >&2
        echoColor "red" "Do you want to add this command to the Dockerfile? (y/n)" >&2
        read -n 1 -r
        echo >&2
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            append_line "$DOCKERFILE" "RUN $cmd"
        fi
    else
        echoColor "green" "The command succeeded with exit code $status. Adding to Dockerfile."
        append_line "$DOCKERFILE" "RUN $cmd"
    fi
    return $status
}

# Function to generate uninstallation commands for common install commands
# Usage: SUGGEST_UNINSTALL <command>
function SUGGEST_UNINSTALL() {
    local command="$@"
    local uninstall_command

    # Check if the command starts with sudo
    local sudo_prefix=""
    if [[ $command == sudo\ * ]]; then
        sudo_prefix="sudo "
        command=${command#sudo }  # Remove sudo from command
    fi

    if [[ $command == apt\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
    elif [[ $command == apt-get\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
    elif [[ $command == yum\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == dnf\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == zypper\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/remove/')
    elif [[ $command == apk\ add* ]]; then
        uninstall_command=$(echo "$command" | sed 's/add/del/')
    elif [[ $command == pip\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == pip3\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == npm\ install\ -g* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == gem\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == cargo\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/uninstall/')
    elif [[ $command == /usr/local/bin/apt-get-wrapper.sh\ install* ]]; then
        uninstall_command=$(echo "$command" | sed 's/install/purge/')
        uninstall_command=${uninstall_command//apt-get-wrapper.sh/apt-get}
    fi

    uninstall_command="${sudo_prefix}${uninstall_command}"
    echo "$uninstall_command"
}

function UNRUN() {
    # If no arguments are given
    if [ $# -eq 0 ]; then
        # Look for the last "RUN " prefixed line of the Dockerfile
        local last_run_command=$(tac $DOCKERFILE | grep -m1 -oP '^RUN \K.*')
        
        # If there is a last RUN command
        if [[ -n $last_run_command ]]; then
            echoColor "yellow" "The last RUN command in the Dockerfile is: $last_run_command"
            echoColor "yellow" "Do you want to UNRUN this command? (y/n)"
            read -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                UNRUN $last_run_command
            fi
        else
            echoColor "red" "No RUN command found in the Dockerfile."
        fi
    else
        local command="RUN $@"
        local line_num
        line_num=$(grep -n -F -- "$command" $DOCKERFILE | cut -d: -f1)
        if [ -n "$line_num" ]; then
            sed -i "${line_num}d" $DOCKERFILE
            echoColor "green" "Command removed from Dockerfile."

            local uninstall_command=$(SUGGEST_UNINSTALL "$@")
            if [[ -n $uninstall_command ]]; then
                echoColor "yellow" "A corresponding uninstall command was found: $uninstall_command"
                echoColor "yellow" "Running uninstall command..."
                $uninstall_command
            else
                echoColor "green" "Please manually remove any files created by this command, or run 'docker-compose build --no-cache' to rebuild the image."
            fi
        else
            echoColor "red" "Command not found in Dockerfile."
        fi
    fi
}

export -f append_line
export -f RUN
export -f SUGGEST_UNINSTALL
export -f UNRUN