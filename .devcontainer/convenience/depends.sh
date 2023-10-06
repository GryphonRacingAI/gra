depends_func() {
    if type "$1" &>/dev/null; then
        if alias "$1" &>/dev/null; then
            # echo "$1 is defined as an alias."
            :
        elif declare -f "$1" &>/dev/null; then
            # echo "$1 is defined as a function."
            :
        else
            # echo "$1 is defined, but not as an alias or function."
            echo_error "$1 is defined, but not as an alias or function."
        fi
    else
        # echo "Error: $1 is not defined as an alias or function."
        echo_error "Error: $1 is not defined as an alias or function."
    fi
}

depends_func echo_info
depends_func echo_warn
depends_func echo_error
depends_func echo_success

depends_var() {
    if [[ -z "$1" ]]; then
        echo_error "Error: No variable name provided."
        return
    fi

    if [[ $(declare -p "$1" 2>/dev/null) ]]; then
        # echo "$1 is defined as a variable."
        :
    else
        # echo "Error: $1 is not defined as a variable."
        echo_error "Error: $1 is not defined as a variable."
    fi
}

export -f depends_func
export -f depends_var