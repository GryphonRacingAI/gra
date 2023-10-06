depends_func echo_error

run_in_dir() {
    if [ -z "$1" ]; then
        echo_error "Directory not provided."
        return 1
    fi
    
    local dir="$1"
    shift
    
    if [ ! -d "$dir" ]; then
        echo_error "Directory $dir does not exist."
        return 1
    fi

    (cd "$dir" && "$@")
}

export -f run_in_dir