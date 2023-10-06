echo_info() {
    local GREEN="\033[1;32m"
    local RESET="\033[0m"
    echo -e "${GREEN}INFO:${RESET} $*"
}

echo_warn() {
    local YELLOW="\033[1;33m"
    local RESET="\033[0m"
    echo -e "${YELLOW}WARN:${RESET} $*"
}

echo_error() {
    local RED="\033[1;31m"
    local RESET="\033[0m"
    echo -e "${RED}ERROR:${RESET} $*"
}

echo_success() {
    local BLUE="\033[1;34m"
    local RESET="\033[0m"
    echo -e "${BLUE}SUCCESS:${RESET} $*"
}

export -f echo_info
export -f echo_warn
export -f echo_error
export -f echo_success