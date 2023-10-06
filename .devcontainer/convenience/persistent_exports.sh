depends_var PERSISTENT_FILE
depends_func echo_success

pexport() {
    # Check for the correct number of arguments
    if [[ $# -ne 2 ]]; then
        echo "Usage: pexport VARNAME VALUE"
        return 1
    fi

    local var_name="$1"
    local value="$2"
    local comment="# pexport generated"

    # If PERSISTENT_FILE does not exist, create it
    if [ ! -f "$PERSISTENT_FILE" ]; then
        touch "$PERSISTENT_FILE"
    fi

    # Remove the previous pexported line if it exists
    grep -v "export $var_name=" "$PERSISTENT_FILE" | grep -v "$var_name=.*$comment" > "${PERSISTENT_FILE}.tmp"
    mv "${PERSISTENT_FILE}.tmp" "$PERSISTENT_FILE"

    # Add the new pexport line
    echo "export $var_name=\"$value\" $comment" >> "$PERSISTENT_FILE"

    echo_success "Variable $var_name has been set to \"$value\" and pexported."
    source "$PERSISTENT_FILE"
}

punset() {
    # Check for the correct number of arguments
    if [[ $# -ne 1 ]]; then
        echo "Usage: punset VARNAME"
        return 1
    fi

    local var_name="$1"
    local comment="# pexport generated"

    # If PERSISTENT_FILE does not exist, create it
    if [ ! -f "$PERSISTENT_FILE" ]; then
        touch "$PERSISTENT_FILE"
    fi

    # Remove the pexported line
    grep -v "export $var_name=" "$PERSISTENT_FILE" | grep -v "$var_name=.*$comment" > "${PERSISTENT_FILE}.tmp"
    mv "${PERSISTENT_FILE}.tmp" "$PERSISTENT_FILE"

    echo_success "Variable $var_name has been unset and removed from pexported variables."
    source "$PERSISTENT_FILE"
}

# Source $PERSISTENT_FILE if it exists
if [ -f "$PERSISTENT_FILE" ]; then
    source "$PERSISTENT_FILE"
fi