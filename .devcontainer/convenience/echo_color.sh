# Usage: echoColor <color> <message>
# Colors: black, red, green, yellow, blue, magenta, cyan, white
function echoColor() {
    local color=$1
    local message=$2
    local colorCode

    case $color in
        black)
            colorCode=0
            ;;
        red)
            colorCode=1
            ;;
        green)
            colorCode=2
            ;;
        yellow)
            colorCode=3
            ;;
        blue)
            colorCode=4
            ;;
        magenta)
            colorCode=5
            ;;
        cyan)
            colorCode=6
            ;;
        white)
            colorCode=7
            ;;
        bright-black)
            colorCode=8
            ;;
        bright-red)
            colorCode=9
            ;;
        bright-green)
            colorCode=10
            ;;
        bright-yellow)
            colorCode=11
            ;;
        bright-blue)
            colorCode=12
            ;;
        bright-magenta)
            colorCode=13
            ;;
        bright-cyan)
            colorCode=14
            ;;
        bright-white)
            colorCode=15
            ;;
        *)
            echo "Invalid color: $color"
            return 1
            ;;
    esac
    echo -e "\033[38;5;${colorCode}m${message}\033[0m"
}

# Convenience function for piping to echoColor
# Usage: <command> | echoColorPipe <color>
function echoColorPipe() {
    local color=$1
    local message
    while read message; do
        echoColor $color "$message"
    done
}

# Convenience function for adding style to text
# Usage: echoStyle <style> <message>
# Styles: bold, italic, underline, blink, inverse, hidden, strikethrough
function echoStyle() {
    local style=$1
    local message=$2
    local styleCode
    case $style in
        bold)
            styleCode=1
            ;;
        italic)
            styleCode=3
            ;;
        underline)
            styleCode=4
            ;;
        blink)
            styleCode=5
            ;;
        inverse)
            styleCode=7
            ;;
        hidden)
            styleCode=8
            ;;
        strikethrough)
            styleCode=9
            ;;
        *)
            echo "Invalid style: $style"
            return 1
            ;;
    esac
    echo -e "\033[${styleCode}m${message}\033[0m"
}

# Convenience function for piping to echoStyle
# Usage: <command> | echoStylePipe <style>
function echoStylePipe() {
    local style=$1
    local message
    while read message; do
        echoStyle $style "$message"
    done
}

export -f echoColor
export -f echoColorPipe
export -f echoStyle
export -f echoStylePipe