depends_var RBC_REPO
depends_func run_in_dir
depends_func cb

function patch() {
  local file="$1"
  local search="$2"
  local replace="$3"
  sed -i "s/$search/$replace/g" "$file"
}

function ac() {
  run_in_dir "$RBC_REPO/controller" arduino-cli compile --fqbn arduino:avr:mega "$@"
}

function au() {
  run_in_dir "$RBC_REPO/controller" arduino-cli upload -p "$ARDUINO_UPDATE_PORT" --fqbn arduino:avr:mega "$@"
}

function acu() {
  ac && au
}

function patch_rosserial_arduino_port() {
  patch "$HOME/Arduino/libraries/ros_lib/ArduinoHardware.h" "iostream = &Serial;" "iostream = \\&Serial3;"
}

function check_rosserial_arduino_port() {
  grep "iostream = &Serial3;" "$HOME/Arduino/libraries/ros_lib/ArduinoHardware.h"
}

function abl() {
  rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries && patch_rosserial_arduino_port
}

function acm() {
  cb && abl && ac
}

export -f patch
export -f ac
export -f au
export -f acu
export -f patch_rosserial_arduino_port
export -f abl
export -f acm