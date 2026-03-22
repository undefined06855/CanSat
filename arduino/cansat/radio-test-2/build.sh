arduino-cli compile --fqbn arduino:avr:uno --build-property compiler.cpp.extra_flags="-std=gnu++17" ./
arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyACM0 ./
# cd ~/projects/CanSat/laptop/cansat-hub/
# go run .
