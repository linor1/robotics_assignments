# !/bin/bash 

echo "Compiling everything under krembot_ws…" ; echo
mkdir -p build && cd build
cmake ..
make
echo "Done." ; echo
