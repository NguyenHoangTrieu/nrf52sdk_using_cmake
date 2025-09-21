cd examples/tinyML_test
rmdir /s /q build
mkdir build
cd build
cmake .. -G "Ninja"
ninja
nrfjprog --program nrf52840_xxaa.hex --chiperase --verify --reset -f nrf52