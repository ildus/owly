# owly

Guitar Effects Pedal based on STM32 and WM8731 audio codec.
This is an early stage of device, I use custom PCB boards for WM8731
and filter stages, with STM32F4Discovery.

## Getting started

### Update libopencm3 library

```
git submodule init
git submodule update
```

### Compile

```
./build.sh
cd build
make libopencm3
make
```

### Upload to STM32F4Discovery

```
make upload
```
