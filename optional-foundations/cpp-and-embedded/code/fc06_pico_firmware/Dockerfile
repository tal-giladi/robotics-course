# FC.06 - a reproducible Pico SDK cross-compilation toolchain.
#
# Build it once (needs the network, takes a few minutes):
#   docker build -t fc-picosdk:2.3.1 optional-foundations/cpp-and-embedded/code/fc06_pico_firmware
#
# Then build firmware with no network at all:
#   docker run --rm --network none --name fc-pico06 \
#     -v "$PWD/optional-foundations/cpp-and-embedded/code/fc06_pico_firmware:/src:ro" -w /work \
#     fc-picosdk:2.3.1 bash -c 'cp -r /src /work/fw && cd /work/fw &&
#       cmake -S . -B build -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=RelWithDebInfo &&
#       cmake --build build -j && arm-none-eabi-size build/fc06_firmware.elf'
#
# Version-sensitive: pin PICO_SDK_TAG. Check https://github.com/raspberrypi/pico-sdk/releases
# for the current release before changing it.

FROM ubuntu:24.04

ARG PICO_SDK_TAG=2.3.1
ENV DEBIAN_FRONTEND=noninteractive
ENV PICO_SDK_PATH=/opt/pico-sdk

RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake ninja-build build-essential \
      gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
      git ca-certificates python3 \
  && rm -rf /var/lib/apt/lists/*

# --recurse-submodules brings in TinyUSB, which pico_enable_stdio_usb needs.
RUN git clone --branch ${PICO_SDK_TAG} --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_PATH}

WORKDIR /work
CMD ["bash"]
