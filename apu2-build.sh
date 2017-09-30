#!/bin/bash
# build -a IA32 -p CorebootPayloadPkg/CorebootPayloadPkgIa32.dsc -b DEBUG -t GCC5
build -a IA32 -a X64 -p CorebootPayloadPkg/CorebootPayloadPkgIa32X64.dsc -b DEBUG -t GCC5
cp Build/CorebootPayloadPkgX64/DEBUG_GCC5/FV/UEFIPAYLOAD.fd ../apu2_fw_rel/apu2/coreboot/
cd ../apu2_fw_rel
./apu2/apu2-documentation/scripts/apu2_fw_rel.sh build-ml
cd ../apu-test-suite
source ~/storage/robot-venv/bin/activate
robot -v rom_path:$PWD/../apu2_fw_rel/apu2/coreboot/build/ -v host:$1 -v username:root -v password:voyage flash_coreboot_rom.robot
deactivate
cd ../edk2
