#!/bin/bash
# build -a IA32 -p CorebootPayloadPkg/CorebootPayloadPkgIa32.dsc -b DEBUG -t GCC5
build -a IA32 -a X64 -p CorebootPayloadPkg/CorebootPayloadPkgIa32X64.dsc -b DEBUG -t GCC5
cp Build/CorebootPayloadPkgX64/DEBUG_GCC5/FV/UEFIPAYLOAD.fd ../apu2_fw_rel/apu2/coreboot/
cd ../apu2_fw_rel
./apu2/apu2-documentation/scripts/apu2_fw_rel.sh build-ml
read -n 1 -s -p "Press any key to continue with flashing recent build ..."
./apu2/apu2-documentation/scripts/apu2_fw_rel.sh flash-ml root@$1
cd ../edk2
