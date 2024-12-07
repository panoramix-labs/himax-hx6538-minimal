set -eux

rm -rf obj_epii_evb_icv30_bdv10

mkdir -p build/ obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/clib/

arm-zephyr-eabi-gcc -c -g -O2 \
  -mthumb \
  -mcpu=cortex-m55 \
  -mfloat-abi=hard \
  -mcmse \
  -ffunction-sections \
  -fdata-sections \
  -fstack-usage \
  -flax-vector-conversions \
  -Wall \
  -I./include \
  main.c -o build/main.o

arm-zephyr-eabi-gcc \
  -Wl,--gc-sections \
  -Wl,-print-memory-usage \
  -Wl,--sort-section=alignment \
  -Wl,--cref \
  -Wl,--cmse-implib \
  -Wl,-M,-Map=bare-metal-firmware.map \
  -mthumb \
  -mcpu=cortex-m55 \
  -mfloat-abi=hard \
  -specs=nano.specs \
  -Tlinker.ld \
  build/main.o -lm -lc_nano -lgcc \
  -o build/bare-metal-firmware.elf

cp build/bare-metal-firmware.elf \
  ../we2_image_gen_local/input_case1_secboot/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf

cd ../we2_image_gen_local/

./we2_local_image_gen project_case1_blp_wlcsp.json
