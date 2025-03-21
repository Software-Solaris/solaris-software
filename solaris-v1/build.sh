#!/bin/bash

# Activar entorno ESP-IDF si hace falta
source $HOME/esp/esp-idf/export.sh

# Compilar el proyecto
idf.py build

# Generar el firmware combinado
esptool.py --chip esp32s3 merge_bin -o build/solaris-v1_all_in_one.bin \
  --flash_mode dio \
  --flash_freq 40m \
  --flash_size 4MB \
  0x1000 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/solaris-v1.bin

echo "✅ Firmware combinado generado como build/solaris-v1_all_in_one.bin"
