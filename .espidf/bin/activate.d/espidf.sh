#!/bin/bash
IDF_PATH="$VIRTUAL_ENV/../solaris-v1/external-esp-idf"

# Ejecutar instalación solo la primera vez
if [ ! -f "$VIRTUAL_ENV/.idf_installed" ]; then
    echo "Instalando ESP-IDF en el venv..."
    $VIRTUAL_ENV/bin/python3 $IDF_PATH/install.sh
    touch $VIRTUAL_ENV/.idf_installed
fi

# Siempre exportar entorno de ESP-IDF
echo "Activando ESP-IDF..."
source $IDF_PATH/export.sh
