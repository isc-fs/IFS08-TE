"""
isc_marple.py
Módulo para gestionar la subida de archivos CSV a Marple Data.
Usa argumentos posicionales para máxima compatibilidad con el SDK 'marple'.
"""
import os
import logging
from marple import DB 

# ================= CONFIGURACIÓN =================
# >>> PEGA TU API TOKEN DE MARPLE AQUÍ <<<
MARPLE_API_TOKEN = "mdb_PDw2ak22qKs2FHmEXnbBC7ZKWoC8FadttcdxEhxH_zk" 

# Nombre de la fuente de datos en Marple
DATASTREAM_NAME = "ISC_Telemetry" 
# =================================================

logger = logging.getLogger("ISC_MARPLE")

def upload_session_csv(file_path: str, metadata: dict):
    """
    Sube un archivo CSV local a la plataforma Marple.
    """
    

    if not os.path.exists(file_path):
        logger.error(f"Archivo no encontrado: {file_path}")
        return

    try:
        logger.info(f"Conectando a Marple DB...")
        
        # 1. Inicializar la conexión
        db = DB(MARPLE_API_TOKEN)
        
        logger.info(f"Subiendo archivo: {os.path.basename(file_path)}...")
        
        # 2. Subida del archivo (USANDO ARGUMENTOS POSICIONALES)
        # Sintaxis: db.push_file(NOMBRE_FUENTE, RUTA_ARCHIVO, metadata=...)
        dataset_id = db.push_file(
            DATASTREAM_NAME,  # <--- Primer argumento sin nombre (Posicional)
            file_path,        # <--- Segundo argumento (Ruta del archivo)
            metadata=metadata # <--- Metadatos (Opcional, keyword argument)
        )
        
        logger.info(f"¡Subida completada con éxito! Dataset ID: {dataset_id}")
        
        # Opcional: Imprimir estado/enlace
        try:
            link = f"https://app.marpledata.com/link/to/dataset/{dataset_id}"
            logger.info(f"Ver en Marple: {link}")
        except:
            pass
        
    except Exception as e:
        logger.error(f"Error crítico subiendo a Marple: {e}")