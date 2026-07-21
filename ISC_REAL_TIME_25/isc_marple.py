"""
isc_marple.py
Módulo para gestionar la subida de archivos CSV a Marple Data.
Usa la API correcta del SDK v3: db.get_stream() → stream.push_file() → wait_for_import()
"""
import os
import logging
from pathlib import Path
from marple import DB

# ================= CONFIGURACIÓN =================
def get_secret(key: str, default: str) -> str:
    # 1. Try environment variable
    val = os.environ.get(key)
    if val:
        return val
    # 2. Try local .env file in workspace root
    env_path = Path(__file__).resolve().parent / ".env"
    if env_path.exists():
        try:
            with open(env_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        k, v = line.split("=", 1)
                        if k.strip() == key:
                            return v.strip().strip("'").strip('"')
        except Exception:
            pass
    # 3. Fallback to default
    return default

MARPLE_API_TOKEN = get_secret("MARPLE_API_TOKEN", "mdb_Le69BDaNdgn1SJ4DqWX6N6btH-a8Dx8Ou96aBbLA4v8")


# Nombre del stream en Marple (debe existir en tu workspace)
DATASTREAM_NAME  = "ISC_Telemetry"

# Tiempo máximo de espera para que Marple procese el archivo (segundos)
IMPORT_TIMEOUT_S = 120
# =================================================

logger = logging.getLogger("ISC_MARPLE")


def upload_session_csv(file_path: str, metadata: dict) -> None:
    """
    Sube un archivo CSV a Marple Data usando el SDK de marple.
    """
    if not os.path.exists(file_path):
        logger.error(f"Archivo no encontrado: {file_path}")
        return

    try:
        logger.info("Conectando a Marple DB…")
        db = DB(MARPLE_API_TOKEN)

        logger.info(f"Subiendo archivo '{os.path.basename(file_path)}' al stream '{DATASTREAM_NAME}'…")
        dataset_id = db.push_file(DATASTREAM_NAME, file_path, metadata=metadata or {})

        logger.info(f"¡Subida completada! ID de Dataset en Marple: {dataset_id}")

    except Exception as e:
        logger.error(f"Error crítico subiendo a Marple: {e}")