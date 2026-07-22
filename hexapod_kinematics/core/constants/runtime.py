"""Default runtime and path constants."""

from pathlib import Path

TOOL_NAME = "stl_reader kinematics extractor"
TOOL_VERSION = "0.1.0"

DEFAULT_KOMPAS_BIN = Path(r"C:\Program Files\ASCON\KOMPAS-3D v21\Bin")
KOMPAS_PROGID = "KOMPAS.Application.7"

DEFAULT_CAD_ROOT = Path(
    r"C:\Users\npara\OneDrive\Desktop\spider_body\996"
)

COM_CONNECT_RETRIES = 2
COM_RETRY_DELAY_SEC = 1.0
OPEN_RETRIES = 2

DUPLICATE_LCS_POLICY = "first_with_warning"
