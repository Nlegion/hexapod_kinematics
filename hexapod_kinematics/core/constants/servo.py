"""MG996R servo dimensions from datasheet DOC017151299.pdf."""

# Body envelope (mm): length × width × height
MG996R_BODY_LENGTH_MM = 40.7
MG996R_BODY_WIDTH_MM = 19.7
MG996R_BODY_HEIGHT_MM = 42.9

# Axis length for LCS_out synthesis = height along shaft / Z
MG996R_AXIS_LENGTH_MM = MG996R_BODY_HEIGHT_MM
MG996R_AXIS_LENGTH_MIN_MM = 5.0
MG996R_AXIS_LENGTH_MAX_MM = 80.0

DATASHEET_REF = "DOC017151299.pdf"
