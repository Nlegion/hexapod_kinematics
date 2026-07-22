"""Foot tip pad RF-F12050 (self-adhesive instrument foot)."""

# RF-F12050: 12 × 5 mm silicone pad
FOOT_PAD_PART = "RF-F12050"
FOOT_PAD_DIAMETER_MM = 12.0
FOOT_PAD_HEIGHT_MM = 5.0
# Recessed into tibia tip
FOOT_PAD_RECESS_MM = 3.0
# Protrusion beyond printed tip (= height - recess) → body stands higher
FOOT_PAD_PROTRUSION_MM = FOOT_PAD_HEIGHT_MM - FOOT_PAD_RECESS_MM  # 2.0
