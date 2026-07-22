"""Default LCS name aliases and mount naming convention."""

# Raw LCS name (case-sensitive as in KOMPAS) -> role
DEFAULT_LCS_ROLE_MAP: dict[str, str] = {
    "LCS_in": "in",
    "LCS_": "in",
    "in": "in",
    "LCS_out": "out",
    "LCS__": "out",
    "out": "out",
    "mount_leg_1": "mount_leg_1",
    "mount_leg_2": "mount_leg_2",
    "mount_leg_3": "mount_leg_3",
    "mount_leg_4": "mount_leg_4",
    "mount_leg_5": "mount_leg_5",
    "mount_leg_6": "mount_leg_6",
}

# LCS name -> hexapod LegID 0..5
DEFAULT_LEG_MOUNT_MAP: dict[str, int] = {
    "mount_leg_1": 0,
    "mount_leg_2": 1,
    "mount_leg_3": 2,
    "mount_leg_4": 3,
    "mount_leg_5": 4,
    "mount_leg_6": 5,
}

ROLE_IN = "in"
ROLE_OUT = "out"
