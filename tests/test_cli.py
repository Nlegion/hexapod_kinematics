"""CLI argument parsing tests (no KOMPAS)."""

from hexapod_kinematics.presentation.cli import build_parser


def test_extract_flags_default_both() -> None:
    parser = build_parser()
    args = parser.parse_args(["extract", "--export-dir", "out"])
    assert args.command == "extract"
    assert args.json is False
    assert args.header is False
    assert args.gabarit is False


def test_simulate_modes_in_parser() -> None:
    parser = build_parser()
    args = parser.parse_args(
        ["simulate", "--mode", "pulse_aligned", "--body-height", "50"]
    )
    assert args.mode == "pulse_aligned"
    assert args.body_height == 50.0
    args2 = parser.parse_args(["visualize", "--json", "x.json"])
    assert args2.command == "visualize"
    args3 = parser.parse_args(["sync-hexapod", "--config-h", "Config.h"])
    assert args3.command == "sync-hexapod"
