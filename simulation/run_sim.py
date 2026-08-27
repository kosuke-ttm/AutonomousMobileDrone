#!/usr/bin/env python3
"""Command-line entry for eVTOL simulation (REQ-NF-002)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build"))

from simulation.scenario import all_scenarios, run_scenario  # noqa: E402
from simulation.visualization import plot_result  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Run eVTOL altitude-control scenarios")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "results",
        help="CSV and plot output directory",
    )
    args = parser.parse_args()

    for scenario in all_scenarios():
        result = run_scenario(scenario)
        csv_path = args.output_dir / f"{scenario.name}.csv"
        png_path = args.output_dir / f"{scenario.name}.png"
        result.write_csv(csv_path)
        plot_result(result, png_path, scenario.name)
        sse = result.steady_state_error()
        print(
            f"{scenario.name}: final_alt={result.final_altitude():.2f} m "
            f"SSE={sse:.3f} m overshoot={result.overshoot(scenario.target_fn(scenario.duration_s)):.2f} m "
            f"csv={csv_path}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
