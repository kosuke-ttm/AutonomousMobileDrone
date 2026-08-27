"""Plot altitude, velocity, thrust, and altitude error vs time (REQ-NF-004)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from simulation.simulator import SimulationResult


def plot_result(result: SimulationResult, output_path: Path, title: str) -> None:
    times = result.column("time_s")
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(title)

    axes[0].plot(times, result.column("altitude_m"), label="altitude")
    axes[0].plot(times, result.column("target_altitude_m"), "--", label="target")
    axes[0].set_ylabel("Altitude [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, result.column("velocity_mps"))
    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, result.column("thrust_n"))
    axes[2].set_ylabel("Thrust [N]")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(times, result.column("altitude_error_m"))
    axes[3].set_ylabel("Altitude error [m]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=120)
    plt.close(fig)
