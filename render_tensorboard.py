"""Export scalar charts from TensorBoard event files as PNG images."""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("event_file", type=Path, nargs="?", default=Path("results/ppo_hockey_1m.tfevents"))
    parser.add_argument("--output", type=Path, default=Path("results/figures"))
    args = parser.parse_args()
    acc = EventAccumulator(str(args.event_file), size_guidance={"scalars": 0})
    acc.Reload()
    tags = acc.Tags().get("scalars", [])
    if not tags:
        raise SystemExit(f"No scalar summaries found in {args.event_file}")
    args.output.mkdir(parents=True, exist_ok=True)
    for tag in tags:
        values = acc.Scalars(tag)
        if not values:
            continue
        fig, ax = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
        ax.plot([v.step for v in values], [v.value for v in values], linewidth=1.6)
        ax.set(title=tag.replace("_", " "), xlabel="Environment steps", ylabel=tag.rsplit("/", 1)[-1])
        ax.grid(alpha=0.25)
        name = "".join(c.lower() if c.isalnum() else "_" for c in tag).strip("_")
        fig.savefig(args.output / f"{name}.png", dpi=160)
        plt.close(fig)
    print("Scalar tags:", ", ".join(tags))


if __name__ == "__main__":
    main()
