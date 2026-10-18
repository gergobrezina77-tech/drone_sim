"""Engine benchmark: compile, restart and per-tick cost for growing fleets.

poetry run python scripts/bench.py [N ...] [--seconds S] [--profile N]
"""

import argparse
import cProfile
import pstats

from drone_sim.engine.benchmark import measure


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("sizes", nargs="*", type=int, default=[10, 100, 500])
    p.add_argument("--seconds", type=float, default=3.0, help="simulated seconds per size")
    p.add_argument("--profile", type=int, metavar="N", help="profile the N-drone case instead")
    a = p.parse_args()

    if a.profile:
        profiler = cProfile.Profile()
        profiler.runcall(measure, a.profile, a.seconds)
        pstats.Stats(profiler).sort_stats("tottime").print_stats(15)
        return

    print(f"{'drones':>7} {'compile ms':>11} {'restart ms':>11} {'tick ms':>9} {'x real time':>12}")
    for n in a.sizes:
        r = measure(n, a.seconds)
        print(f"{r.n:>7} {r.compile_ms:>11.1f} {r.restart_ms:>11.2f} {r.tick_ms:>9.3f} {r.realtime_factor:>12.2f}")


if __name__ == "__main__":
    main()
