from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a debug case by scene token")
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene_token", required=True)
    _ = parser.parse_args()
    raise SystemExit("Debug exporter skeleton created; implementation pending.")


if __name__ == "__main__":
    main()
