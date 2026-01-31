from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="Rebuild manifest from output root")
    parser.add_argument("--output", required=True)
    _ = parser.parse_args()
    raise SystemExit("Manifest builder skeleton created; implementation pending.")


if __name__ == "__main__":
    main()
