import argparse
from pathlib import Path


def generate(header_dir: Path, source_dir: Path) -> None:
    pass


def main() -> None:
    parser = argparse.ArgumentParser(description="generate FC autogen C source and headers.")
    parser.add_argument("--header-dir", type=Path, required=True)
    parser.add_argument("--source-dir", type=Path, required=True)
    args = parser.parse_args()

    args.header_dir.mkdir(parents=True, exist_ok=True)
    args.source_dir.mkdir(parents=True, exist_ok=True)

    generate(header_dir=args.header_dir, source_dir=args.source_dir)


if __name__ == "__main__":
    main()
