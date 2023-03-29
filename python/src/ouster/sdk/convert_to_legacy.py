"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

import argparse

from ouster import client


def main() -> None:
    descr = """Convert non-legacy metadata to legacy format"""

    parser = argparse.ArgumentParser(description=descr)
    parser.add_argument('metadata_path')
    parser.add_argument('-o', '--output-path')

    args = parser.parse_args()

    with open(args.metadata_path) as json:
        print(f"Reading metadata from: {args.metadata_path}")
        legacy = client.convert_to_legacy(json.read())

    if args.output_path:
        with open(args.output_path, "w") as output:
            print(f"Writing converted legacy metadata to: {args.output_path}")
            output.write(legacy)
    else:
        print(legacy)
