import argparse
import sys
import os

import numpy as np

from .runner import online


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    # "online" subcommand
    parser_online = subparsers.add_parser(
        "online", help="Run UWBINS viewer in online mode."
    )
    parser_online.add_argument(
        "config", type=str, help="Path to the configuration file."
    )
    parser_online.set_defaults(handler=online)

    return parser


def main() -> None:
    parser = create_parser()
    args = parser.parse_args()
    if len(sys.argv) == 1:
        parser.print_usage()
        sys.exit(-1)
    if hasattr(args, "handler"):
        args.handler(args)


if __name__ == "__main__":
    main()