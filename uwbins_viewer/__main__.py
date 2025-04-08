import argparse
import sys
import os

import numpy as np

from .runner import online, offline


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

    # "online" subcommand
    parser_offline = subparsers.add_parser(
        "offline", help="Run UWBINS viewer in offline mode."
    )
    parser_offline.add_argument(
        "config", type=str, help="Path to the configuration file."
    )
    # parser_offline.add_argument(
    #     "dataset", type=str, help="Path to the dataset directory."
    # )
    parser_offline.set_defaults(handler=offline)

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