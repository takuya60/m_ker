"""Bundled command-line bridge for pymcuprog and esptool."""

import sys


def run_pymcuprog(arguments):
    from pymcuprog import pymcuprog

    sys.argv = ["pymcuprog", *arguments]
    return pymcuprog.main()


def run_esptool(arguments):
    import esptool

    sys.argv = ["esptool", *arguments]
    try:
        return esptool.main(arguments)
    except TypeError:
        return esptool.main()


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("pymcuprog", "esptool"):
        print("Usage: ker_flash_cli.exe <pymcuprog|esptool> [arguments...]", file=sys.stderr)
        return 2

    tool = sys.argv[1]
    arguments = sys.argv[2:]
    result = run_pymcuprog(arguments) if tool == "pymcuprog" else run_esptool(arguments)
    return result if isinstance(result, int) else 0


if __name__ == "__main__":
    raise SystemExit(main())
