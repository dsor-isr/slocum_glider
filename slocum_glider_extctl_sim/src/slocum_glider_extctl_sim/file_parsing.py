"""Helpers for reading lines and values from .mi and .ma files."""


def parse_value(units, string_value):
    if units == "bool":
        # Ugh. It would be nice if we could truly use bools. However, many
        # sensors and b_args have a type of bool but are *really* enums.
        return int(float(string_value))
    elif units == "byte" or units == "enum":
        # Sometimes integers are written in the masterdata file with a ".0" at
        # the end. This make python not complain about that.
        return int(float(string_value))
    else:
        return float(string_value)


def split_line(line):
    return line.strip().split("#")[0].split()
