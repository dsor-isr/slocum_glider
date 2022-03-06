from math import copysign, trunc


def decimal_degs_to_decimal_mins(x):
    degs = trunc(x)
    return trunc(x) * 100 + (x - degs) * 60


def decimal_mins_to_decimal_degs(x):
    degs = trunc(x / 100)
    minutes = x % copysign(100, x)
    return degs + (minutes / 60)
