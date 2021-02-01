"""Utilties for dealing with Local Mission Coordinates"""

from math import copysign, trunc

from utm import from_latlon


def decimal_degs_to_decimal_mins(x):
    degs = trunc(x)
    return trunc(x) * 100 + (x - degs) * 60


def decimal_mins_to_decimal_degs(x):
    degs = trunc(x / 100)
    minutes = x % copysign(100, x)
    return degs + (minutes / 60)


def zone_char_to_hemisphere(zone_char):
    """Given a zone character, return the hemisphere as 1 (North) or -1 (South).

    """
    if zone_char + ord('A') < ord('N'):
        return -1
    else:
        return 1


def zone_offset(zone_from, zone_to):
    diff = zone_to - zone_from

    if diff > 30:
        return diff - 60
    if diff < -30:
        return diff + 60
    else:
        return diff


def band_mid_latitude(zone_char):
    # I and O are not used (due to confusion with 1 and 0).
    if zone_char >= ord('O') - ord('A'):
        zone_char -= 1
    if zone_char >= ord('I') - ord('A'):
        zone_char -= 1

    # Compute number of bands from N:
    dist_from_N = zone_char - 12

    return 4 + dist_from_N * 8


def zone_width_at_band(zone_char):
    return (500000 - from_latlon(band_mid_latitude(zone_char), 0)[0]) * 2


def utm_to_lmc(easting, northing, zone_num, zone_char, x):
    """Convert a set of coordinates into the glider's LMC."""
    hemisphere = zone_char_to_hemisphere(zone_char)
    lmc_hemisphere = zone_char_to_hemisphere(x.x_lmc_utm_zone_char)

    if hemisphere != lmc_hemisphere:
        northing_correction = copysign(10000000, hemisphere)
    else:
        northing_correction = 0

    if zone_num != x.x_lmc_utm_zone_digit:
        easting_correction = (zone_offset(x.x_lmc_utm_zone_digit, zone_num)
                              * zone_width_at_band(zone_char))
    else:
        easting_correction = 0

    easting += easting_correction
    northing += northing_correction

    lmc_x = easting + x.x_utm_to_lmc_x0
    lmc_y = northing + x.x_utm_to_lmc_y0

    return (x.x_utm_to_lmc_00 * lmc_x + x.x_utm_to_lmc_01 * lmc_y,
            x.x_utm_to_lmc_10 * lmc_x + x.x_utm_to_lmc_11 * lmc_y)


def latlon_to_lmc(lat, lon, x):
    lat = decimal_mins_to_decimal_degs(lat)
    lon = decimal_mins_to_decimal_degs(lon)
    easting, northing, zone_num, zone_char = from_latlon(lat, lon)
    zone_char = ord(zone_char) - ord('A')
    return utm_to_lmc(easting, northing, zone_num, zone_char, x)


def set_lmc_origin(x):
    lat = decimal_mins_to_decimal_degs(x.m_lat)
    lon = decimal_mins_to_decimal_degs(x.m_lon)
    easting, northing, zone_num, zone_char = from_latlon(lat, lon)
    zone_char = ord(zone_char) - ord('A')

    x.x_lmc_utm_zone_digit = zone_num
    x.x_lmc_utm_zone_char = zone_char
    x.x_utm_to_lmc_x0 = -easting
    x.x_utm_to_lmc_y0 = -northing

    # TODO: This does not account for magnetic deviation!
    x.x_utm_to_lmc_00 = 1
    x.x_utm_to_lmc_01 = 0
    x.x_utm_to_lmc_10 = 0
    x.x_utm_to_lmc_11 = 1
