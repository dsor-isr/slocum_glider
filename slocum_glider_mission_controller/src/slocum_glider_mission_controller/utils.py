from math import copysign, sqrt, trunc

from utm import from_latlon


def decimal_degs_to_decimal_mins(x):
    degs = trunc(x)
    return trunc(x) * 100 + (x - degs) * 60


def decimal_mins_to_decimal_degs(x):
    degs = trunc(x / 100)
    minutes = x % copysign(100, x)
    return degs + (minutes / 60)


# TODO: make this more robust to different zones. See lmc.py in
# slocum_glider_extctl_sim for ideas on how to do this.
def glider_lon_lat_dist(lon1, lat1, lon2, lat2):
    lon1 = decimal_mins_to_decimal_degs(lon1)
    lat1 = decimal_mins_to_decimal_degs(lat1)
    lon2 = decimal_mins_to_decimal_degs(lon2)
    lat2 = decimal_mins_to_decimal_degs(lat2)

    easting1, northing1, zone_num1, zone_char1 = from_latlon(lat1, lon1)
    easting2, northing2, zone_num2, zone_char2 = from_latlon(lat2, lon2)

    if zone_num1 != zone_num2:
        return 5000
    if zone_char1 != zone_char2:
        return 5000

    return sqrt((easting1 - easting2) ** 2 + (northing1 - northing2) ** 2)
