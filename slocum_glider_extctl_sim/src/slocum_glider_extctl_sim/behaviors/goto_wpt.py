from math import atan2, pi, sqrt

from .behavior import Behavior, maybe_deref
from ..lmc import latlon_to_lmc, utm_to_lmc
from ..modes import HEADING_MODE_HEADING


WPT_UNITS_LMC = 0
WPT_UNITS_UTM = 1
WPT_UNITS_LAT_LONG = 2


class GotoWptBehavior(Behavior):
    NAME = 'goto_wpt'

    def distance_from_waypoint(self, x):
        wpt_units = self.args.wpt_units
        wpt_x = maybe_deref(self.args.wpt_x, x)
        wpt_y = maybe_deref(self.args.wpt_y, x)

        if wpt_units == WPT_UNITS_LMC:
            c_wpt_x_lmc = wpt_x
            c_wpt_y_lmc = wpt_y
        elif wpt_units == WPT_UNITS_UTM:
            c_wpt_x_lmc, c_wpt_y_lmc = utm_to_lmc(
                wpt_x, wpt_y,
                maybe_deref(self.args.utm_zd, x),
                maybe_deref(self.args.utm_zc, x),
                x
            )
        elif wpt_units == WPT_UNITS_LAT_LONG:
            c_wpt_x_lmc, c_wpt_y_lmc = latlon_to_lmc(wpt_y, wpt_x, x)
        else:
            raise ValueError('Unknown value for wpt_units: ' + str(wpt_units))

        return sqrt((c_wpt_x_lmc - x.m_x_lmc)**2
                    + (c_wpt_y_lmc - x.m_y_lmc)**2)

    # TODO: Figure out when the actual glider hardware sets (and clears
    # x_hit_a_waypoint)
    def should_stop(self, x):
        result = super(GotoWptBehavior, self).should_stop(x)
        if result:
            x.x_hit_a_waypoint = True
            x.x_last_wpt_x_lmc = x.c_wpt_x_lmc
            x.x_last_wpt_y_lmc = x.c_wpt_y_lmc
        return result

    def compute_controls(self, x):
        wpt_units = self.args.wpt_units
        wpt_x = maybe_deref(self.args.wpt_x, x)
        wpt_y = maybe_deref(self.args.wpt_y, x)

        if wpt_units == WPT_UNITS_LMC:
            c_wpt_x_lmc = wpt_x
            c_wpt_y_lmc = wpt_y
        elif wpt_units == WPT_UNITS_UTM:
            c_wpt_x_lmc, c_wpt_y_lmc = utm_to_lmc(
                wpt_x, wpt_y,
                maybe_deref(self.args.utm_zd, x),
                maybe_deref(self.args.utm_zc, x),
                x
            )
        elif wpt_units == WPT_UNITS_LAT_LONG:
            x.c_wpt_lat = wpt_y
            x.c_wpt_lon = wpt_x
            c_wpt_x_lmc, c_wpt_y_lmc = latlon_to_lmc(wpt_y, wpt_x, x)
        else:
            raise ValueError('Unknown value for wpt_units: ' + str(wpt_units))

        x.c_wpt_x_lmc = c_wpt_x_lmc
        x.c_wpt_y_lmc = c_wpt_y_lmc

        if x.x_hit_a_waypoint \
           and (c_wpt_x_lmc != x.x_last_wpt_x_lmc
                or c_wpt_y_lmc != x.x_last_wpt_y_lmc):
            x.x_hit_a_waypoint = False

        # Heading is measured from north, so that's why we have x and y
        # "reversed" here.
        target_heading = atan2(c_wpt_x_lmc - x.m_x_lmc,
                               c_wpt_y_lmc - x.m_y_lmc)
        # Atan2 gives us [-pi. pi], we need [0, 2pi]
        if target_heading < 0:
            target_heading += 2 * pi

        x.m_dist_to_wpt = sqrt((c_wpt_x_lmc - x.m_x_lmc)**2
                               + (c_wpt_y_lmc - x.m_y_lmc)**2)
        x.cc_heading_mode = HEADING_MODE_HEADING
        x.cc_heading_value = target_heading
