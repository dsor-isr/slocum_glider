from .behavior import Behavior, maybe_deref
from ..lmc import latlon_to_lmc, utm_to_lmc


WPT_UNITS_LMC = 0
WPT_UNITS_UTM = 1
WPT_UNITS_LAT_LONG = 2


class GotoWptBehavior(Behavior):
    NAME = 'goto_wpt'
    CONTROLS = ['heading']

    def init(self, g):
        wpt_units = self.args.wpt_units
        wpt_x = maybe_deref(self.args.wpt_x, g)
        wpt_y = maybe_deref(self.args.wpt_y, g)
        if wpt_units == WPT_UNITS_LMC:
            wpt_x_lmc = wpt_x
            wpt_y_lmc = wpt_y
        elif wpt_units == WPT_UNITS_UTM:
            wpt_x_lmc, wpt_y_lmc = utm_to_lmc(
                wpt_x, wpt_y,
                maybe_deref(self.args.utm_zd, g),
                maybe_deref(self.args.utm_zc, g),
                g
            )
        elif wpt_units == WPT_UNITS_LAT_LONG:
            wpt_x_lmc, wpt_y_lmc = latlon_to_lmc(wpt_y, wpt_x, g)
        else:
            raise ValueError('Unknown value for wpt_units: ' + str(wpt_units))

        self.wpt_x_lmc = wpt_x_lmc
        self.wpt_y_lmc = wpt_y_lmc

    def compute_controls(self, u, x):
        wpt_units = self.args.wpt_units
        wpt_x = maybe_deref(self.args.wpt_x, x)
        wpt_y = maybe_deref(self.args.wpt_y, x)

        if wpt_units == WPT_UNITS_LMC:
            u.c_wpt_x_lmc = wpt_x
            u.c_wpt_y_lmc = wpt_y
        elif wpt_units == WPT_UNITS_UTM:
            u.c_wpt_x_lmc, u.c_wpt_y_lmc = utm_to_lmc(
                wpt_x, wpt_y,
                maybe_deref(self.args.utm_zd, x),
                maybe_deref(self.args.utm_zc, x),
                x
            )
        elif wpt_units == WPT_UNITS_LAT_LONG:
            u.c_wpt_x_lmc, u.c_wpt_y_lmc = latlon_to_lmc(wpt_y, wpt_x, x)
        else:
            raise ValueError('Unknown value for wpt_units: ' + str(wpt_units))
