/**
 * Time functions
 *
 * Copyright © 2025 Nippy Networks
 * Distributed under the MIT License.
 *
 */

#include "time.h"

#include <math.h>

double subGpsTime(gpstime_t g1, gpstime_t g0) {
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double) (g1.week - g0.week) * SECONDS_IN_WEEK;

    return (dt);
}

gpstime_t incGpsTime(gpstime_t g0, double dt) {
    gpstime_t g1;

    g1.week = g0.week;
    g1.sec = g0.sec + dt;

    // g1.sec = round(g1.sec * 1000.0) / 1000.0; // Avoid rounding error

    while (g1.sec >= SECONDS_IN_WEEK) {
        g1.sec -= SECONDS_IN_WEEK;
        g1.week++;
    }

    while (g1.sec < 0.0) {
        g1.sec += SECONDS_IN_WEEK;
        g1.week--;
    }

    return (g1);
}

/* Convert a UTC date into a GPS date
 * t input date in UTC form
 * g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g) {
    int leap_seconds = 18;

    int doy[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int ye;
    int de;
    int lpdays;

    ye = t->y - 1980;

    // Compute the number of leap days since Jan 5/Jan 6, 1980.
    lpdays = ye / 4 + 1;
    if ((ye % 4) == 0 && t->m <= 2)
        lpdays--;

    // Compute the number of days elapsed since Jan 5/Jan 6, 1980.
    de = ye * 365 + doy[t->m - 1] + t->d + lpdays - 6;

    // Convert time to GPS weeks and seconds.
    g->week = de / 7;
    g->sec = (double) (de % 7) * SECONDS_IN_DAY
            + t->hh * SECONDS_IN_HOUR
            + t->mm * SECONDS_IN_MINUTE
            + t->sec;

    *g = incGpsTime(*g, (double)leap_seconds);
    return;
}

void gps2date(const gpstime_t *g, datetime_t *t) {
    int leap_seconds = 18;
    gpstime_t g_utc = incGpsTime(*g, (double)-leap_seconds);

    // Convert Julian day number to calendar date
    int c = (int) (7 * g_utc.week + floor(g_utc.sec / 86400.0) + 2444245.0) + 1537;
    int d = (int) ((c - 122.1) / 365.25);
    int e = 365 * d + d / 4;
    int f = (int) ((c - e) / 30.6001);

    t->d = c - e - (int) (30.6001 * f);
    t->m = f - 1 - 12 * (f / 14);
    t->y = d - 4715 - ((7 + t->m) / 10);

    t->hh = ((int) (g_utc.sec / 3600.0)) % 24;
    t->mm = ((int) (g_utc.sec / 60.0)) % 60;
    t->sec = g_utc.sec - 60.0 * floor(g_utc.sec / 60.0);

    return;
}