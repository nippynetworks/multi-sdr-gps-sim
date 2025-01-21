#ifndef EPHEMERIS_H
#define EPHEMERIS_H

#include "gps.h"

int readRinexVersion(const char *fname);
int readRinex2(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, char rinex_date[21], const char *fname);
int readRinex3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, char rinex_date[21], const char *fname);

#endif // EPHEMERIS_H