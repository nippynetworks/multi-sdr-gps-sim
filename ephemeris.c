#include "ephemeris.h"
#include "time.h"
#include "gps.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

/*
#include <stdio.h>
*/

/* Replace all 'E' exponential designators to 'D'
 * str String in which all occurrences of 'E' are replaced with *  'D'
 * len Length of input string in bytes
 * Number of characters replaced
 */
static int replaceExpDesignator(char *str, int len) {
    int i, n = 0;

    for (i = 0; i < len; i++) {
        if (str[i] == 0) {
            break;
        }

        if (str[i] == 'D' || str[i] == 'd') {
            n++;
            str[i] = 'E';
        }
    }

    return (n);
}

int readRinexVersion(const char *fname) {
    struct gzFile_s *fp;

    char str[MAX_CHAR];
    char tmp[20];
    double ver = 0.0;
    int version = 0;

    if (NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    // Read header lines
    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        if (strncmp(str + 60, "COMMENT", 7) == 0) {
            continue;
        } else if (strncmp(str + 60, "END OF HEADER", 13) == 0) {
            break;
        } else if (strncmp(str + 60, "RINEX VERSION / TYPE", 20) == 0) {
            strncpy(tmp, str, 9);
            tmp[9] = 0;
            replaceExpDesignator(tmp, 9);
            ver = atof(tmp);
            if ((ver >= 2.0) && (ver < 3.0) && (str[20] == 'N')) {
                version = 2;
                break;
            }

            if ((ver >= 3.0) && (ver < 4.0) && (str[20] == 'N') && (str[40] == 'G')) {
                version = 3;
                break;
            }
        }
    }
    gzclose(fp);
    return version;
}

/* Read Ephemeris data from the RINEX v2 Navigation file
 * eph Array of Output SV ephemeris data
 * fname File name of the RINEX file
 * Number of sets of ephemerides in the file
 */
int readRinex2(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, char rinex_date[21], const char *fname) {
    struct gzFile_s *fp;
    int ieph;

    int sv;
    char str[MAX_CHAR];
    char tmp[20];
    double ver = 0.0;

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    if (NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    // Clear valid flag
    for (ieph = 0; ieph < EPHEM_ARRAY_SIZE; ieph++)
        for (sv = 0; sv < MAX_SAT; sv++)
            eph[ieph][sv].vflg = false;

    // Read header lines
    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        if (strncmp(str + 60, "COMMENT", 7) == 0) {
            continue;
        } else if (strncmp(str + 60, "END OF HEADER", 13) == 0) {
            break;
        } else if (strncmp(str + 60, "RINEX VERSION / TYPE", 20) == 0) {
            strncpy(tmp, str, 9);
            tmp[9] = 0;
            replaceExpDesignator(tmp, 9);
            ver = atof(tmp);
            if (ver > 3.0) {
                gzclose(fp);
                return -2;
            }

            if (str[20] != 'N') {
                gzclose(fp);
                return -3;
            }
        } else if (strncmp(str + 60, "PGM / RUN BY / DATE", 19) == 0) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (strncmp(str + 60, "ION ALPHA", 9) == 0) {
            strncpy(tmp, str + 2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha0 = atof(tmp);

            strncpy(tmp, str + 14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha1 = atof(tmp);

            strncpy(tmp, str + 26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha2 = atof(tmp);

            strncpy(tmp, str + 38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha3 = atof(tmp);

            flags |= 0x1;
        } else if (strncmp(str + 60, "ION BETA", 8) == 0) {
            strncpy(tmp, str + 2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta0 = atof(tmp);

            strncpy(tmp, str + 14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta1 = atof(tmp);

            strncpy(tmp, str + 26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta2 = atof(tmp);

            strncpy(tmp, str + 38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta3 = atof(tmp);

            flags |= 0x1 << 1;
        } else if (strncmp(str + 60, "DELTA-UTC", 9) == 0) {
            strncpy(tmp, str + 3, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A0 = atof(tmp);

            strncpy(tmp, str + 22, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A1 = atof(tmp);

            strncpy(tmp, str + 41, 9);
            tmp[9] = 0;
            ionoutc->tot = atoi(tmp);

            strncpy(tmp, str + 50, 9);
            tmp[9] = 0;
            ionoutc->wnt = atoi(tmp);

            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (strncmp(str + 60, "LEAP SECONDS", 12) == 0) {
            strncpy(tmp, str, 6);
            tmp[6] = 0;
            ionoutc->dtls = atoi(tmp);

            flags |= 0x1 << 3;
        }
    }

    ionoutc->vflg = false;
    if (flags == 0xF) // Read all Iono/UTC lines
        ionoutc->vflg = true;

    // Read ephemeris blocks
    g0.week = -1;
    ieph = 0;

    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // PRN
        strncpy(tmp, str, 2);
        tmp[2] = 0;
        sv = atoi(tmp) - 1;

        // EPOCH
        strncpy(tmp, str + 3, 2);
        tmp[2] = 0;
        t.y = atoi(tmp) + 2000;

        strncpy(tmp, str + 6, 2);
        tmp[2] = 0;
        t.m = atoi(tmp);

        strncpy(tmp, str + 9, 2);
        tmp[2] = 0;
        t.d = atoi(tmp);

        strncpy(tmp, str + 12, 2);
        tmp[2] = 0;
        t.hh = atoi(tmp);

        strncpy(tmp, str + 15, 2);
        tmp[2] = 0;
        t.mm = atoi(tmp);

        strncpy(tmp, str + 18, 4);
        tmp[2] = 0;
        t.sec = atof(tmp);

        date2gps(&t, &g);

        if (g0.week == -1)
            g0 = g;

        // Check current time of clock
        dt = subGpsTime(g, g0);

        if (dt > SECONDS_IN_HOUR) {
            g0 = g;
            ieph++; // a new set of ephemerides

            if (ieph >= EPHEM_ARRAY_SIZE)
                break;
        }

        // Date and time
        eph[ieph][sv].t = t;

        // SV CLK
        eph[ieph][sv].toc = g;

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19); // tmp[15]='E';
        eph[ieph][sv].af0 = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af1 = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af2 = atof(tmp);

        // BROADCAST ORBIT - 1
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iode = (int) atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crs = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].deltan = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].m0 = atof(tmp);

        // BROADCAST ORBIT - 2
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cuc = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].ecc = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cus = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sqrta = atof(tmp);

        // BROADCAST ORBIT - 3
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.sec = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cic = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omg0 = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cis = atof(tmp);

        // BROADCAST ORBIT - 4
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].inc0 = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crc = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].aop = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omgdot = atof(tmp);

        // BROADCAST ORBIT - 5
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].idot = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].code = (int) atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.week = (int) atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].flag = (int) atof(tmp);

        // BROADCAST ORBIT - 6
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sva = (int) atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].svh = (int) atof(tmp);
        if ((eph[ieph][sv].svh > 0) && (eph[ieph][sv].svh < 32))
            eph[ieph][sv].svh += 32; // Set MSB to 1

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].tgd = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iodc = (int) atof(tmp);

        // BROADCAST ORBIT - 7
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].fit = atof(tmp);

        // Set valid flag
        eph[ieph][sv].vflg = true;

        // Update the working variables
        eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
        eph[ieph][sv].n = sqrt(GM_EARTH / (eph[ieph][sv].A * eph[ieph][sv].A * eph[ieph][sv].A)) + eph[ieph][sv].deltan;
        eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc * eph[ieph][sv].ecc);
        eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
    }

    gzclose(fp);

    if (g0.week >= 0)
        ieph += 1; // Number of sets of ephemerides

    return (ieph);
}

/* Read Ephemeris data from the RINEX v3 Navigation file
 * eph Array of Output SV ephemeris data
 * fname File name of the RINEX file
 * Number of sets of ephemerides in the file
 */
int readRinex3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, char rinex_date[21], const char *fname) {
    struct gzFile_s *fp;
    int ieph;

    int sv;
    char str[MAX_CHAR];
    char tmp[20];
    double ver = 0.0;

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    if (NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    // Clear valid flag
    for (ieph = 0; ieph < EPHEM_ARRAY_SIZE; ieph++)
        for (sv = 0; sv < MAX_SAT; sv++)
            eph[ieph][sv].vflg = false;

    // Read header lines
    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        if (strncmp(str + 60, "COMMENT", 7) == 0) {
            continue;
        } else if (strncmp(str + 60, "END OF HEADER", 13) == 0) {
            break;
        } else if (strncmp(str + 60, "RINEX VERSION / TYPE", 20) == 0) {
            strncpy(tmp, str, 9);
            tmp[9] = 0;
            replaceExpDesignator(tmp, 9);
            ver = atof(tmp);
            if (ver < 3.0) {
                gzclose(fp);
                return -2;
            }

            if (str[20] != 'N' && str[40] != 'G') {
                gzclose(fp);
                return -3;
            }
        } else if (strncmp(str + 60, "PGM / RUN BY / DATE", 19) == 0) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (strncmp(str + 60, "IONOSPHERIC CORR", 16) == 0) {
            if (strncmp(str, "GPSA", 4) == 0) {
                strncpy(tmp, str + 5, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha0 = atof(tmp);

                strncpy(tmp, str + 17, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha1 = atof(tmp);

                strncpy(tmp, str + 29, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha2 = atof(tmp);

                strncpy(tmp, str + 41, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha3 = atof(tmp);

                flags |= 0x1;
            } else if (strncmp(str, "GPSB", 4) == 0) {
                strncpy(tmp, str + 5, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta0 = atof(tmp);

                strncpy(tmp, str + 17, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta1 = atof(tmp);

                strncpy(tmp, str + 29, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta2 = atof(tmp);

                strncpy(tmp, str + 41, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta3 = atof(tmp);

                flags |= 0x1 << 1;
            }
        } else if (strncmp(str + 60, "TIME SYSTEM CORR", 16) == 0 && strncmp(str, "GPUT", 4) == 0) {
            strncpy(tmp, str + 5, 17);
            tmp[17] = 0;
            replaceExpDesignator(tmp, 17);
            ionoutc->A0 = atof(tmp);

            strncpy(tmp, str + 22, 16);
            tmp[16] = 0;
            replaceExpDesignator(tmp, 16);
            ionoutc->A1 = atof(tmp);

            strncpy(tmp, str + 38, 7);
            tmp[7] = 0;
            replaceExpDesignator(tmp, 7);
            ionoutc->tot = atoi(tmp);

            strncpy(tmp, str + 45, 6);
            tmp[6] = 0;
            ionoutc->wnt = atoi(tmp);

            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (strncmp(str + 60, "LEAP SECONDS", 12) == 0) {
            strncpy(tmp, str, 6);
            tmp[6] = 0;
            ionoutc->dtls = atoi(tmp);

            flags |= 0x1 << 3;
        }
    }

    ionoutc->vflg = false;
    if (flags == 0xF) // Read all Iono/UTC lines
        ionoutc->vflg = true;

    // Read ephemeris blocks
    g0.week = -1;
    ieph = 0;

    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // Check for GPS data record
        if (str[0] != 'G') {
            continue;
        }

        // PRN
        strncpy(tmp, str + 1, 2);
        tmp[2] = 0;
        sv = atoi(tmp) - 1;

        // EPOCH
        strncpy(tmp, str + 4, 4);
        tmp[4] = 0;
        t.y = atoi(tmp);

        strncpy(tmp, str + 9, 2);
        tmp[2] = 0;
        t.m = atoi(tmp);

        strncpy(tmp, str + 12, 2);
        tmp[2] = 0;
        t.d = atoi(tmp);

        strncpy(tmp, str + 15, 2);
        tmp[2] = 0;
        t.hh = atoi(tmp);

        strncpy(tmp, str + 18, 2);
        tmp[2] = 0;
        t.mm = atoi(tmp);

        strncpy(tmp, str + 21, 2);
        tmp[2] = 0;
        t.sec = (double) atoi(tmp);

        date2gps(&t, &g);

        if (g0.week == -1)
            g0 = g;

        // Check current time of clock
        dt = subGpsTime(g, g0);

        if (dt > SECONDS_IN_HOUR) {
            g0 = g;
            ieph++; // a new set of ephemerides

            if (ieph >= EPHEM_ARRAY_SIZE)
                break;
        }

        // Date and time
        eph[ieph][sv].t = t;

        // SV CLK
        eph[ieph][sv].toc = g;

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af0 = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af1 = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af2 = atof(tmp);

        // BROADCAST ORBIT - 1
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iode = (int) atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crs = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].deltan = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].m0 = atof(tmp);

        // BROADCAST ORBIT - 2
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cuc = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].ecc = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cus = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sqrta = atof(tmp);

        // BROADCAST ORBIT - 3
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.sec = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cic = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omg0 = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cis = atof(tmp);

        // BROADCAST ORBIT - 4
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].inc0 = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crc = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].aop = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omgdot = atof(tmp);

        // BROADCAST ORBIT - 5
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].idot = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].code = (int) atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.week = (int) atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].flag = (int) atof(tmp);

        // BROADCAST ORBIT - 6
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // SV accuracy not read

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].svh = (int) atof(tmp);
        if ((eph[ieph][sv].svh > 0) && (eph[ieph][sv].svh < 32))
            eph[ieph][sv].svh += 32; // Set MSB to 1

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].tgd = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iodc = (int) atof(tmp);

        // BROADCAST ORBIT - 7
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].fit = atof(tmp);

        // Set valid flag
        eph[ieph][sv].vflg = true;

        // Update the working variables
        eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
        eph[ieph][sv].n = sqrt(GM_EARTH / (eph[ieph][sv].A * eph[ieph][sv].A * eph[ieph][sv].A)) + eph[ieph][sv].deltan;
        eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc * eph[ieph][sv].ecc);
        eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
    }

    gzclose(fp);

    if (g0.week >= 0)
        ieph += 1; // Number of sets of ephemerides

    return (ieph);
}