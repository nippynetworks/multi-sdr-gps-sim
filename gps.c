/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <curl/curl.h>
#include <zlib.h>
#include "download.h"
#include "sdr.h"
#include "gui.h"
#include "fifo.h"
#include "almanac.h"
#include "ephemeris.h"
#include "gps-sim.h"

/**
 * Note:
 * Not all stations provide RINEX data with ionosphere data.
 */

/**
 * Stations providing Rinex v3 format.
 * 4-characters station ID
 * 9-characters station ID
 * Station name
 * Including Ionosphere data in Rinex file
 */
const stations_t stations[] = {
    // {"abmf", "ABMF00GLP", "Aeroport du Raizet"},
    // {"aggo", "AGGO00ARG", "AGGO"},
    // {"ajac", "AJAC00FRA", "Ajaccio"},
    {"areg", "AREG00PER", "Arequipa"},
    {"ascg", "ASCG00SHN", "Ascension"},
    {"bogi", "BOGI00POL", "Borowa Gora"},
    {"bor1", "BOR100POL", "Borowiec"},
    // {"brst", "BRST00FRA", "Brest"},
    {"chpg", "CHPG00BRA", "Cachoeira Paulista"},
    {"cibg", "CIBG00IDN", "Cibinong"},
    {"cpvg", "CPVG00CPV", "CAP-VERT"},
    {"djig", "DJIG00DJI", "Djibouti"},
    // {"dlf1", "DLF100NLD", "Delft"},
    // {"ffmj", "FFMJ00DEU", "Frankfurt/Main"},
    {"flrs", "FLRS00PRT", "Santa Cruz das Flore"},
    {"ftna", "FTNA00WLF", "Futuna"},
    {"func", "FUNC00PRT", "Funchal"},
    {"gamb", "GAMB00PYF", "Rikitea"},
    {"gamg", "GAMG00KOR", "Geochang"},
    // {"glsv", "GLSV00UKR", "Kiev/Golosiiv"},
    {"gop6", "GOP600CZE", "Pecny, Ondrejov"},
    {"gop7", "GOP700CZE", "Pecny, Ondrejov"},
    {"gope", "GOPE00CZE", "Pecny, Ondrejov"},
    // {"grac", "GRAC00FRA", "Grasse"},
    {"gras", "GRAS00FRA", "Observatoire de Calern - OCA"},
    {"ieng", "IENG00ITA", "Torino"},
    // {"ista", "ISTA00TUR", "Istanbul"},
    // {"izmi", "IZMI00TUR", "Izmir"},
    // {"joz2", "JOZ200POL", "Jozefoslaw"},
    {"kerg", "KERG00ATF", "Kerguelen Islands"},
    {"kitg", "KITG00UZB", "Kitab"},
    {"koug", "KOUG00GUF", "Kourou"},
    {"krgg", "KRGG00ATF", "Kerguelen Islands"},
    // {"krs1", "KRS100TUR", "Kars"},
    {"lama", "LAMA00POL", "Lamkowo"},
    // {"leij", "LEIJ00DEU", "Leipzig"},
    // {"lmmf", "LMMF00MTQ", "Aeroport Aime CESAIRE-LE LAMENTIN"},
    {"lroc", "LROC00FRA", "La Rochelle"},
    {"mayg", "MAYG00MYT", "Dzaoudzi"},
    {"mers", "MERS00TUR", "Mersin"},
    // {"mikl", "MIKL00UKR", "Mykolaiv"},
    {"nklg", "NKLG00GAB", "N'KOLTANG"},
    {"nyal", "NYAL00NOR", "Ny-Alesund"},
    {"nya1", "NYA100NOR", "Ny-Alesund"},
    // {"orid", "ORID00MKD", "Ohrid"},
    {"pdel", "PDEL00PRT", "PONTA DELGADA"},
    // {"polv", "POLV00UKR", "Poltava"},
    // {"ptbb", "PTBB00DEU", "Braunschweig"},
    {"ptgg", "PTGG00PHL", "Manilla"},
    // {"rabt", "RABT00MAR", "Rabat, EMI"},
    // {"reun", "REUN00REU", "La Reunion - Observatoire Volcanologique"},
    {"rgdg", "RGDG00ARG", "Rio Grande"},
    {"riga", "RIGA00LVA", "RIGA permanent GPS"},
    {"seyg", "SEYG00SYC", "Mahe"},
    // {"sofi", "SOFI00BGR", "Sofia"},
    {"stj3", "STJ300CAN", "STJ3 CACS-GSD"},
    // {"sulp", "SULP00UKR", "Lviv Polytechnic"},
    {"svtl", "SVTL00RUS", "Svetloe"},
    {"thtg", "THTG00PYF", "Papeete Tahiti"},
    {"thti", "THTI00PYF", "Tahiti"},
    // {"tit2", "TIT200DEU", "Titz / Jackerath"},
    {"tlse", "TLSE00FRA", "Toulouse"},
    {"tro1", "TRO100NOR", "Tromsoe"},
    // {"warn", "WARN00DEU", "Warnemuende"},
    {"wroc", "WROC00POL", "Wroclaw"},
    {"wtza", "WTZA00DEU", "Wettzell"},
    {"yel2", "YEL200CAN", "Yellow Knife"},
    {"zeck", "ZECK00RUS", "Zelenchukskaya"},
    // {"zim2", "ZIM200CHE", "Zimmerwald"},
    // {"zimm", "ZIMM00CHE", "Zimmerwald L+T 88"},
    {NULL, NULL, NULL} // Always last entry
};

const int sinTable512[] = {
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
    50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 91, 94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100, 97,
    94, 91, 89, 86, 83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 53, 50,
    47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 2,
    -2, -5, -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97, -100, -103, -105, -108, -111, -114, -116, -119, -122, -125, -127, -130, -132, -135, -138,
    -140, -143, -145, -148, -150, -153, -155, -157, -160, -162, -164, -167, -169, -171, -173, -176,
    -178, -180, -182, -184, -186, -188, -190, -192, -194, -196, -198, -200, -202, -204, -205, -207,
    -209, -210, -212, -214, -215, -217, -218, -220, -221, -223, -224, -225, -227, -228, -229, -230,
    -232, -233, -234, -235, -236, -237, -238, -239, -240, -241, -241, -242, -243, -244, -244, -245,
    -245, -246, -247, -247, -248, -248, -248, -249, -249, -249, -249, -250, -250, -250, -250, -250,
    -250, -250, -250, -250, -250, -249, -249, -249, -249, -248, -248, -248, -247, -247, -246, -245,
    -245, -244, -244, -243, -242, -241, -241, -240, -239, -238, -237, -236, -235, -234, -233, -232,
    -230, -229, -228, -227, -225, -224, -223, -221, -220, -218, -217, -215, -214, -212, -210, -209,
    -207, -205, -204, -202, -200, -198, -196, -194, -192, -190, -188, -186, -184, -182, -180, -178,
    -176, -173, -171, -169, -167, -164, -162, -160, -157, -155, -153, -150, -148, -145, -143, -140,
    -138, -135, -132, -130, -127, -125, -122, -119, -116, -114, -111, -108, -105, -103, -100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11, -8, -5, -2
};

const int cosTable512[] = {
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100, 97,
    94, 91, 89, 86, 83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 53, 50,
    47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 2,
    -2, -5, -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97, -100, -103, -105, -108, -111, -114, -116, -119, -122, -125, -127, -130, -132, -135, -138,
    -140, -143, -145, -148, -150, -153, -155, -157, -160, -162, -164, -167, -169, -171, -173, -176,
    -178, -180, -182, -184, -186, -188, -190, -192, -194, -196, -198, -200, -202, -204, -205, -207,
    -209, -210, -212, -214, -215, -217, -218, -220, -221, -223, -224, -225, -227, -228, -229, -230,
    -232, -233, -234, -235, -236, -237, -238, -239, -240, -241, -241, -242, -243, -244, -244, -245,
    -245, -246, -247, -247, -248, -248, -248, -249, -249, -249, -249, -250, -250, -250, -250, -250,
    -250, -250, -250, -250, -250, -249, -249, -249, -249, -248, -248, -248, -247, -247, -246, -245,
    -245, -244, -244, -243, -242, -241, -241, -240, -239, -238, -237, -236, -235, -234, -233, -232,
    -230, -229, -228, -227, -225, -224, -223, -221, -220, -218, -217, -215, -214, -212, -210, -209,
    -207, -205, -204, -202, -200, -198, -196, -194, -192, -190, -188, -186, -184, -182, -180, -178,
    -176, -173, -171, -169, -167, -164, -162, -160, -157, -155, -153, -150, -148, -145, -143, -140,
    -138, -135, -132, -130, -127, -125, -122, -119, -116, -114, -111, -108, -105, -103, -100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11, -8, -5, -2,
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
    50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 91, 94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
const double ant_pat_db[37] = {
    0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22,
    4.89, 5.56, 6.22, 6.89, 7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56
};
// Page number to SVID conversion for subframe 4&5
// See IS-GPS-200L, p.110, table 20-V
const unsigned long sbf4_svId[25] = {
    57UL, 0UL, 0UL, 0UL, 0UL, 57UL, 0UL, 0UL, 0UL, 0UL,
    57UL, 62UL, 52UL, 53UL, 54UL, 57UL, 55UL, 56UL, 58UL,
    59UL, 57UL, 60UL, 61UL, 62UL, 63UL
};

const unsigned long sbf5_svId[25] = {
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
    0UL, 0UL, 0UL, 0UL, 51UL
};

/* Subtract two vectors of double
 * y Result of subtraction
 * x1 Minuend of subtracion
 * x2 Subtrahend of subtracion
 */
static inline void subVect(double *y, const double *x1, const double *x2) {
    y[0] = x1[0] - x2[0];
    y[1] = x1[1] - x2[1];
    y[2] = x1[2] - x2[2];

    return;
}

/* Compute Norm of Vector
 * x Input vector
 * Length (Norm) of the input vector
 */
static inline double normVect(const double *x) {
    return (sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
}

/* Compute dot-product of two vectors
 * x1 First multiplicand
 * x2 Second multiplicand
 * Dot-product of both multiplicands
 */
static inline double dotProd(const double *x1, const double *x2) {
    return (x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2]);
}

/* Generate the C/A code sequence for a given Satellite Vehicle PRN
 * prn PRN nuber of the Satellite Vehicle
 * ca Caller-allocated integer array of 1023 bytes
 */
static void codegen(int *ca, int prn) {
    int delay[] = {
        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862
    };

    int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
    int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
    int c1, c2;
    int i, j;

    if (prn < 1 || prn > 32)
        return;

    for (i = 0; i < N_DWRD_SBF; i++)
        r1[i] = r2[i] = -1;

    for (i = 0; i < CA_SEQ_LEN; i++) {
        g1[i] = r1[9];
        g2[i] = r2[9];
        c1 = r1[2] * r1[9];
        c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];

        for (j = 9; j > 0; j--) {
            r1[j] = r1[j - 1];
            r2[j] = r2[j - 1];
        }
        r1[0] = c1;
        r2[0] = c2;
    }

    for (i = 0, j = CA_SEQ_LEN - delay[prn - 1]; i < CA_SEQ_LEN; i++, j++)
        ca[i] = (1 - g1[i] * g2[j % CA_SEQ_LEN]) / 2;

    return;
}


/* Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Height
 * xyz Input Array of X, Y and Z ECEF coordinates
 * llh Output Array of Latitude, Longitude and Height
 */
static void xyz2llh(const double *xyz, double *llh) {
    double a, eps, e, e2;
    double x, y, z;
    double rho2, dz, zdz, nh, slat, n, dz_new;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;

    eps = 1.0e-3;
    e2 = e*e;

    if (normVect(xyz) < eps) {
        // Invalid ECEF vector
        llh[0] = 0.0;
        llh[1] = 0.0;
        llh[2] = -a;

        return;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    rho2 = x * x + y*y;
    dz = e2*z;

    while (1) {
        zdz = z + dz;
        nh = sqrt(rho2 + zdz * zdz);
        slat = zdz / nh;
        n = a / sqrt(1.0 - e2 * slat * slat);
        dz_new = n * e2*slat;

        if (fabs(dz - dz_new) < eps)
            break;

        dz = dz_new;
    }

    llh[0] = atan2(zdz, sqrt(rho2));
    llh[1] = atan2(y, x);
    llh[2] = nh - n;

    return;
}

/* Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 * llh Input Array of Latitude, Longitude and Height
 * xyz Output Array of X, Y and Z ECEF coordinates
 */
static void llh2xyz(const double *llh, double *xyz) {
    double n;
    double a;
    double e;
    double e2;
    double clat;
    double slat;
    double clon;
    double slon;
    double d, nph;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;
    e2 = e*e;

    clat = cos(llh[0]);
    slat = sin(llh[0]);
    clon = cos(llh[1]);
    slon = sin(llh[1]);
    d = e*slat;

    n = a / sqrt(1.0 - d * d);
    nph = n + llh[2];

    double tmp = nph*clat;
    xyz[0] = tmp*clon;
    xyz[1] = tmp*slon;
    xyz[2] = ((1.0 - e2) * n + llh[2]) * slat;

    return;
}

/* Compute the intermediate matrix for LLH to ECEF
 * llh Input position in Latitude-Longitude-Height format
 * t Three-by-Three output matrix
 */
static void ltcmat(const double *llh, double t[3][3]) {
    double slat, clat;
    double slon, clon;

    slat = sin(llh[0]);
    clat = cos(llh[0]);
    slon = sin(llh[1]);
    clon = cos(llh[1]);

    t[0][0] = -slat*clon;
    t[0][1] = -slat*slon;
    t[0][2] = clat;
    t[1][0] = -slon;
    t[1][1] = clon;
    t[1][2] = 0.0;
    t[2][0] = clat*clon;
    t[2][1] = clat*slon;
    t[2][2] = slat;

    return;
}

/* Convert Earth-centered Earth-Fixed to ?
 * xyz Input position as vector in ECEF format
 * t Intermediate matrix computed by \ref ltcmat
 * neu Output position as North-East-Up format
 */
static void ecef2neu(const double *xyz, double t[3][3], double *neu) {
    neu[0] = t[0][0] * xyz[0] + t[0][1] * xyz[1] + t[0][2] * xyz[2];
    neu[1] = t[1][0] * xyz[0] + t[1][1] * xyz[1] + t[1][2] * xyz[2];
    neu[2] = t[2][0] * xyz[0] + t[2][1] * xyz[1] + t[2][2] * xyz[2];

    return;
}

/* Convert North-Eeast-Up to Azimuth + Elevation
 * neu Input position in North-East-Up format
 * azel Output array of azimuth + elevation as double
 */
static void neu2azel(double *azel, const double *neu) {
    double ne;

    azel[0] = atan2(neu[1], neu[0]);
    if (azel[0] < 0.0)
        azel[0] += (2.0 * PI);

    ne = sqrt(neu[0] * neu[0] + neu[1] * neu[1]);
    azel[1] = atan2(neu[2], ne);

    return;
}

/* Compute Satellite position, velocity and clock at given time
 * eph Ephemeris data of the satellite
 * g GPS time at which position is to be computed
 * pos Computed position (vector)
 * vel Computed velociy (vector)
 * clk Computed clock
 */
static void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk) {
    // Computing Satellite Velocity using the Broadcast Ephemeris
    // http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

    double tk;
    double mk;
    double ek;
    double ekold;
    double ekdot;
    double cek, sek;
    double pk;
    double pkdot;
    double c2pk, s2pk;
    double uk;
    double ukdot;
    double cuk, suk;
    double ok;
    double sok, cok;
    double ik;
    double ikdot;
    double sik, cik;
    double rk;
    double rkdot;
    double xpk, ypk;
    double xpkdot, ypkdot;

    double relativistic, OneMinusecosE;

    tk = g.sec - eph.toe.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    mk = eph.m0 + eph.n*tk;
    ek = mk;
    ekold = ek + 1.0;

    OneMinusecosE = 0; // Suppress the uninitialized warning.
    while (fabs(ek - ekold) > 1.0E-14) {
        ekold = ek;
        OneMinusecosE = 1.0 - eph.ecc * cos(ekold);
        ek = ek + (mk - ekold + eph.ecc * sin(ekold)) / OneMinusecosE;
    }

    sek = sin(ek);
    cek = cos(ek);

    ekdot = eph.n / OneMinusecosE;

    relativistic = -4.442807633E-10 * eph.ecc * eph.sqrta*sek;

    pk = atan2(eph.sq1e2*sek, cek - eph.ecc) + eph.aop;
    pkdot = eph.sq1e2 * ekdot / OneMinusecosE;

    s2pk = sin(2.0 * pk);
    c2pk = cos(2.0 * pk);

    uk = pk + eph.cus * s2pk + eph.cuc*c2pk;
    suk = sin(uk);
    cuk = cos(uk);
    ukdot = pkdot * (1.0 + 2.0 * (eph.cus * c2pk - eph.cuc * s2pk));

    rk = eph.A * OneMinusecosE + eph.crc * c2pk + eph.crs*s2pk;
    rkdot = eph.A * eph.ecc * sek * ekdot + 2.0 * pkdot * (eph.crs * c2pk - eph.crc * s2pk);

    ik = eph.inc0 + eph.idot * tk + eph.cic * c2pk + eph.cis*s2pk;
    sik = sin(ik);
    cik = cos(ik);
    ikdot = eph.idot + 2.0 * pkdot * (eph.cis * c2pk - eph.cic * s2pk);

    xpk = rk*cuk;
    ypk = rk*suk;
    xpkdot = rkdot * cuk - ypk*ukdot;
    ypkdot = rkdot * suk + xpk*ukdot;

    ok = eph.omg0 + tk * eph.omgkdot - OMEGA_EARTH * eph.toe.sec;
    sok = sin(ok);
    cok = cos(ok);

    pos[0] = xpk * cok - ypk * cik*sok;
    pos[1] = xpk * sok + ypk * cik*cok;
    pos[2] = ypk*sik;

    double tmp = ypkdot * cik - ypk * sik*ikdot;
    vel[0] = -eph.omgkdot * pos[1] + xpkdot * cok - tmp*sok;
    vel[1] = eph.omgkdot * pos[0] + xpkdot * sok + tmp*cok;
    vel[2] = ypk * cik * ikdot + ypkdot*sik;

    // Satellite clock correction
    tk = g.sec - eph.toc.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    clk[0] = eph.af0 + tk * (eph.af1 + tk * eph.af2) + relativistic - eph.tgd;
    clk[1] = eph.af1 + 2.0 * tk * eph.af2;

    return;
}

/* Compute Subframe from Ephemeris
 * eph Ephemeris of given SV
 * sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]) {
    unsigned long wn;
    unsigned long toe;
    unsigned long toc;
    unsigned long iode;
    unsigned long iodc;
    long deltan;
    long cuc;
    long cus;
    long cic;
    long cis;
    long crc;
    long crs;
    unsigned long ecc;
    unsigned long sqrta;
    long m0;
    long omega0;
    long inc0;
    long aop;
    long omegadot;
    long idot;
    long af0;
    long af1;
    long af2;
    long tgd;

    unsigned long ura = 0UL;
    unsigned long dataId = 1UL;

    unsigned long wna;
    unsigned long toa;

    signed long alpha0, alpha1, alpha2, alpha3;
    signed long beta0, beta1, beta2, beta3;
    signed long A0, A1;
    signed long dtls, dtlsf;
    unsigned long tot, wnt, wnlsf, dn;

    int i;
    unsigned long svId;
    signed long delta_i; // Relative to i0 = 0.30 semicircles

    // FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
    //wn = (unsigned long)(eph.toe.week%1024);
    wn = 0UL;
    toe = (unsigned long) (eph.toe.sec / 16.0);
    toc = (unsigned long) (eph.toc.sec / 16.0);
    iode = (unsigned long) (eph.iode);
    iodc = (unsigned long) (eph.iodc);
    deltan = (long) (eph.deltan / POW2_M43 / PI);
    cuc = (long) (eph.cuc / POW2_M29);
    cus = (long) (eph.cus / POW2_M29);
    cic = (long) (eph.cic / POW2_M29);
    cis = (long) (eph.cis / POW2_M29);
    crc = (long) (eph.crc / POW2_M5);
    crs = (long) (eph.crs / POW2_M5);
    ecc = (unsigned long) (eph.ecc / POW2_M33);
    sqrta = (unsigned long) (eph.sqrta / POW2_M19);
    m0 = (long) (eph.m0 / POW2_M31 / PI);
    omega0 = (long) (eph.omg0 / POW2_M31 / PI);
    inc0 = (long) (eph.inc0 / POW2_M31 / PI);
    aop = (long) (eph.aop / POW2_M31 / PI);
    omegadot = (long) (eph.omgdot / POW2_M43 / PI);
    idot = (long) (eph.idot / POW2_M43 / PI);
    af0 = (long) (eph.af0 / POW2_M31);
    af1 = (long) (eph.af1 / POW2_M43);
    af2 = (long) (eph.af2 / POW2_M55);
    tgd = (long) (eph.tgd / POW2_M31);

    alpha0 = (signed long) round(ionoutc.alpha0 / POW2_M30);
    alpha1 = (signed long) round(ionoutc.alpha1 / POW2_M27);
    alpha2 = (signed long) round(ionoutc.alpha2 / POW2_M24);
    alpha3 = (signed long) round(ionoutc.alpha3 / POW2_M24);
    beta0 = (signed long) round(ionoutc.beta0 / 2048.0);
    beta1 = (signed long) round(ionoutc.beta1 / 16384.0);
    beta2 = (signed long) round(ionoutc.beta2 / 65536.0);
    beta3 = (signed long) round(ionoutc.beta3 / 65536.0);
    A0 = (signed long) round(ionoutc.A0 / POW2_M30);
    A1 = (signed long) round(ionoutc.A1 / POW2_M50);
    dtls = (signed long) (ionoutc.dtls);
    tot = (unsigned long) (ionoutc.tot / 4096);
    wnt = (unsigned long) (ionoutc.wnt % 256);
    // TO DO: Specify scheduled leap seconds in command options
    // 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
    // Days are counted from 1 to 7 (Sunday is 1).
    wnlsf = 1929 % 256;
    dn = 7;
    dtlsf = 18;

    // Subframe 1
    sbf[0][0] = 0x8B0000UL << 6;
    sbf[0][1] = 0x1UL << 8;
    sbf[0][2] = ((wn & 0x3FFUL) << 20) | (ura << 14) | (((iodc >> 8)&0x3UL) << 6);
    sbf[0][3] = 0UL;
    sbf[0][4] = 0UL;
    sbf[0][5] = 0UL;
    sbf[0][6] = (tgd & 0xFFUL) << 6;
    sbf[0][7] = ((iodc & 0xFFUL) << 22) | ((toc & 0xFFFFUL) << 6);
    sbf[0][8] = ((af2 & 0xFFUL) << 22) | ((af1 & 0xFFFFUL) << 6);
    sbf[0][9] = (af0 & 0x3FFFFFUL) << 8;

    // Subframe 2
    sbf[1][0] = 0x8B0000UL << 6;
    sbf[1][1] = 0x2UL << 8;
    sbf[1][2] = ((iode & 0xFFUL) << 22) | ((crs & 0xFFFFUL) << 6);
    sbf[1][3] = ((deltan & 0xFFFFUL) << 14) | (((m0 >> 24)&0xFFUL) << 6);
    sbf[1][4] = (m0 & 0xFFFFFFUL) << 6;
    sbf[1][5] = ((cuc & 0xFFFFUL) << 14) | (((ecc >> 24)&0xFFUL) << 6);
    sbf[1][6] = (ecc & 0xFFFFFFUL) << 6;
    sbf[1][7] = ((cus & 0xFFFFUL) << 14) | (((sqrta >> 24)&0xFFUL) << 6);
    sbf[1][8] = (sqrta & 0xFFFFFFUL) << 6;
    sbf[1][9] = (toe & 0xFFFFUL) << 14;

    // Subframe 3
    sbf[2][0] = 0x8B0000UL << 6;
    sbf[2][1] = 0x3UL << 8;
    sbf[2][2] = ((cic & 0xFFFFUL) << 14) | (((omega0 >> 24)&0xFFUL) << 6);
    sbf[2][3] = (omega0 & 0xFFFFFFUL) << 6;
    sbf[2][4] = ((cis & 0xFFFFUL) << 14) | (((inc0 >> 24)&0xFFUL) << 6);
    sbf[2][5] = (inc0 & 0xFFFFFFUL) << 6;
    sbf[2][6] = ((crc & 0xFFFFUL) << 14) | (((aop >> 24)&0xFFUL) << 6);
    sbf[2][7] = (aop & 0xFFFFFFUL) << 6;
    sbf[2][8] = (omegadot & 0xFFFFFFUL) << 6;
    sbf[2][9] = ((iode & 0xFFUL) << 22) | ((idot & 0x3FFFUL) << 8);

    // Empty all the pages of subframes 4 and 5
    for (i = 0; i < 25; i++) {
        svId = 0UL; // Dummy SV
        //svId = sbf4_svId[i];

        sbf[3 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
        sbf[3 + i * 2][1] = 0x4UL << 8; // Subframe ID for HOW
        sbf[3 + i * 2][2] = (dataId << 28) | (svId << 22) | ((EMPTY_WORD & 0xFFFFUL) << 6);
        sbf[3 + i * 2][3] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][4] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][5] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][6] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][7] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][8] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[3 + i * 2][9] = (EMPTY_WORD & 0x3FFFFFUL) << 8;

        //svId = sbf5_svId[i];

        sbf[4 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
        sbf[4 + i * 2][1] = 0x5UL << 8; // Subframe ID for HOW
        sbf[4 + i * 2][2] = (dataId << 28) | (svId << 22) | ((EMPTY_WORD & 0xFFFFUL) << 6);
        sbf[4 + i * 2][3] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][4] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][5] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][6] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][7] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][8] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
        sbf[4 + i * 2][9] = (EMPTY_WORD & 0x3FFFFFUL) << 8;
    }

    // Subframe 4, pages 2-5 and 7-10: almanac data for PRN 25 through 32
    for (int sv = 24; sv < MAX_SAT; sv++) {
        if (sv >= 24 && sv <= 27) // PRN 25-28
            i = sv - 23; // Pages 2-5 (i = 1-4)
        else if (sv >= 28 && sv < MAX_SAT) // PRN 29-32
            i = sv - 22; // Page 7-10 (i = 6-9)

        if (alm->sv[sv].valid != 0) {
            svId = (unsigned long) (sv + 1);
            ecc = (unsigned long) (alm->sv[sv].e / POW2_M21);
            toa = (unsigned long) (alm->sv[sv].toa.sec / POW2_12);
            delta_i = (signed long) (alm->sv[sv].delta_i / POW2_M19);
            omegadot = (signed long) (alm->sv[sv].omegadot / POW2_M38);
            sqrta = (unsigned long) (alm->sv[sv].sqrta / POW2_M11);
            omega0 = (signed long) (alm->sv[sv].omega0 / POW2_M23);
            aop = (signed long) (alm->sv[sv].aop / POW2_M23);
            m0 = (signed long) (alm->sv[sv].m0 / POW2_M23);
            af0 = (signed long) (alm->sv[sv].af0 / POW2_M20);
            af1 = (signed long) (alm->sv[sv].af1 / POW2_M38);

            sbf[3 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
            sbf[3 + i * 2][1] = 0x4UL << 8; // Subframe ID for HOW
            sbf[3 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
            sbf[3 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((delta_i & 0xFFFFUL) << 6);
            sbf[3 + i * 2][4] = ((omegadot & 0xFFFFUL) << 14); // SV HEALTH = 000 (ALL DATA OK)
            sbf[3 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][6] = ((omega0 & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
        }
    }

    // Subframe 4, page 18: ionospheric and UTC data
    if (ionoutc.vflg == true) {
        sbf[3 + 17 * 2][0] = 0x8B0000UL << 6;
        sbf[3 + 17 * 2][1] = 0x4UL << 8;
        sbf[3 + 17 * 2][2] = (dataId << 28) | (sbf4_svId[17] << 22) | ((alpha0 & 0xFFUL) << 14) | ((alpha1 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][3] = ((alpha2 & 0xFFUL) << 22) | ((alpha3 & 0xFFUL) << 14) | ((beta0 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][4] = ((beta1 & 0xFFUL) << 22) | ((beta2 & 0xFFUL) << 14) | ((beta3 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][5] = (A1 & 0xFFFFFFUL) << 6;
        sbf[3 + 17 * 2][6] = ((A0 >> 8)&0xFFFFFFUL) << 6;
        sbf[3 + 17 * 2][7] = ((A0 & 0xFFUL) << 22) | ((tot & 0xFFUL) << 14) | ((wnt & 0xFFUL) << 6);
        sbf[3 + 17 * 2][8] = ((dtls & 0xFFUL) << 22) | ((wnlsf & 0xFFUL) << 14) | ((dn & 0xFFUL) << 6);
        sbf[3 + 17 * 2][9] = (dtlsf & 0xFFUL) << 22;
    }

    // Subframe 4, page 25: SV health data for PRN 25 through 32
    sbf[3 + 24 * 2][0] = 0x8B0000UL << 6;
    sbf[3 + 24 * 2][1] = 0x4UL << 8;
    sbf[3 + 24 * 2][2] = (dataId << 28) | (sbf4_svId[24] << 22);
    sbf[3 + 24 * 2][3] = 0UL;
    sbf[3 + 24 * 2][4] = 0UL;
    sbf[3 + 24 * 2][5] = 0UL;
    sbf[3 + 24 * 2][6] = 0UL;
    sbf[3 + 24 * 2][7] = 0UL;
    sbf[3 + 24 * 2][8] = 0UL;
    sbf[3 + 24 * 2][9] = 0UL;

    // Subframe 5, page 1-24: almanac data for PRN 1 through 24
    for (int sv = 0; sv < 24; sv++) {
        i = sv;

        if (alm->sv[sv].svid != 0) {
            svId = (unsigned long) (sv + 1);
            ecc = (unsigned long) (alm->sv[sv].e / POW2_M21);
            toa = (unsigned long) (alm->sv[sv].toa.sec / 4096.0);
            delta_i = (signed long) (alm->sv[sv].delta_i / POW2_M19);
            omegadot = (signed long) (alm->sv[sv].omegadot / POW2_M38);
            sqrta = (unsigned long) (alm->sv[sv].sqrta / POW2_M11);
            omega0 = (signed long) (alm->sv[sv].omega0 / POW2_M23);
            aop = (signed long) (alm->sv[sv].aop / POW2_M23);
            m0 = (signed long) (alm->sv[sv].m0 / POW2_M23);
            af0 = (signed long) (alm->sv[sv].af0 / POW2_M20);
            af1 = (signed long) (alm->sv[sv].af1 / POW2_M38);

            sbf[4 + i * 2][0] = 0x8B0000UL << 6; // Preamble
            sbf[4 + i * 2][1] = 0x5UL << 8; // Subframe ID
            sbf[4 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
            sbf[4 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((delta_i & 0xFFFFUL) << 6);
            sbf[4 + i * 2][4] = ((omegadot & 0xFFFFUL) << 14); // SV HEALTH = 000 (ALL DATA OK)
            sbf[4 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][6] = ((omega0 & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
        }
    }

    // Subframe 5, page 25: SV health data for PRN 1 through 24
    wna = (unsigned long) (eph.toe.week % 256);
    toa = (unsigned long) (eph.toe.sec / 4096.0);

    for (int sv = 0; sv < MAX_SAT; sv++) {
        if (alm->sv[sv].svid != 0) // Valid almanac is availabe
        {
            wna = (unsigned long) (alm->sv[sv].toa.week % 256);
            toa = (unsigned long) (alm->sv[sv].toa.sec / 4096.0);
            break;
        }
    }

    sbf[4 + 24 * 2][0] = 0x8B0000UL << 6;
    sbf[4 + 24 * 2][1] = 0x5UL << 8;
    sbf[4 + 24 * 2][2] = (dataId << 28) | (sbf5_svId[24] << 22) | ((toa & 0xFFUL) << 14) | ((wna & 0xFFUL) << 6);
    sbf[4 + 24 * 2][3] = 0UL;
    sbf[4 + 24 * 2][4] = 0UL;
    sbf[4 + 24 * 2][5] = 0UL;
    sbf[4 + 24 * 2][6] = 0UL;
    sbf[4 + 24 * 2][7] = 0UL;
    sbf[4 + 24 * 2][8] = 0UL;
    sbf[4 + 24 * 2][9] = 0UL;
}

/* Count number of bits set to 1
 * v long word in which bits are counted
 * Count of bits set to 1
 */
static inline unsigned long countBits(unsigned long v) {
    return __builtin_popcountl(v);
}

static int decode_wordN(unsigned int word) {
    const unsigned int hamming[] = {
        0xBB1F3480, 0x5D8F9A40, 0xAEC7CD00, 0x5763E680, 0x6BB1F340, 0x8B7A89C0
    };
    unsigned int parity = 0, w;
    int i;

    if (word & 0x40000000) word ^= 0x3FFFFFC0;

    for (i = 0; i < 6; i++) {
        parity <<= 1;
        for (w = (word & hamming[i]) >> 6; w; w >>= 1) parity ^= w & 1;
    }
    if (parity != (word & 0x3F)) return 0;

    //for (i=0;i<3;i++) data[i]=(unsigned char)(word>>(22-i*8));
    return 1;
}

static bool validate_parityN(unsigned int W) {

    // Parity stuff

    static const unsigned int PARITY_25 = 0xBB1F3480;
    static const unsigned int PARITY_26 = 0x5D8F9A40;
    static const unsigned int PARITY_27 = 0xAEC7CD00;
    static const unsigned int PARITY_28 = 0x5763E680;
    static const unsigned int PARITY_29 = 0x6BB1F340;
    static const unsigned int PARITY_30 = 0x8B7A89C0;

    // Look-up table for parity of eight bit bytes
    // (parity=0 if the number of 0s and 1s is equal, else parity=1)
    static unsigned char byteParity[] = {
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
    };

    // Local variables

    unsigned int t, w, p;

    // The sign of the data is determined by the D30* parity bit 
    // of the previous data word. If  D30* is set, invert the data 
    // bits D01..D24 to obtain the d01..d24 (but leave all other
    // bits untouched).

    w = W;
    if (w & 0x40000000) w ^= 0x3FFFFFC0;

    // Compute the parity of the sign corrected data bits d01..d24
    // as described in the ICD-GPS-200

    t = w & PARITY_25;
    p = (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    t = w & PARITY_26;
    p = (p << 1) |
            (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    t = w & PARITY_27;
    p = (p << 1) |
            (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    t = w & PARITY_28;
    p = (p << 1) |
            (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    t = w & PARITY_29;
    p = (p << 1) |
            (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    t = w & PARITY_30;
    p = (p << 1) |
            (byteParity[t & 0xff] ^ byteParity[(t >> 8) & 0xff] ^
            byteParity[(t >> 16) & 0xff] ^ byteParity[(t >> 24)]);

    if ((W & 0x3f) != p) {
        gui_status_wprintw(RED, "%d-%u ", (W & 0x3f), p);
    }
    if (!decode_wordN(W)) {
        gui_status_wprintw(RED, "%d-%u ", (W & 0x3f), p);
    }
    return ((W & 0x3f) == p);
};

/* Compute the Checksum for one given word of a subframe
 * source The input data
 * nib Does this word contain non-information-bearing bits?
 * Computed Checksum
 */
static unsigned long computeChecksum(unsigned long source, int nib) {
    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Source data bits, d1, d2, ..., d24
    Bits  5 to  0 = Empty parity bits
     */

    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
    Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
     */

    /*
                      1            2           3
    bit    12 3456 7890 1234 5678 9012 3456 7890
    ---    -------------------------------------
    D25    11 1011 0001 1111 0011 0100 1000 0000
    D26    01 1101 1000 1111 1001 1010 0100 0000
    D27    10 1110 1100 0111 1100 1101 0000 0000
    D28    01 0111 0110 0011 1110 0110 1000 0000
    D29    10 1011 1011 0001 1111 0011 0100 0000
    D30    00 1011 0111 1010 1000 1001 1100 0000
     */

    unsigned long bmask[6] = {
        0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
        0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL
    };

    unsigned long D;
    unsigned long d = source & 0x3FFFFFC0UL;
    unsigned long D29 = (source >> 31)&0x1UL;
    unsigned long D30 = (source >> 30)&0x1UL;

    if (nib) // Non-information bearing bits for word 2 and 10
    {
        /*
        Solve bits 23 and 24 to presearve parity check
        with zeros in bits 29 and 30.
         */

        if ((D30 + countBits(bmask[4] & d)) % 2)
            d ^= (0x1UL << 6);
        if ((D29 + countBits(bmask[5] & d)) % 2)
            d ^= (0x1UL << 7);
    }

    D = d;
    if (D30)
        D ^= 0x3FFFFFC0UL;

    D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
    D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
    D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
    D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
    D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
    D |= ((D29 + countBits(bmask[5] & d)) % 2);

    D &= 0x3FFFFFFFUL;
    D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits
    unsigned int W = D;
    validate_parityN(W);
    return (D);
}

static double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel) {
    double iono_delay = 0.0;
    double E, phi_u, lam_u, F;

    if (ionoutc->enable == false)
        return (0.0); // No ionospheric delay

    E = azel[1] / PI;
    phi_u = llh[0] / PI;
    lam_u = llh[1] / PI;

    // Obliquity factor
    F = 1.0 + 16.0 * pow((0.53 - E), 3.0);

    if (ionoutc->vflg == false)
        iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    else {
        double t, psi, phi_i, lam_i, phi_m, phi_m2, phi_m3;
        double AMP, PER, X, X2, X4;

        // Earth's central angle between the user position and the earth projection of
        // ionospheric intersection point (semi-circles)
        psi = 0.0137 / (E + 0.11) - 0.022;

        // Geodetic latitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        phi_i = phi_u + psi * cos(azel[0]);
        if (phi_i > 0.416)
            phi_i = 0.416;
        else if (phi_i<-0.416)
            phi_i = -0.416;

        // Geodetic longitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        lam_i = lam_u + psi * sin(azel[0]) / cos(phi_i * PI);

        // Geomagnetic latitude of the earth projection of the ionospheric intersection
        // point (mean ionospheric height assumed 350 km) (semi-circles)
        phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
        phi_m2 = phi_m*phi_m;
        phi_m3 = phi_m2*phi_m;

        AMP = ionoutc->alpha0 + ionoutc->alpha1 * phi_m
                + ionoutc->alpha2 * phi_m2 + ionoutc->alpha3*phi_m3;
        if (AMP < 0.0)
            AMP = 0.0;

        PER = ionoutc->beta0 + ionoutc->beta1 * phi_m
                + ionoutc->beta2 * phi_m2 + ionoutc->beta3*phi_m3;
        if (PER < 72000.0)
            PER = 72000.0;

        // Local time (sec)
        t = SECONDS_IN_DAY / 2.0 * lam_i + g.sec;
        while (t >= SECONDS_IN_DAY)
            t -= SECONDS_IN_DAY;
        while (t < 0)
            t += SECONDS_IN_DAY;

        // Phase (radians)
        X = 2.0 * PI * (t - 50400.0) / PER;

        if (fabs(X) < 1.57) {
            X2 = X*X;
            X4 = X2*X2;
            iono_delay = F * (5.0e-9 + AMP * (1.0 - X2 / 2.0 + X4 / 24.0)) * SPEED_OF_LIGHT;
        } else
            iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    }

    return (iono_delay);
}

/* Compute range between a satellite and the receiver
 * rho The computed range
 * eph Ephemeris data of the satellite
 * g GPS time at time of receiving the signal
 * xyz position of the receiver
 */
static void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]) {
    double pos[3], vel[3], clk[2];
    double los[3];
    double tau;
    double range, rate;
    double xrot, yrot;

    double llh[3], neu[3];
    double tmat[3][3];

    // SV position at time of the pseudorange observation.
    satpos(eph, g, pos, vel, clk);

    // Receiver to satellite vector and light-time.
    subVect(los, pos, xyz);
    tau = normVect(los) / SPEED_OF_LIGHT;

    // Extrapolate the satellite position backwards to the transmission time.
    pos[0] -= vel[0] * tau;
    pos[1] -= vel[1] * tau;
    pos[2] -= vel[2] * tau;

    // Earth rotation correction. The change in velocity can be neglected.
    xrot = pos[0] + pos[1] * OMEGA_EARTH*tau;
    yrot = pos[1] - pos[0] * OMEGA_EARTH*tau;
    pos[0] = xrot;
    pos[1] = yrot;

    // New observer to satellite vector and satellite range.
    subVect(los, pos, xyz);
    range = normVect(los);
    rho->d = range;

    // Pseudorange.
    rho->range = range - SPEED_OF_LIGHT * clk[0];

    // Relative velocity of SV and receiver.
    rate = dotProd(vel, los) / range;

    // Pseudorange rate.
    rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

    // Time of application.
    rho->g = g;

    // Azimuth and elevation angles.
    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);
    ecef2neu(los, tmat, neu);
    neu2azel(rho->azel, neu);

    // Add ionospheric delay
    rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
    rho->range += rho->iono_delay;
}

/* Compute the code phase for a given channel (satellite)
 * chan Channel on which we operate (is updated)
 * rho1 Current range, after \a dt has expired
 * dt delta-t (time difference) in seconds
 */
static void computeCodePhase(channel_t *chan, range_t rho1, double dt) {
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range) / dt;

    // Carrier and code frequency.
    chan->f_carr = -rhorate / LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g, chan->g0) + 6.0) - chan->rho0.range / SPEED_OF_LIGHT)*1000.0;

    ims = (int) ms;
    chan->code_phase = (ms - (double) ims) * CA_SEQ_LEN; // in chip

    chan->iword = ims / 600; // 1 word = 30 bits = 600 ms
    ims -= chan->iword * 600;

    chan->ibit = ims / 20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int) chan->code_phase]*2 - 1;
    chan->dataBit = (int) ((chan->dwrd[chan->iword]>>(29 - chan->ibit)) & 0x1UL)*2 - 1;

    // Save current pseudorange
    chan->rho0 = rho1;
}

void generateNavMsg(gpstime_t g, channel_t *chan, int init) {
    int iwrd, isbf;
    gpstime_t g0;
    unsigned long wn, tow;
    unsigned sbfwrd;
    unsigned long prevwrd;
    int nib;

    g0.week = g.week;
    g0.sec = (double) (((unsigned long) (g.sec + 0.5)) / 30UL) * 30.0; // Align with the full frame length = 30 sec
    chan->g0 = g0; // Data bit reference time

    wn = (unsigned long) (g0.week % 1024);
    tow = ((unsigned long) g0.sec) / 6UL;

    // Initialize the subframe 5
    if (init == 1) {
        prevwrd = 0UL;

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            sbfwrd = chan->sbf[4 + chan->ipage * 2][iwrd];

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[iwrd];
        }
    } else {
        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF * N_SBF + iwrd];

            prevwrd = chan->dwrd[iwrd];
        }
    }

    // Generate subframe words
    for (isbf = 0; isbf < N_SBF; isbf++) {
        tow++;

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            if (isbf < 3) // Subframes 1-3
                sbfwrd = chan->sbf[isbf][iwrd];
            else if (isbf == 3) // Subframe 4
                sbfwrd = chan->sbf[3 + chan->ipage * 2][iwrd];
            else // Subframe 5 
                sbfwrd = chan->sbf[4 + chan->ipage * 2][iwrd];

            // Add transmission week number to Subframe 1
            if ((isbf == 0)&&(iwrd == 2))
                sbfwrd |= (wn & 0x3FFUL) << 20;

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd];
        }
    }

    // Move to the next pages
    chan->ipage++;
    if (chan->ipage >= 25)
        chan->ipage = 0;
}

static int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel) {
    double llh[3], neu[3];
    double pos[3], vel[3], clk[3], los[3];
    double tmat[3][3];

    if (eph.vflg == false)
        return (-1); // Invalid

    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);

    if (azel[1] * R2D > elvMask)
        return (1); // Visible
    // else
    return (0); // Invisible
}

// Cache used only in allocateChannel()
static int allocatedSatCache[MAX_SAT] = {-1};

static int allocateChannel(channel_t *chan, almanac_gps_t *alm, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz, double elvMask) {
    NOTUSED(elvMask);
    int nsat = 0;
    int i;
    double azel[2];

    range_t rho;
    double ref[3] = {0.0};
    double r_ref, r_xyz;
    double phase_ini;

    for (int sv = 0; sv < MAX_SAT; sv++) {
        if (checkSatVisibility(eph[sv], grx, xyz, 0.0, azel) == 1) {
            nsat++; // Number of visible satellites

            if (allocatedSatCache[sv] == -1) // Visible but not allocated
            {
                // Allocated new satellite
                for (i = 0; i < MAX_CHAN; i++) {
                    if (chan[i].prn == 0) {
                        // Initialize channel
                        chan[i].prn = sv + 1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, alm, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;

                        // Initialize carrier phase
                        r_xyz = rho.range;

                        computeRange(&rho, eph[sv], &ionoutc, grx, ref);
                        r_ref = rho.range;

                        phase_ini = (2.0 * r_ref - r_xyz) / LAMBDA_L1;
#ifdef FLOAT_CARR_PHASE
                        chan[i].carr_phase = phase_ini - floor(phase_ini);
#else
                        phase_ini -= floor(phase_ini);
                        chan[i].carr_phase = (unsigned int) (512.0 * 65536.0 * phase_ini);
#endif
                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < MAX_CHAN)
                    allocatedSatCache[sv] = i;
            }
        } else if (allocatedSatCache[sv] >= 0) // Not visible but allocated
        {
            // Clear channel
            chan[allocatedSatCache[sv]].prn = 0;

            // Clear satellite allocation flag
            allocatedSatCache[sv] = -1;
        }
    }

    return (nsat);
}

/* Read the list of user motions from the input file
 * xyz Output array of ECEF vectors for user motion
 * filename File name of the text input file
 * Returns number of user data motion records read, -1 on error
 */
static int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename) {
    FILE *fp;
    int numd;
    char str[MAX_CHAR];
    double t, x, y, z;

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++) {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (EOF == sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV line
            break;

        xyz[numd][0] = x;
        xyz[numd][1] = y;
        xyz[numd][2] = z;
    }

    fclose(fp);

    return (numd);
}

/*
 *
 */
void *gps_thread_ep(void *arg) {
    simulator_t *simulator = (simulator_t *) (arg);
    short *iq_buff = NULL;

    gpstime_t g0;
    g0.week = -1; // Invalid start time
    date2gps(&simulator->start, &g0);

    int start_y = 4; // Row to start output in LS_FIX window/panel

    /* On a multi-core CPU we run the main thread and reader thread on different cores.
     * Try sticking the main thread to core 2
     */
    thread_to_core(2);
    set_thread_name("gps-thread");

    ////////////////////////////////////////////////////////////
    // User motion
    ////////////////////////////////////////////////////////////

    // Allocate user motion array
    double (*xyz)[3] = malloc(sizeof (double[USER_MOTION_SIZE][3]));
    if (xyz == NULL) {
        gui_status_wprintw(RED, "Failed to allocate user motion memory.\n");
        goto end_gps_thread;
    }

    // Initialize user motion array with current location/position
    int iumd = 0;
    int numd = simulator->duration;
    double tmat[3][3];
    double neu[3];

    // Set user location
    double llh[3];
    llh[0] = simulator->location.lat / R2D;
    llh[1] = simulator->location.lon / R2D;
    llh[2] = simulator->location.height;
    llh2xyz(llh, xyz[0]);
    ltcmat(llh, tmat);

    if (!simulator->target.valid) {
        // No target position given, use location
        simulator->target.lat = simulator->location.lat;
        simulator->target.lon = simulator->location.lon;
        simulator->target.height = simulator->location.height;
    } else {
        // Set target position as simulation start
        // Given as distance and bearing from user location in -t option
        neu[0] = simulator->target.distance * cos((simulator->target.bearing / 1000) / R2D);
        neu[1] = simulator->target.distance * sin((simulator->target.bearing / 1000) / R2D);
        neu[2] = simulator->target.height;
        xyz[0][0] += tmat[0][0] * neu[0] + tmat[1][0] * neu[1] + tmat[2][0] * neu[2];
        xyz[0][1] += tmat[0][1] * neu[0] + tmat[1][1] * neu[1] + tmat[2][1] * neu[2];
        xyz[0][2] += tmat[0][2] * neu[0] + tmat[1][2] * neu[1] + tmat[2][2] * neu[2];
    }

    for (iumd = 1; iumd < numd; iumd++) {
        xyz[iumd][0] = xyz[0][0];
        xyz[iumd][1] = xyz[0][1];
        xyz[iumd][2] = xyz[0][2];
    }

    // Read user motion file if given
    if (simulator->motion_file_name != NULL) {
        numd = readUserMotion(xyz, simulator->motion_file_name);
        if (numd <= 0) {
            gui_status_wprintw(RED, "Failed to read user motion file.\n");
            goto end_gps_thread;
        }
        gui_status_wprintw(GREEN, "%u user motion points applied.\n", numd);
        if (numd > simulator->duration) {
            numd = simulator->duration;
        }
    }

    gui_show_location(&simulator->location);

    ////////////////////////////////////////////////////////////
    // Read ephemeris
    ////////////////////////////////////////////////////////////
    if (simulator->nav_file_name == NULL) {
        if (simulator->online_fetch == false) {
            gui_status_wprintw(RED, "GPS ephemeris file is not specified.\n");
            goto end_gps_thread;
        } else {
            simulator->nav_file_name = strdup(RINEX_FILE_NAME);
        }
    }

    // Maybe download latest ephemeris file
    if (simulator->online_fetch) {
        time_t t = time(NULL);
        struct tm *tm = gmtime(&t);
        char* url = malloc(NAME_MAX);
        int station_index = -1;
        int station_count = 0;
        int attempts = 1;
        int max_attempts = 1;
        const stations_t *pstation = stations;

        // Get number of stations available, find index for given one
        for (int s = 0; pstation[s].id_v2 != NULL; s++) {
            // Station id given, get index
            if (simulator->station_id != NULL) {
                if (strncmp(pstation[s].id_v2, simulator->station_id, 4) == 0 || strncmp(pstation[s].id_v3, simulator->station_id, 9) == 0) {
                    station_index = s;
                }
            }
            station_count += 1;
        }

        // Pick a random station if none given, or given station couldn't be found
        if (station_index == -1) {
            srand((unsigned int) g0.sec + (g0.week * 1000000));
            station_index = rand() % station_count;
            max_attempts = station_count;
        }
        // Check that we have a picked a valid station
        // Take the first one when invalid
        if (pstation[station_index].id_v2 == NULL) {
            station_index = 0;
        }

        // We fetch data from previous hour because the actual hour is still in progress
        tm->tm_hour -= 1;
        mktime(tm);

        CURLcode curl_code;
        do {
            // Compose HTTPS URL
            snprintf(url, NAME_MAX, RINEX_HTTPS_URL RINEX_HTTPS_FILE,
                    tm->tm_yday + 1, tm->tm_hour, pstation[station_index].id_v3, tm->tm_year - 100 + 2000, tm->tm_yday + 1, tm->tm_hour * 100);
            gui_status_wprintw(GREEN, "Pulling RINEX from station: %s - %s\n", pstation[station_index].name, url);

            curl_code = download_file_to_disk(url, simulator->nav_file_name);
            if (curl_code == CURLE_OK) {
                simulator->station_id = strndup(pstation[station_index].id_v3, 9);
                max_attempts = 0;
            } else {
                attempts += 1;
                station_index = (station_index + 1) % station_count;
            }
        } while (attempts <= max_attempts);

        free(url);
        if (curl_code != CURLE_OK) {
            gui_status_wprintw(RED, "Curl error: Unable to download any emphemeris file!\n");
            goto end_gps_thread;
        }
    }

    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    ionoutc_t ionoutc;
    ionoutc.enable = simulator->ionosphere_enable;
    int neph = 0, ieph;

    char rinex_date[21];
    int rinex_version = readRinexVersion(simulator->nav_file_name);
    if (rinex_version == 2) {
        neph = readRinex2(eph, &ionoutc, rinex_date, simulator->nav_file_name);
    } else if (rinex_version == 3) {
        neph = readRinex3(eph, &ionoutc, rinex_date, simulator->nav_file_name);
    } else {
        gui_status_wprintw(RED, "Unable to determine if emphemeris version is 2 or 3.\n");
    }

    if (neph == 0) {
        gui_status_wprintw(RED, "No valid ephemeris available.\n");
        goto end_gps_thread;
    }

    if (simulator->show_verbose) {
        int y=15;
        if (ionoutc.vflg && ionoutc.enable) {
            gui_mvwprintw(LS_FIX, y++, 40, "ION ALPHA %12.3e %12.3e %12.3e %12.3e",
                    ionoutc.alpha0, ionoutc.alpha1, ionoutc.alpha2, ionoutc.alpha3);
            gui_mvwprintw(LS_FIX, y++, 40, "ION BETA  %12.3e %12.3e %12.3e %12.3e",
                    ionoutc.beta0, ionoutc.beta1, ionoutc.beta2, ionoutc.beta3);
            gui_mvwprintw(LS_FIX, y++, 40, "DELTA UTC %12.3e %12.3e %9d  %9d",
                    ionoutc.A0, ionoutc.A1, ionoutc.tot, ionoutc.wnt);
            gui_mvwprintw(LS_FIX, y++, 40, "LEAP SECONDS %d", ionoutc.dtls);
        } else {
            gui_mvwprintw(LS_FIX, y++, 40, "Ionospheric data invalid or disabled!");
        }
    }

    ////////////////////////////////////////////////////////////
    // Check start/end times of the simulation
    ////////////////////////////////////////////////////////////

    datetime_t tmin, tmax = {0};
    gpstime_t gmin, gmax = {0};

    for (int sv = 0; sv < MAX_SAT; sv++) {
        if (eph[0][sv].vflg == true) {
            gmin = eph[0][sv].toc;
            tmin = eph[0][sv].t;
            break;
        }
    }

    for (int sv = 0; sv < MAX_SAT; sv++) {
        if (eph[neph - 1][sv].vflg == true) {
            gmax = eph[neph - 1][sv].toc;
            tmax = eph[neph - 1][sv].t;
            break;
        }
    }

    if (g0.week >= 0) // Scenario start time has been set.
    {
        if (simulator->time_overwrite == true) {
            double dsec;
            gpstime_t gtmp;

            gtmp.week = g0.week;
            gtmp.sec = (double) (((int) (g0.sec)) / 7200)*7200.0;

            dsec = subGpsTime(gtmp, gmin);

            // Overwrite the UTC reference week number
            ionoutc.wnt = gtmp.week;
            ionoutc.tot = (int) gtmp.sec;

            // Iono/UTC parameters may no longer valid
            //ionoutc.vflg = FALSE;

            // Overwrite the TOC and TOE to the scenario start time
            datetime_t ttmp;
            for (int sv = 0; sv < MAX_SAT; sv++) {
                for (int i = 0; i < neph; i++) {
                    if (eph[i][sv].vflg == true) {
                        gtmp = incGpsTime(eph[i][sv].toc, dsec);
                        gps2date(&gtmp, &ttmp);
                        eph[i][sv].toc = gtmp;
                        eph[i][sv].t = ttmp;

                        gtmp = incGpsTime(eph[i][sv].toe, dsec);
                        eph[i][sv].toe = gtmp;
                    }
                }
            }
        } else {
            if (subGpsTime(g0, gmin) < 0.0 || subGpsTime(gmax, g0) < -28800.0) {
                gui_status_wprintw(RED, "Invalid start time. Must be: tmin -> tmax +8hr\n");
                gui_status_wprintw(RED, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
                        gmin.week, gmin.sec);
                gui_status_wprintw(RED, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
                        gmax.week, gmax.sec);
                goto end_gps_thread;
            }
        }
    } else {
        g0 = gmin;
        simulator->start = tmin;
    }

    gui_mvwprintw(LS_FIX, 8, 40, "RINEX date:      %s", rinex_date);
    gui_mvwprintw(LS_FIX, 10, 40, "Start time:      %4d/%02d/%02d,%02d:%02d:%02.9f (%d:%.0f)",
            simulator->start.y, simulator->start.m, simulator->start.d, simulator->start.hh, simulator->start.mm, simulator->start.sec, g0.week, g0.sec);
    if (simulator->show_verbose) {
        gui_mvwprintw(LS_FIX, 11, 40, "Simulation time: ");
    }
    gui_mvwprintw(LS_FIX, 7, 40, "Duration:        %.1fs", ((double) numd) / 10.0);

    // Select the current set of ephemerides
    ieph = -1;

    for (int i = 0; i < neph; i++) {
        for (int sv = 0; sv < MAX_SAT; sv++) {
            if (eph[i][sv].vflg == true) {
                double dt = subGpsTime(g0, eph[i][sv].toc);
                if (dt >= -SECONDS_IN_HOUR && ((dt < SECONDS_IN_HOUR) || (i == (neph - 1) && dt < SECONDS_IN_HOUR * 8))) {
                    ieph = i;
                    break;
                }
            }
        }

        if (ieph >= 0) // ieph has been set
            break;
    }

    if (ieph == -1) {
        gui_status_wprintw(RED, "No current set of ephemerides has been found.\n");
        goto end_gps_thread;
    }

    ////////////////////////////////////////////////////////////
    // Read almanac
    ////////////////////////////////////////////////////////////

    char* almanac_file_name = "almanac.sem";
    almanac_gps_t *alm = almanac_init();
    if (simulator->almanac_enable) {
        CURLcode curl_code;
        if (simulator->online_fetch) {
            curl_code = almanac_download(almanac_file_name);
        } else {
            curl_code = almanac_read_file(almanac_file_name);
        }

        if (curl_code != CURLE_OK) {
            switch (curl_code) {
                case CURLE_REMOTE_FILE_NOT_FOUND:
                    gui_status_wprintw(RED, "Almanac file not found!\n");
                    break;
                case CURLE_READ_ERROR:
                    gui_status_wprintw(RED, "Error reading almanac file!\n");
                    break;
                default:
                    gui_status_wprintw(RED, "Almanac error, code: %d\n", curl_code);
                    break;
            }
        }
    }

    if (simulator->almanac_enable && alm->valid) {
        double dt = subGpsTime(alm->almanac_time, g0);
        if (dt < (-4.0 * SECONDS_IN_WEEK) || dt > (4.0 * SECONDS_IN_WEEK)) {
            gui_status_wprintw(RED, "Invalid time of almanac.\n");
            goto end_gps_thread;
        }

        datetime_t ttmp;
        gps2date(&alm->almanac_time, &ttmp);
        gui_mvwprintw(LS_FIX, 9, 40, "Almanac date:    %4d/%02d/%02d,%02d:%02d:%02.0f",
                ttmp.y, ttmp.m, ttmp.d, ttmp.hh, ttmp.mm, ttmp.sec);
    } else {
        gui_mvwprintw(LS_FIX, 9, 40, "Almanac date:    Disabled or invalid.");
    }

    ////////////////////////////////////////////////////////////
    // Initialise simulation zero time
    // If we are running realtime_sim or sync to PPS
    // Then we need to adjust the simulation start a little
    ////////////////////////////////////////////////////////////
    gpstime_t grx;

    if (simulator->realtime_sim) {
        struct timespec ts;
        int ret = clock_gettime(CLOCK_REALTIME, &ts);

        if (ret != 0) {
            perror("clock_gettime");
            gui_status_wprintw(RED, "Failed to get clock.\n");
            goto end_gps_thread;
        }
        struct tm *gmt = gmtime(&ts.tv_sec);
        datetime_t start;

        start.y = gmt->tm_year + 1900;
        start.m = gmt->tm_mon + 1;
        start.d = gmt->tm_mday;
        start.hh = gmt->tm_hour;
        start.mm = gmt->tm_min;
        start.sec = (double) gmt->tm_sec;
        date2gps(&start, &g0);

        if (simulator->sync_start) {
            incGpsTime(g0, 1.0);
            simulator->elapsed_ns = 0.0 - simulator->sdr_latency_ns;
        } else {
            simulator->elapsed_ns = ts.tv_nsec - simulator->sdr_latency_ns;
        }
    } else {
        simulator->elapsed_ns = round((g0.sec - floor(g0.sec)) * 1e9) - simulator->sdr_latency_ns;
        g0.sec = floor(g0.sec);
    }


    // Initial reception time
    grx = incGpsTime(g0, (double)(simulator->elapsed_ns + simulator->offset_ns) / 1e9);

    ////////////////////////////////////////////////////////////
    // Initialize channels
    ////////////////////////////////////////////////////////////
    double ant_pat[37];
    double elvmask = 0.0; // in degree
    bool sat_simulated[33] = {false};

    channel_t chan[MAX_CHAN];

    // Clear all channels
    for (int i = 0; i < MAX_CHAN; i++)
        chan[i].prn = 0;

    // Clear satellite allocation flag
    for (int sv = 0; sv < MAX_SAT; sv++)
        allocatedSatCache[sv] = -1;

    // Allocate visible satellites
    allocateChannel(chan, alm, eph[ieph], ionoutc, grx, xyz[0], elvmask);

    for (int i = 0; i < MAX_CHAN; i++) {
        if (chan[i].prn > 0) {
            gui_mvwprintw(LS_FIX, start_y++, 1, "%02d %6.1f %5.1f %11.1f %5.1f", chan[i].prn,
                    chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
        }

        sat_simulated[chan[i].prn] = true;
    }
    gui_mvwprintw(LS_FIX, 3, 40, "Nav: %02d satellites", start_y - 4);

    // Receiver antenna gain pattern
    for (int i = 0; i < 37; i++)
        ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

    // Update receiver time
    // grx = incGpsTime(grx, 0.1);
    simulator->elapsed_ns += 1e8;
    grx = incGpsTime(g0, (double)(simulator->elapsed_ns + simulator->offset_ns) / 1e9);

    // Create IQ buffer.
    iq_buff = calloc(IQ_BUFFER_SIZE, 2);

    // Acquire first fifo block for transfer buffer
    struct iq_buf *iq = fifo_acquire();

    ////////////////////////////////////////////////////////////
    // Generate baseband signals
    ////////////////////////////////////////////////////////////
    double gain[MAX_CHAN];

    const double delt = 1.0 / (double) TX_SAMPLERATE;

    for (iumd = 1; iumd < numd; iumd++) {
        if (simulator->gps_thread_exit) {
            break;
        }

        // Signal GPS init done and running
        if (simulator->gps_thread_running == false) {
            simulator->gps_thread_running = true;
            pthread_cond_signal(&(simulator->gps_init_done));
        }

        if (simulator->interactive_mode) {
            // Stay at the current location
            xyz[iumd][0] = xyz[iumd - 1][0];
            xyz[iumd][1] = xyz[iumd - 1][1];
            xyz[iumd][2] = xyz[iumd - 1][2];

            // Update the target location
            double dir = (simulator->target.bearing / 1000) / R2D;
            neu[0] = (simulator->target.velocity * cos(dir)) * 0.1;
            neu[1] = (simulator->target.velocity * sin(dir)) * 0.1;
            neu[2] = simulator->target.vertical_speed * 0.1;

            xyz[iumd][0] += tmat[0][0] * neu[0] + tmat[1][0] * neu[1] + tmat[2][0] * neu[2];
            xyz[iumd][1] += tmat[0][1] * neu[0] + tmat[1][1] * neu[1] + tmat[2][1] * neu[2];
            xyz[iumd][2] += tmat[0][2] * neu[0] + tmat[1][2] * neu[1] + tmat[2][2] * neu[2];
        }

        for (int i = 0; i < MAX_CHAN; i++) {
            if (chan[i].prn > 0) {
                // Refresh code phase and data bit counters
                range_t rho;
                int sv = chan[i].prn - 1;

                // Current pseudorange
                computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[iumd]);

                chan[i].azel[0] = rho.azel[0];
                chan[i].azel[1] = rho.azel[1];

                // Update code phase and data bit counters
                computeCodePhase(&chan[i], rho, 0.1);
#ifndef FLOAT_CARR_PHASE
                chan[i].carr_phasestep = (int) round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
                // Path loss
                double path_loss = 20200000.0 / rho.d;

                int ibs; // boresight angle index
                // Receiver antenna gain
                ibs = (int) ((90.0 - rho.azel[1] * R2D) / 5.0); // covert elevation to boresight

                // Signal gain
                double ant_gain = ant_pat[ibs];
                gain[i] = (double) (path_loss * ant_gain);
                // Pluto SDR needs more signal strength due to 12 bit DAC range.
                // Otherwise signal dynamic range is very low.
                if (simulator->sdr_type == SDR_PLUTOSDR) {
                    // Will result in larger IQ values, hence higher signal amplitude.
                    // Best value to be defined.
                    gain[i] *= 2;
                }
            }
        }

        for (int isamp = 0; isamp < NUM_IQ_SAMPLES; isamp++) {
            int i_acc = 0;
            int q_acc = 0;

            for (int i = 0; i < MAX_CHAN; i++) {
                if (chan[i].prn > 0) {
                    int iTable;
#ifdef FLOAT_CARR_PHASE
                    // carr_phase 0.0 - 1.0
                    iTable = (int) floor(chan[i].carr_phase * 512.0);
#else
                    iTable = (chan[i].carr_phase >> 16) & 511; // 9-bit index
#endif
                    // dataBit -1 or 1
                    // codeCA  -1 or 1
                    int ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
                    int qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

                    // Accumulate for all visible satellites
                    i_acc += ip;
                    q_acc += qp;

                    // Update code phase
                    chan[i].code_phase += chan[i].f_code * delt;

                    if (chan[i].code_phase >= CA_SEQ_LEN) {
                        chan[i].code_phase -= CA_SEQ_LEN;

                        chan[i].icode++;

                        if (chan[i].icode >= 20) // 20 C/A codes = 1 navigation data bit
                        {
                            chan[i].icode = 0;
                            chan[i].ibit++;

                            if (chan[i].ibit >= 30) // 30 navigation data bits = 1 word
                            {
                                chan[i].ibit = 0;
                                chan[i].iword++;
                                /*
                                if (chan[i].iword>=N_DWRD)
                                        fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
                                */
                            }

                            // Set new navigation data bit
                            chan[i].dataBit = (int) ((chan[i].dwrd[chan[i].iword]>>(29 - chan[i].ibit)) & 0x1UL)*2 - 1;
                        }
                    }

                    // Set current code chip
                    chan[i].codeCA = (chan[i].ca[(int) chan[i].code_phase] * 2) - 1;

                    // Update carrier phase
#ifdef FLOAT_CARR_PHASE
                    chan[i].carr_phase += chan[i].f_carr * delt;

                    if (chan[i].carr_phase >= 1.0)
                        chan[i].carr_phase -= 1.0;
                    else if (chan[i].carr_phase < 0.0)
                        chan[i].carr_phase += 1.0;
#else
                    chan[i].carr_phase += chan[i].carr_phasestep;
#endif
                }
            }

            // Store I/Q samples into buffer
            iq_buff[isamp * 2] = (short) i_acc;
            iq_buff[isamp * 2 + 1] = (short) q_acc;
        }

        // Fill transfer fifo
        for (int isamp = 0; isamp < IQ_BUFFER_SIZE; isamp++) {
            // validLength starts with 0 on aquire
            if (simulator->sample_size == SC16) {
                iq->data16[iq->validLength] = iq_buff[isamp];
            } else {
                iq->data8[iq->validLength] = iq_buff[isamp] >> 4;
            }
            iq->validLength += 1;
            if (simulator->sdr_type == SDR_HACKRF) {
                // Fill one fifo block until full
                if (iq->validLength == HACKRF_TRANSFER_BUFFER_SIZE) {
                    // Enqueue full fifo block
                    fifo_enqueue(iq);
                    // Get a new one and fill as long as we don't reach IQ_BUFFER_SIZE
                    iq = fifo_acquire();
                }
                // We don't enque a partly filled fifo block but keep it for the next round
            }
        }

        // File writer and Pluto SDR taking the entire IQ buffer at once.
        if (simulator->sdr_type == SDR_IQFILE || simulator->sdr_type == SDR_PLUTOSDR) {
            // Enqueue full fifo block
            fifo_enqueue(iq);
            // Get a new one
            iq = fifo_acquire();
        }

        //
        // Update navigation message and channel allocation every 30 seconds
        //
        int igrx = (int) (grx.sec * 10.0 + 0.5);

        xyz2llh(xyz[iumd], llh);
        simulator->target.lat = llh[0] * R2D;
        simulator->target.lon = llh[1] * R2D;
        simulator->target.height = llh[2];
        gui_show_target(&simulator->target);

        if (igrx % 300 == 0) // Every 30 seconds
        {
            // Update navigation message
            for (int i = 0; i < MAX_CHAN; i++) {
                if (chan[i].prn > 0) {
                    generateNavMsg(grx, &chan[i], 0);
                }
            }

            // Refresh ephemeris and subframes
            // Quick and dirty fix. Need more elegant way.
            for (int sv = 0; sv < MAX_SAT; sv++) {
                if (eph[ieph + 1][sv].vflg == true) {
                    double dt = subGpsTime(eph[ieph + 1][sv].toc, grx);
                    if (dt < SECONDS_IN_HOUR) {
                        ieph++;
                        if (ieph >= EPHEM_ARRAY_SIZE) {
                            ieph = 0;
                        }

                        for (int i = 0; i < MAX_CHAN; i++) {
                            // Generate new subframes if allocated
                            if (chan[i].prn != 0)
                                eph2sbf(eph[ieph][chan[i].prn - 1], ionoutc, alm, chan[i].sbf);
                        }
                    }
                    break;
                }
            }

            // Update channel allocation
            allocateChannel(chan, alm, eph[ieph], ionoutc, grx, xyz[0], elvmask);

            if (simulator->show_verbose) {
                gps2date(&grx, &simulator->start);
                gui_mvwprintw(LS_FIX, 11, 57, "%4d/%02d/%02d,%02d:%02d:%02.9f (%d:%.0f)",
                        simulator->start.y, simulator->start.m, simulator->start.d, simulator->start.hh, simulator->start.mm, simulator->start.sec, grx.week, grx.sec);
                gui_mvwprintw(LS_FIX, 5, 40, "xyz = %11.1f, %11.1f, %11.1f", xyz[iumd][0], xyz[iumd][1], xyz[iumd][2]);
                gui_mvwprintw(LS_FIX, 6, 40, "llh = %11.6f, %11.6f, %11.1f", llh[0] * R2D, llh[1] * R2D, llh[2]);
                start_y = 4;

                for (int i = 0; i < 33; i++)
                    sat_simulated[i] = false;

                for (int i = 0; i < MAX_CHAN; i++) {
                    if (chan[i].prn > 0) {
                        gui_mvwprintw(LS_FIX, start_y++, 1, "%02d %6.1f %5.1f %11.1f %5.1f", chan[i].prn,
                                chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
                    }

                    sat_simulated[chan[i].prn] = true;
                }

                gui_mvwprintw(LS_FIX, 3, 40, "Nav: %02d satellites", start_y - 4);
            }
        }
        // Update receiver time
        // grx = incGpsTime(grx, 0.100 + 0.0000001046);
        simulator->elapsed_ns += 1e8;
        grx = incGpsTime(g0, (double)(simulator->elapsed_ns + simulator->offset_ns) / 1e9);

        // Update time counter
        if (simulator->show_verbose && (igrx % 10 == 0)) {
            gps2date(&grx, &simulator->start);
            gui_mvwprintw(LS_FIX, 11, 57, "%4d/%02d/%02d,%02d:%02d:%02.9f (%d:%.0f)",
                          simulator->start.y, simulator->start.m, simulator->start.d, simulator->start.hh, simulator->start.mm, simulator->start.sec, grx.week, grx.sec);
        }
        gui_mvwprintw(LS_FIX, 12, 40, "Elapsed:         %5.9fs ", simulator->elapsed_ns / 1e9);
    }

    gui_status_wprintw(GREEN, "Simulation complete\n");

end_gps_thread:
    free(iq_buff);
    if (xyz)
        free(xyz);
    gui_status_wprintw(RED, "Exit GPS thread\n");
    simulator->gps_thread_exit = true;
    pthread_cond_signal(&(simulator->gps_init_done));
    pthread_exit(NULL);
}
