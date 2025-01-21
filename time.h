#ifndef TIME_H
#define TIME_H

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

/* Structure representing GPS time */
typedef struct {
    int week; /* GPS week number (since January 1980) */
    double sec; /* second inside the GPS \a week */
} gpstime_t;

/* Structure repreenting UTC time */
typedef struct {
    int y; /* Calendar year */
    int m; /* Calendar month */
    int d; /* Calendar day */
    int hh; /* Calendar hour */
    int mm; /* Calendar minutes */
    double sec; /* Calendar seconds */
} datetime_t;

double subGpsTime(gpstime_t g1, gpstime_t g0);
gpstime_t incGpsTime(gpstime_t g0, double dt);
void date2gps(const datetime_t *t, gpstime_t *g);
void gps2date(const gpstime_t *g, datetime_t *t);

#endif // TIME_H
