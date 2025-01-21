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
#include "gps-sim.h"
#include "almanac.h"
#include "download.h"

static almanac_gps_t almanac_gps;

struct sem_file {
    const char *filename;
    FILE *stream;
};

/**
 * Initialize empty almanac.
 */
almanac_gps_t* almanac_init(void) {
    almanac_gps.valid = 0;
    almanac_gps.almanac_time.week = 0;
    almanac_gps.almanac_time.sec = 0;

    almanac_prn_t* a = NULL;
    for (int prn = 0; prn < MAX_SAT; prn++) {
        a = &almanac_gps.sv[prn];
        a->ura = 0;
        a->health = 0;
        a->config_code = 0;
        a->svid = 0;
        a->svn = 0;
        a->valid = 0;
        a->toa.sec = 0.0;
        a->toa.week = 0;
        a->e = 0.0;
        a->delta_i = 0.0;
        a->omegadot = 0.0;
        a->sqrta = 0.0;
        a->omega0 = 0.0;
        a->aop = 0.0;
        a->m0 = 0.0;
        a->af0 = 0.0;
        a->af1 = 0.0;
    }
    return &almanac_gps;
}

/**
 * Read almanac from local file.
 * sem format expected.
 */
CURLcode almanac_read_file(char* almanac_file_name) {
    char buf[100];
    char title[25];
    char *pbuf;
    unsigned int n, week, sec, id;
    almanac_gps_t* almanac = &almanac_gps;

    FILE *fp = fopen(almanac_file_name, "rt");
    almanac_prn_t* a = NULL;

    // Start with empty almanac
    almanac_init();

    if (!fp) {
        return CURLE_READ_ERROR;
    }

    pbuf = fgets(buf, sizeof (buf), fp);
    if (pbuf == NULL || sscanf(buf, "%u %24s", &n, title) != 2) goto error;

    pbuf = fgets(buf, sizeof (buf), fp);
    if (pbuf == NULL || sscanf(buf, "%u %u", &week, &sec) != 2) goto error;
    // GPS week rollover
    week += 2048;

    almanac->almanac_time.week = (int) week;
    almanac->almanac_time.sec = (double) sec;

    n -= 1; // PRN in file counts 1-32, array counts 0-31
    if (n > 31) n = 31; // Max 32 PRN's to read (0-31)

    for (unsigned int j = 0; j <= n; j++) {
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        // Check and skip blank line
        if (buf[0] == '\n' || buf[0] == '\r') {
            pbuf = fgets(buf, sizeof (buf), fp);
            if (pbuf == NULL) goto error;
        }

        if (sscanf(buf, "%u", &id) != 1) goto error;
        if (id == 0) id = 1;
        if (id > 32) id = 32;

        a = &almanac->sv[id - 1];

        a->svid = (unsigned short) id;

        // SVN is optional, could be a blank line
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (buf[0] == '\n' || buf[0] == '\r') {
            a->svn = 0;
        } else {
            if (sscanf(buf, "%hu", &a->svn) != 1) goto error;
        }

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->ura) != 1) goto error;
        if (a->ura > 15) a->ura = 15;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->e, &a->delta_i, &a->omegadot) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->sqrta, &a->omega0, &a->aop) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->m0, &a->af0, &a->af1) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->health) != 1) goto error;
        if (a->health > 63) a->health = 63;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->config_code) != 1) goto error;
        if (a->config_code > 15) a->config_code = 15;

        a->toa.week = (int) week;
        a->toa.sec = (double) sec;

        a->valid = 1;
        almanac->valid = 1; // We have at least one valid record
    }
    fclose(fp);
    return CURLE_OK;

error:
    if (!feof(fp)) {
        // Not end of file, something wrong
        // Drop all parsed almanac data on error
        almanac_init();
    }
    /**
     * If this is the end of file we may read less records than what 
     * field "number of records" announced.
     * Found this happening when saving almanac.sem in ublox u-center software.
     */
    fclose(fp);
    return CURLE_READ_ERROR;
}

/**
 * Read almanac from online source.
 * sem format expected.
 *
 */
CURLcode almanac_download(char* almanac_file_name) {
    CURLcode res;

    res = download_file_to_disk(ALMANAC_DOWNLOAD_SEM_URL, almanac_file_name);
    if (res != CURLE_OK) {
        return res;
    }
    return (almanac_read_file(almanac_file_name));
}
