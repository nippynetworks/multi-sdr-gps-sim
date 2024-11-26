/**
 * Download a file via curl
 *
 * Copyright © 2024 Nippy Networks
 * Distributed under the MIT License.
 *
 */

#include "download.h"

#include <curl/curl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zlib.h>

#include "gui.h"

/* Structure to hold data downloaded from the server */
struct MemoryStruct {
    char *memory;
    size_t size;
};

/* Function to write data to a file on disk */
int write_data_to_file(const char *filename, const void *data, size_t size) {
    FILE *fp = fopen(filename, "wb");
    if (fp == NULL) {
        gui_status_wprintw(RED, "Failed to open file for writing: %s\n", filename);
        perror("Failed to open file for writing");
        return CURLE_GOT_NOTHING;
    }

    size_t written = fwrite(data, 1, size, fp);
    if (written != size) {
        gui_status_wprintw(RED, "Failed to write all data to file: %s\n", filename);
        fclose(fp);
        return CURLE_GOT_NOTHING;
    }

    fclose(fp);
    return CURLE_OK;
}

/* Callback function for libcurl to write data into memory */
static size_t write_memory_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    char *ptr = realloc(mem->memory, mem->size + realsize + 1);
    if (ptr == NULL) {
        gui_status_wprintw(RED, "Not enough memory (realloc returned NULL)\n");
        return 0;  // Out of memory
    }

    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;  // Null-terminate the string

    return realsize;
}

/* Function to download a file over HTTPS into file */
int download_file_to_disk(const char *url, const char *filename) {
    CURL *curl_handle;
    CURLcode res;
    long http_code = 0;

    struct MemoryStruct chunk = {malloc(1), 0};

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl_handle = curl_easy_init();
    if (!curl_handle) {
        gui_status_wprintw(RED, "Failed to initialize curl\n");
        return CURLE_GOT_NOTHING;
    }

    // Set the URL
    curl_easy_setopt(curl_handle, CURLOPT_URL, url);
    // Set common options
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, write_memory_callback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&chunk);
    curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "multi-sdr-gps-sim/1.0");
    curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);
    // SSL Options (Ensure verification is enabled. Should be defaults)
    curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 2L);
    // Instruct libcurl to use the native CA store
    curl_easy_setopt(curl_handle, CURLOPT_SSL_OPTIONS, CURLSSLOPT_NATIVE_CA);
    // Transform 400/500 responses into errors
    curl_easy_setopt(curl_handle, CURLOPT_FAILONERROR, 1L);
    // Set a sensible timeout, don't wait forever
    curl_easy_setopt(curl_handle, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 20L);

    // Perform the request
    res = curl_easy_perform(curl_handle);

    // Check if request was successful
    curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);
    if (res == CURLE_OK) {
        if (http_code == 200) {
            // Success, write it to disk
            res = write_data_to_file(filename, chunk.memory, chunk.size);
        } else {
            res = CURLE_HTTP_RETURNED_ERROR;
        }
    } else {
        gui_status_wprintw(RED, "failed: %s\n", curl_easy_strerror(res));
    }

    /* cleanup */
    curl_easy_cleanup(curl_handle);
    curl_global_cleanup();
    free(chunk.memory);

    return res;
}
