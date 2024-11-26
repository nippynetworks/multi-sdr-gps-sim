/**
 * Download a file via curl
 *
 * Copyright © 2024 Nippy Networks
 * Distributed under the MIT License.
 *
 */

#ifndef DOWNLOAD_H
#define DOWNLOAD_H

#include <stdlib.h>

int download_file_to_disk(const char *url, const char *filename);

#endif /* DOWNLOAD_H */