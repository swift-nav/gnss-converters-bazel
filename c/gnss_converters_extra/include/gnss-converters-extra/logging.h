/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_EXTRA_LOGGING_H
#define GNSS_CONVERTERS_EXTRA_LOGGING_H

#include <swiftnav/logging.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function type use by the "gnss_converters_extra_setup_logging". The
 * first argument corresponds to the logging level that is being provided,
 * values are reflective of the libswiftnav's log level. The second parameter is
 * a string representation of the message.
 */
typedef void (*gnss_converters_extra_logging_callback_t)(int, const char*);

/**
 * Installs a callback function which is invoked anytime there is a logging
 * message generated via Swift Navigation's logging framework.
 *
 * @param callback function to set as the callback function
 */
void gnss_converters_extra_setup_logging(
    gnss_converters_extra_logging_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_CONVERTERS_EXTRA_LOGGING_H
