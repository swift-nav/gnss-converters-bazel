/**
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/time_truth.h>

void time_truth_init(time_truth_t *instance) { (void)instance; }

bool time_truth_update(time_truth_t *instance,
                       enum time_truth_source source,
                       gps_time_t time) {
  (void)instance;
  (void)source;
  (void)time;
  return false;
}

void time_truth_get(time_truth_t *instance,
                    enum time_truth_state *state,
                    gps_time_t *time) {
  (void)instance;
  (void)state;
  (void)time;
}