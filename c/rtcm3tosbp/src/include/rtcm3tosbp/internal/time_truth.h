#ifndef RTCM3TOSBP_INTERNAL_TIME_TRUTH_H
#define RTCM3TOSBP_INTERNAL_TIME_TRUTH_H

#include <gnss-converters/time_truth_v2.h>

#ifdef __cplusplus
extern "C" {
#endif

extern TimeTruth *rtcm_time_truth;
extern TimeTruthCache *rtcm_time_truth_cache;

#ifdef __cplusplus
}
#endif

#endif  // RTCM3TOSBP_INTERNAL_TIME_TRUTH_H
