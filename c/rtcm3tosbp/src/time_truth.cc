#include <rtcm3tosbp/internal/time_truth.h>

static TimeTruth singleton_time_truth;
static TimeTruthCache singleton_time_truth_cache;

TimeTruth *rtcm_time_truth = &singleton_time_truth;
TimeTruthCache *rtcm_time_truth_cache = &singleton_time_truth_cache;
