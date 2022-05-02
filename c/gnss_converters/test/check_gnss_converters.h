#ifndef CHECK_GNSS_CONVERTERS_H
#define CHECK_GNSS_CONVERTERS_H

typedef struct Suite Suite;

Suite *nmea_suite(void);
Suite *nmea_gpths_suite(void);
Suite *options_suite(void);
Suite *time_truth_suite(void);
Suite *rtcm_time_suite(void);
Suite *sbp_rtcm_suite(void);
Suite *utils_suite(void);

#endif /* CHECK_GNSS_CONVERTERS_H */
