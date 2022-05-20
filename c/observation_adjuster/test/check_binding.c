#include <observation_adjuster/binding.h>

int main() {
  double vrs_ecef[3] = {0};
  uint8_t msm_output_type = 0;

  void* obs_adjuster = obs_adjuster_extern_new(vrs_ecef, msm_output_type);

  uint8_t buffer[1];
  const size_t buffer_size = sizeof(buffer) / sizeof(buffer[0]);

  obs_adjuster_extern_read_station_observations(
      obs_adjuster, 0, 0, buffer, buffer_size);
  obs_adjuster_extern_read_station_corrections(
      obs_adjuster, 0, 0, buffer, buffer_size);
  obs_adjuster_extern_read_receiver_corrections(
      obs_adjuster, 0, 0, buffer, buffer_size);
  obs_adjuster_extern_write_VRS_corrections_SBP(
      obs_adjuster, buffer, buffer_size);
  obs_adjuster_extern_write_VRS_corrections_RTCM(
      obs_adjuster, buffer, buffer_size);

  obs_adjuster_extern_delete(obs_adjuster);

  return 0;
}
