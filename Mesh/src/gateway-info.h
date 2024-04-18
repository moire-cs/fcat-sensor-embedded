RTC_DATA_ATTR uint64_t curr_time;


float dur = 0.01;                              // hours
unsigned int num_meas = 2;                             // num measurements to take in a set time
float time_sync_tol = 0.005; // factor
float mesh_sync_tol = 0.005; // factor

String settings = String(dur) + ", " + String(num_meas) + ", " + String(time_sync_tol) + ", " + String(mesh_sync_tol);
