RTC_DATA_ATTR uint64_t curr_time;

float demo_time = 10;
// float dur = 0.01;                              // hours
// float dur = demo_time/((float)hours_to_seconds);
float dur = 0.1;
unsigned int num_meas = 3;                             // num measurements to take in a set time
float time_sync_tol = 0.005; // factor
float mesh_sync_tol = 0.005; // factor
std::string num_text = std::to_string(dur);
std::string rounded = num_text.substr(0, num_text.find(".")+4);
String settings = String(rounded.c_str()) + ", " + String(num_meas) + ", " + String(time_sync_tol) + ", " + String(mesh_sync_tol);

String getTimes() {
    String times = "[";
    for (int i = 0; i < num_meas; i++) {
        times = times + String(dur*hours_to_seconds*(i+1)/num_meas);
        if (i < num_meas-1)
            times = times + ",";
    }
    times = times + "]";
    return times;
}

RTC_DATA_ATTR String cur_times;
