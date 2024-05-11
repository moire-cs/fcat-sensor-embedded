float demo_time = 10;
// float dur = 0.01;         up                     // hours
// float dur = demo_time/((float)hours_to_seconds);
float dur = 0.05;
unsigned int num_meas = 3;                             // num measurements to take in a set time
float time_sync_tol = 0.005; // factor
std::string num_text = std::to_string(dur);
std::string rounded = num_text.substr(0, num_text.find(".")+4);
String settings = String(rounded.c_str()) + ", " + String(num_meas) + ", " + String(time_sync_tol) + ", " + String(curr_time);

float time_factor = 1;

String getTimes() {
    time_factor = (curr_time - last_time)/(dur*hours_to_seconds);

    String times = "[";
    for (int i = 0; i < num_meas; i++) {
        times = times + String(dur*hours_to_seconds*(i+1)/num_meas);
        if (i < num_meas-1)
            times = times + ",";
    }
    times = times + "]";
    return times;
}

String buildTimeSyncMessage() {
    std::string num_text = std::to_string(dur);
    std::string rounded = num_text.substr(0, num_text.find(".")+4);
    return String(rounded.c_str()) + ", " + String(num_meas) + ", " + String(time_sync_tol) + ", " + String(curr_time);
}

RTC_DATA_ATTR String cur_times;
