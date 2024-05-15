float demo_time = 10;
// float dur = 0.01;         up                     // hours
// float dur = demo_time/((float)hours_to_seconds);
RTC_DATA_ATTR float dur = 0.05;
RTC_DATA_ATTR unsigned int num_meas = 3;                             // num measurements to take in a set time
RTC_DATA_ATTR float time_sync_tol = 0.005; // factor
RTC_DATA_ATTR int sync_dur = 30 * 1000;
std::string num_text = std::to_string(dur);
std::string rounded = num_text.substr(0, num_text.find(".")+4);
std::string time_tol_text = std::to_string(time_sync_tol);
std::string rounded_tol = time_tol_text.substr(0, time_tol_text.find(".") + 4);
String settings = String(rounded.c_str()) + ", " + String(num_meas) + ", " + String(rounded_tol.c_str()) + ", " + String(sync_dur) + ", " + String(curr_time);

RTC_DATA_ATTR float time_factor = 1;

String getTimes(unsigned int time_duration) {
    if (last_time != 0) {
        Serial.printf("Curr Time: %d\n", curr_time);
        Serial.printf("Last Time: %d\n", last_time);
        Serial.printf("Denom: %d\n", (dur*hours_to_seconds));
        time_factor = (curr_time - last_time)/(dur*hours_to_seconds);  
    }
    

    String times = "[";
    for (int i = 0; i < num_meas; i++) {
        times = times + String(time_duration*hours_to_seconds*(i+1)/num_meas + last_time);
        if (i < num_meas-1)
            times = times + ",";
    }
    times = times + "]";
    return times;
}

String buildTimeSyncMessage() {
    std::string num_text = std::to_string(dur);
    std::string rounded = num_text.substr(0, num_text.find(".")+4);

    std::string time_tol_text = std::to_string(time_sync_tol);
    std::string rounded_tol = time_tol_text.substr(0, time_tol_text.find(".") + 4);
    return String(rounded.c_str()) + ", " + String(num_meas) + ", " + String(rounded_tol.c_str()) + ", " + String(sync_dur) + ", " + String(curr_time);
}

RTC_DATA_ATTR String cur_times;
