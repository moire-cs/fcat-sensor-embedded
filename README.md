
## Plan:
- Create sync function to sync each node to a particular time
- Measure data 4x per day, and send to gateway
- For pulse: measure time between pulses
- Determine power to cover distance
    - Rerun power analysis
    - Do we want dynamic power to radio?
How to build PlatformIO based project
=====================================

```shell
# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e esp32dev

# Upload firmware for the specific environment
$ pio run -e esp32dev --target upload

# Clean build files
$ pio run --target clean
```

Development note: Currently, the project is configured to autolaunch the arduino code on startup and run setup() and loop() functions. Alternatively, you can run the code manually by spawning the process.

We are going to want to disable the auto-launch feature in the future as we will want threads with and without arduino code. This can be done in `sdkconfig.defaults` by setting `CONFIG_AUTOSTART_ARDUINO` to `n`.

