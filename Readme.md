# TPMS application for [Flipper Zero](https://flipperzero.one/)
[![FlipC.org](https://flipc.org/wosk/flipperzero-tpms/badge)](https://flipc.org/wosk/flipperzero-tpms)

## Features
- Read [TPMS](https://en.wikipedia.org/wiki/Tire-pressure_monitoring_system) sensors
- Decode Toyota PMV-C210/C215 sensors
- Filter received frames by minimum RSSI
- Select radio path explicitly: `Auto`, `Internal`, or `External`
- Relearn by 125kHz signal

####  Supported sensors
* Schrader GG4
* Toyota PMV-C210 / PMV-C215
* Abarth 124 (soon)

## How to use
In some circumstances TPMS sensors should transmit message periodically (car moving) or by event (emergency pressure reduction or temperature increase), so it can be caught.

While the car is stationary or sensor is not mounted into tire, Relearn mode can be enabled by emitting 125kHz signal. Keep sensor housing or valve near Flipper Zero`s back, like RFID card and push Right button to activate relearn signal for 1 second.

When sensor transmit message, you will see jumps of RSSI meter.
If sensor is supported and correct frequency and modulation was set, an item with Model and ID will be added for each sensor.

Pressing OK displays temperature and pressure.

Receiver configuration is available from the main receive screen:
- `Modulation` now includes the Toyota `PMV-C2x` preset
- `Min RSSI` can be used to ignore weak frames
- `Radio` can force `Internal` or `External` hardware instead of relying on `Auto`

![input](tpms.gif)

Feel free to contribute via PR or report issue

Code based on [weather station app](https://github.com/flipperdevices/flipperzero-good-faps/tree/dev/weather_station)
