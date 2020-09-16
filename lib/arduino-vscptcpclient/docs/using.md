# Using the vscptcpclient library

To help you up to speed with the development of VSCP Client applications you can find a couple of examples [here](https://github.com/grodansparadis/arduino-vscptcpclient/tree/master/examples) that uses most of the functionality available in the library or you can check _File > Examples > VscpTcpClient_ within the Arduino application or the example folder.

For platform IO you can add 

```ini
lib_deps =
  https://github.com/grodansparadis/arduino-vscptcpclient
```
to your platform.ini file and the library will be downloaded for you and is easy to keep updated to the latest version.

A typical platformio.ini file look like this for esp8266 development using Platform IO

```ini
[env:esp01]
platform = espressif8266
framework = arduino
board = esp12e
platform_packages = 
upload_port = /dev/ttyUSB2
monitor_port = /dev/ttyUSB2
monitor_speed = 115200
lib_deps =
  https://github.com/grodansparadis/arduino-vscptcpclient
```

[filename](./bottom_copyright.md ':include')