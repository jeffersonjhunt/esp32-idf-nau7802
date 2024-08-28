ESP32 IDF NAU7802 Library
===========================================================

 [![Nuvoton NAU7802](https://www.nuvoton.com/export/sites/nuvoton/images/Audio/DS_NAU7802SGI_BlockDiagram_N.png_1181042230.png)](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU7802_DataSheet_EN_Rev2.4.pdf) 

 This NAU7802 ESP32 IDF driver allows you to connect the Nuvoton NAU7802 to easily read load cells to measure weight. By connecting the amplifier to your ESP32 microcontroller you will be able to read the changes in the resistance of the load cell, and with some calibration you'll be able to get very accurate weight measurements. This can be handy for creating your own industrial scale, process control or simple presence detection.

A breakout board for prototyping is avaialbe from:

- [SparkFun Qwiic Scale - NAU7802](https://www.sparkfun.com/products/15242)
- [Adafruit NAU7802 24-Bit ADC - STEMMA QT / Qwiic](https://www.adafruit.com/product/4538)

Thanks to:

- SparkFun and Nathan Seidle for the [Arduino driver](https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library/) code on which this driver is based.

Repository Contents
-------------------

- **/examples** - Example code for the library.
- **/licenses** - Attributions.

Documentation
--------------

- **[IDF Component Registry](https://components.espressif.com)** - Espressif Component Registry
- **[IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/en/latest/index.html)** - Information on the Espressif IDF Component Manager.
- **[Product documentation](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU7802_DataSheet_EN_Rev2.4.pdf)** - Datasheet including I2C documentation.

Notable Changes & Improvements
------------------------------

- **[Espressif IoT Development Framework](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)** - based on v5 of the ESP-IDF.
- **[Utilizes the new I2C master/slave drivers](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html)**
- **[Support for DRDY](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU7802_DataSheet_EN_Rev2.4.pdf)** - Data Ready Output monitoring to avoid unessary pulling of the I2C bus.

License Information
-------------------

This library is _**open source**_! 

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.
