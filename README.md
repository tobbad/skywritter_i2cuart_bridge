# skywritter_i2cuart_bridge
The aim of this project is to realize a simple i2c to uart bridge for the [skywritter}(https://shop.pimoroni.com/products/skywriter-hat) or [here](https://github.com/pimoroni/skywriter-hat). I do not use the RasPi head put the larger skywritter which just has a I2C interface with a reset and a transfer request line. 

The bridge is realized on a [STM32L432 Nucleo32 board](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-l432kc.html). The CubeMx files are included.
