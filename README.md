# MLX90614_SENSOR_Interfacing
This project demonstrates how to interface the MLX90614 infrared temperature sensor with the STM32L4 IoT Discovery Kit (STM32L4IOT1A) using the I2C protocol. The temperature data is printed via UART (e.g., Tera Term) for monitoring.

🔧 Hardware Used
STM32L4 IoT Discovery Kit (STM32L4IOT1A)

MLX90614 IR Temperature Sensor

Jumper Wires

USB to UART Converter (if not using onboard ST-LINK virtual COM port)

📡 Sensor Overview
MLX90614 is an IR thermometer that measures object and ambient temperature.

Communicates over I2C (Default address: 0x5A)

Temperature data is available in RAM register 0x07 (Object) and 0x06 (Ambient)

🧰 Software Requirements
STM32CubeIDE

STM32CubeMX (optional)

USB-UART terminal (Tera Term / PuTTY)

STM32 HAL Library

⚙️ Pin Connections
MLX90614	STM32L4IOT1A
VCC	3.3V
GND	GND
SDA	PB9 (I2C1 SDA)
SCL	PB8 (I2C1 SCL)

📝 How It Works
Initialize I2C1 and UART.

Read RAM register 0x07 from the MLX90614 using I2C.

Convert the 16-bit raw value to Celsius using the formula:

𝑇
=
RAW_DATA
×
0.02
1
−
273.15
T= 
1
RAW_DATA×0.02
​
 −273.15
Send the result via UART every 1 second.

