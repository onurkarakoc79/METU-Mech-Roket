# METU Mech Rocket Team Electronic Cards and Codes
This repository contains the electronic codes and card designs for the METU Mech Rocket Team's rocket system. The system consists of two card designs, each equipped with an Arduino Due controller.

## Communication Card
The first Arduino Due is responsible for communication and utilizes the following modules:

SX1262 LoRa module
SIM800L module
CH376S USB module
The communication card enables data transfer with the ground station. The SX1262 LoRa module and SIM800L module are used for wireless communication, while the CH376S USB module handles data storage to a USB device.

## Processor Card
The second Arduino Due is responsible for data processing and incorporates the following components:

BME280 sensor
Dallas temperature sensor
BMP180 sensor
NEO M8N GPS module
Parachute opening system (controlled by a servo and a transistor that fires the gas tank)
The processor card collects data related to humidity, temperature, altitude, and absolute location. It supports two sensors for each type of data, such as BME280 and BMP180. The collected data is processed using a Kalman filter algorithm for improved accuracy. The processed data is then transferred to the communication card via serial communication.

## Card Designs
This repository also includes the necessary files for the card designs. The provided KiCad files contain the card designs and necessary footprints. To use the designs, add the footprint files to KiCad in the appropriate manner. The card designs have been configured for use with either Arduino Due or Arduino Mega Due, based on the pin configuration.

