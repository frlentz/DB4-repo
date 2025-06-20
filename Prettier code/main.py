import network
import sys
import time
from machine import Pin, I2C, PWM, ADC
from umqttsimple import MQTTClient
import config
from Learn import ssd1306
from Learn import tcs34725
from Learn import read_temp
from Devices import motors, I2Cs
from MQTT import MQTTfuncs
from PID_controller import PID
from Runs import offline_run, online_run

# --- Wi-Fi Setup ---
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(config.wifi_id, config.wifi_password)

# --- Constants ---
OLED_width, OLED_height = 128, 64 # OLED dimensions (probably)
SCL_PIN, SDA_PIN = 22, 23 # I2C pins for the OLED (SCL=Serial Clock Line, SDA=Serial Data Line)
OLED_addr, RGB_addr = 0x3c, 0x29 # hexidecimal addresses for the I2C devices (when scanning using i2c.scan() it gives us 61 and 40)
LED_PIN = 13
Temp_PIN = 36
Cooler_PIN = 27
Fan_PIN = 12
Client_ID = "esp32_rgb_project"
AIO_user = config.username # Needs to be configured in config.py
AIO_key = config.key # Needs to be configured in config.py
MQTT_broker = "io.adafruit.com"
MQTT_port = 1883
Log_file_name = "rgb_sensor_log.csv"
OLED_width=128
OLED_height=64

# --- MQTT Topics ---
TOPIC_LED = f"{AIO_user}/feeds/esp32-led-command"
TOPIC_R = f"{AIO_user}/feeds/esp32-r"
TOPIC_G = f"{AIO_user}/feeds/esp32-g"
TOPIC_B = f"{AIO_user}/feeds/esp32-b"
TOPIC_pump_speed = f"{AIO_user}/feeds/esp32-pump-speed"
TOPIC_sub_pump_speed = f"{AIO_user}/feeds/esp32-sub-pump-speed"  
TOPIC_temp = f"{AIO_user}/feeds/esp32-temp"

# --- Pin setup ---
led = Pin(LED_PIN, Pin.OUT)
led.value(0)
cooler = Pin(Cooler_PIN, Pin.OUT)  # Cooler pin
fan = Pin(Fan_PIN, Pin.OUT)  # Fan pin
pump_pwm = PWM(Pin(33), freq=1000)  # 1 kHz PWM on pin 33
pump_pwm.duty(0)  # Start with pump off

sub_pump_pwm = PWM(Pin(32), freq=1000)  # submersible pump on pin 32
sub_pump_pwm.duty(0)  # Start with pump off

#this file contains the offline and online runs

# --- Try to connect ---
wifi_connecting_time = 0
timeout = 20  # seconds
print("Connecting to WiFi", end="")

while not wifi.isconnected() and wifi_connecting_time < timeout:
    print(".", end="")
    wifi_connecting_time += 1
    time.sleep(1)

if wifi.isconnected():
    print("\nWiFi connected:", wifi.ifconfig())
    online_run(wifi, SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height, Temp_PIN, sub_pump_pwm)  # Call the online function
else:
    print("\nFailed to connect to WiFi after 20 seconds. Switching to offline mode.")
    offline_run(SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height, Temp_PIN, sub_pump_pwm, TOPIC_sub_pump_speed, TOPIC_temp, TOPIC_R, TOPIC_G, TOPIC_B, MQTT_broker, MQTT_port, AIO_user, AIO_key, Client_ID)  # Call the offline function

sys.exit()  # End this script after calling one of the modes

