import time
from machine import Pin, I2C, PWM, ADC
from umqttsimple import MQTTClient
from Learn import ssd1306
from Learn import tcs34725
from Learn import read_temp

# --- Constants ---
OLED_width, OLED_height = 128, 64 # OLED dimensions (probably)
SCL_PIN, SDA_PIN = 22, 23 # I2C pins for the OLED (SCL=Serial Clock Line, SDA=Serial Data Line)
OLED_addr, RGB_addr = 0x3c, 0x29 # hexidecimal addresses for the I2C devices (when scanning using i2c.scan() it gives us 61 and 40)
LED_PIN = 13
Temp_PIN = 36
Cooler_PIN = 27
Fan_PIN = 12
Log_file_name = "rgb_sensor_log.csv"


# --- Pin setup ---
led = Pin(LED_PIN, Pin.OUT)
led.value(0)
cooler = Pin(Cooler_PIN, Pin.OUT)  # Cooler pin
fan = Pin(Fan_PIN, Pin.OUT)  # Fan pin
pump_pwm = PWM(Pin(33), freq=1000)  # 1 kHz PWM on pin 33
pump_pwm.duty(0)  # Start with pump off

sub_pump_pwm = PWM(Pin(32), freq=1000)  # submersible pump on pin 32
sub_pump_pwm.duty(0)  # Start with pump off

