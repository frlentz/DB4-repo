import network
import sys
import time
from machine import Pin, I2C, PWM, ADC
import math
from umqttsimple import MQTTClient
import config
from Learn import ssd1306
from Learn import tcs34725
import os
from Learn import read_temp

# --- Wi-Fi Setup ---
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(config.wifi_id, config.wifi_password)

print("Connecting to WiFi...", end="")
while not wifi.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWiFi connected" if wifi.isconnected() else "\nFailed to connect")


# --- Constants ---
OLED_width, OLED_height = 128, 64 # OLED dimensions (probably)
SCL_PIN, SDA_PIN = 22, 23 # I2C pins for the OLED (SCL=Serial Clock Line, SDA=Serial Data Line)
OLED_addr, RGB_addr = 0x3c, 0x29 # hexidecimal addresses for the I2C devices (when scanning using i2c.scan() it gives us 61 and 40)
LED_PIN = 13
Temp_PIN = 36
Client_ID = "esp32_rgb_project"
AIO_user = config.username # Needs to be configured in config.py
AIO_key = config.key # Needs to be configured in config.py
MQTT_broker = "io.adafruit.com"
MQTT_port = 1883
Log_file_name = "rgb_sensor_log.csv"


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
pump_pwm = PWM(Pin(33), freq=1000)  # 1 kHz PWM on pin 33
pump_pwm.duty(0)  # Start with pump off

sub_pump_pwm = PWM(Pin(32), freq=1000)  # submersible pump on pin 32
sub_pump_pwm.duty(0)  # Start with pump off


# --- Pump definitions ---
def set_pump_speed(percent):
    percent = max(0, min(percent, 100))
    # Remap to usable PWM range: 0–100% → 0.7–1.0
    if percent == 0:
        pwm_value = 0
    else:
        scaled = 0.7 + (percent / 100) * 0.3
        pwm_value = int(scaled * 1023)
    pump_pwm.duty(pwm_value)
    print(f"Pump speed set to {percent}% → PWM: {pwm_value}/1023")


# Control submersible pump speed (0–100%)
def set_sub_pump_speed(percent):
    percent = max(0, min(percent, 100))
    if percent == 0:
        pwm_value = 0
    else:
        scaled = 0.7 + (percent / 100) * 0.3
        pwm_value = int(scaled * 1023)  
    sub_pump_pwm.duty(pwm_value)
    print(f"Submersible pump speed set to {percent}% → PWM: {pwm_value}/1023")


# --- Initializing I2C devices ---
def init_i2c_devices():
    i2c = I2C(1, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000) # Initializing the I2C bus once for all devices
    oled = None
    rgb = None

    # Scanning for I2C devices
    print('Scanning I2C bus for devices...')
    devices = i2c.scan()
    print(f'Detected I2C devices: {[hex(d) for d in devices]}')

    if OLED_addr in devices: # Initial display on OLED
        oled = ssd1306.SSD1306_I2C(OLED_width, OLED_height, i2c, addr=OLED_addr)
        oled.fill(0)
        oled.text("Hello,", 0, 0) # The second parameter is the x position, third is y
        oled.text("DB4 Project", 0, 16)
        oled.text("Starting...", 0, 32)
        oled.show()

    if RGB_addr in devices:
        rgb = tcs34725.TCS34725(i2c) # Initializing RGB sensor
        rgb.integration_time(400)  # This tells the sensor how long to collect light before converting it into a digital reading. In this case 154 milliseconds. Must be one of the allowed gain values: 1, 4, 16, or 60. Higher gain amplifies the signal more. Useful in dim light to get stronger readings
        rgb.gain(4)                # Higher gain amplifies the signal more.
    return oled, rgb


# --- MQTT Callback ---
def mqtt_callback(topic, message):
    topic_str = topic.decode()
    msg_str = message.decode().lower()

    if topic_str == TOPIC_LED:
        led.value(1 if msg_str == "on" else 0)

    elif topic_str == TOPIC_pump_speed:
        try:
            percent = int(msg_str)
            set_pump_speed(percent)

        except ValueError as e: 
            print("Invalid speed value for main pump")
            sys.print_exception(e) #
'''
    elif topic_str == TOPIC_sub_pump_speed:
        try:
            percent = int(msg_str)
            set_sub_pump_speed(percent)
        except:
            print("Invalid speed value for submersible pump")
'''
# --- MQTT Connect ---
def connect_mqtt():
    client = MQTTClient(Client_ID, MQTT_broker, port=MQTT_port, user=AIO_user, password=AIO_key)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_LED.encode())
    client.subscribe(TOPIC_pump_speed.encode())
    client.subscribe(TOPIC_sub_pump_speed.encode())
    return client

# --- PID controller ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(0, 100)): #controls pump speed as a percentage
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits  # (min, max)
        self.integral = 0
        self.last_error = 0
        self.last_time = time.ticks_ms()

    def compute(self, measurement):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000  # convert ms to seconds, we need to make sure that this parameter is not too small
        #maybe add better error handling for when dt is small in order to avoid division by 0
        error = measurement - self.setpoint 

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt 
        I = self.Ki * self.integral #the integral is accumulated as sum of error

        # Derivative term
        derivative = 0
        if dt > 0:
            derivative = (error - self.last_error) / dt #approximation of the derivative of an error
        D = self.Kd * derivative 

        # Save for next iteration
        self.last_error = error
        self.last_time = now

        # Compute output and clamp to limits
        output = P + I + D
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        return output

# --- Main ---
oled = None
rgb = None
mqtt_client = None
temp_sens = None
last_temp_publish_time = 0 
temp_publish_interval = 30 # We only want a temp readiing every 30 seconds
data_for_esp=("")

#Example constants for the PID controller
#these need to be from the web server
Kp = 2.0
Ki = 0.1
Kd = 1.0
target_temp = 17.5

try:
    oled, rgb = init_i2c_devices()
    mqtt_client = connect_mqtt()
    last_ping = time.time()
    ping_interval = 60
    temp_sens = read_temp.init_temp_sensor(Temp_PIN)
    pid_temp = PID(Kp, Ki, Kd, setpoint=target_temp, output_limits=(0, 100))  # 0-100% pump speed

    if oled:
        time.sleep(3) # Lets the OLED "Starting..." message show for 3 seconds

    while True:
        mqtt_client.check_msg()

        # This part is to send keep-alive pings to the MQTT broker so the connection stays alive
        if (time.time() - last_ping) > ping_interval:
            mqtt_client.ping()
            last_ping = time.time()

        if oled:
            oled.fill(0) # Clears display (black background)
            oled.text("Time: " + str(int(time.time())), 0, 0)

        if temp_sens and (time.time() - last_temp_publish_time) > temp_publish_interval:
            try:
                current_temp = read_temp.read_temp(temp_sens)
                print(f"Current Temperature: {current_temp:.2f} °C") # Print to console

                # Compute PID output
                sub_pump_speed = pid_temp.compute(current_temp)

                # Apply the computed speed
                set_sub_pump_speed(sub_pump_speed)

                # publish PID output
                mqtt_client.publish(TOPIC_pump_speed.encode(), str(int(sub_pump_speed)))
                data=str(current_temp) 
                data_for_esp=str(str(time.time()) + ": " + data + ", ")

                #Logging the data
                f = open('data.txt', 'a')
                f.write(data_for_esp)
                f.close() 
                # To see it: >>> f = open("data.txt")  >>> f.read(


                if mqtt_client: # Only publish if MQTT client is connected
                    mqtt_client.publish(TOPIC_temp.encode(), str(f"{current_temp:.2f}"))
                
                last_temp_publish_time = time.time() # Update the timer after successful read/publish

            except Exception as e:
                print(f"Temperature Sensor Error:")
                sys.print_exception(e) 


        if rgb: 
            try:
                r, g, b, c = rgb.read(raw=True) # These are the direct 16-bit numbers that the sensor's Analog-to-Digital Converter (ADC) produces for each color channel (Red, Green, Blue) and the Clear (unfiltered) channel.
                                                # they range from 0 (no light detected) up to 65535 (maximum light detected).
                if oled:
                    oled.text(f"R:{r} G:{g}", 0, 16) # Red, Green, Blue, and Clear values
                    oled.text(f"B:{b} C:{c}", 0, 32)
                    oled.show()

                # Publish the RGB values at a safe rate
                mqtt_client.publish(TOPIC_R.encode(), str(r))
                mqtt_client.publish(TOPIC_G.encode(), str(g)) 
                mqtt_client.publish(TOPIC_B.encode(), str(b))


            except Exception as e:
                print(f"RGB Sensor Error:") 
                sys.print_exception(e) 
                if oled:
                    oled.text("RGB Error", 0, 16)

        # This part is to send keep-alive pings to the MQTT broker so the connection stays alive
        sleep_duration = 15
        sleep_start = time.time()
        while time.time() - sleep_start < sleep_duration:
            mqtt_client.check_msg()
            time.sleep(0.2)

except Exception as e:
    print("Fatal Error caught in main loop:") 
    sys.print_exception(e) 
    if oled:
        oled.fill(0)
        oled.text("FATAL ERROR", 0, 0)
        oled.show()
        time.sleep(5)

finally:
    if mqtt_client:
        mqtt_client.disconnect()
    if wifi.isconnected():
        wifi.disconnect()
    if oled:
        oled.fill(0)
        oled.text("Goodbye", 0, 0)
        oled.show()

