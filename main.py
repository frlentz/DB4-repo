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
Cooler_PIN = 27
Fan_PIN = 12
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
TOPIC_P = f"{AIO_user}/feeds/esp32-p"
TOPIC_I = f"{AIO_user}/feeds/esp32-i"
TOPIC_D = f"{AIO_user}/feeds/esp32-d"


# --- Pin setup ---
led = Pin(LED_PIN, Pin.OUT)
led.value(0)
cooler = Pin(Cooler_PIN, Pin.OUT)  # Cooler pin
fan = Pin(Fan_PIN, Pin.OUT)  # Fan pin
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
    # Remap to usable PWM range: 0–100% → 0.3–1
    if percent == 0:
        pwm_value = 0
    else:
        scaled = 0.4 + (percent / 100) * 0.6
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
            sys.print_exception(e) 

    elif topic_str == TOPIC_P:
        try:
            pid_temp.Kp = float(msg_str)
            print(f"P coefficient set to {pid_temp.Kp}")
        except ValueError as e:
            print("Invalid Kp value")
            sys.print_exception(e)

    elif topic_str == TOPIC_I:
        try:
            pid_temp.Ki = float(msg_str)
            print(f"I coefficient set to {pid_temp.Ki}")
        except ValueError as e:
            print("Invalid Ki value")
            sys.print_exception(e)

    elif topic_str == TOPIC_D:
        try:
            pid_temp.Kd = float(msg_str)
            print(f"D coefficient set to {pid_temp.Kd}")
        except ValueError as e:
            print("Invalid Kd value")
            sys.print_exception(e)

# --- MQTT Connect ---
def connect_mqtt():
    client = MQTTClient(Client_ID, MQTT_broker, port=MQTT_port, user=AIO_user, password=AIO_key)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_LED.encode())
    client.subscribe(TOPIC_pump_speed.encode())
    client.subscribe(TOPIC_sub_pump_speed.encode())
    client.subscribe(TOPIC_P.encode())
    client.subscribe(TOPIC_I.encode())
    client.subscribe(TOPIC_D.encode())
    return client

# --- PID controller ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(0, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral = 0
        self.last_error = 0
        self.last_time = time.ticks_ms()
        self.last_output = 0 # To store the last computed output for dt=0 case

    def compute(self, measurement):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000

        # --- DEBUG PRINTS START (at start of compute) ---
        print(f"PID Debug (Setpoint={self.setpoint:.2f}, Meas={measurement:.2f}):")
        # --- DEBUG PRINTS END ---

        if dt == 0:
            print("  dt=0, returning last output.")
            return self.last_output

        error = measurement - self.setpoint

        P = self.Kp * error

        derivative_term = 0
        if dt > 0:
            derivative_term = (error - self.last_error) / dt

        # Calculate the potential output based on P, current I, and D (before clamping)
        unclamped_output = P + (self.Ki * self.integral) + (self.Kd * derivative_term)

        # --- ANTI-WINDUP LOGIC (Conditional Integration) ---
        # Integrate only if the unclamped output is within limits,
        # OR if it's outside limits but the error is trying to bring it back within limits.
        
        integrate_cond = False
        if (unclamped_output >= self.output_limits[0] and unclamped_output <= self.output_limits[1]):
            integrate_cond = True # Case 1: Within limits, always integrate
        elif (unclamped_output > self.output_limits[1] and error < 0):
            integrate_cond = True # Case 2: Above max, but error is trying to reduce it
        elif (unclamped_output < self.output_limits[0] and error > 0):
            integrate_cond = True # Case 3: Below min, but error is trying to increase it

        if integrate_cond:
            self.integral += error * dt
        # --- END ANTI-WINDUP LOGIC ---

        I = self.Ki * self.integral # Recalculate I for the actual output and debug prints
        D = self.Kd * derivative_term # Corrected to use derivative_term

        # Final output calculation
        output = P + I + D
        clamped_output = max(self.output_limits[0], min(output, self.output_limits[1]))

        self.last_error = error
        self.last_time = now
        self.last_output = clamped_output

        if current_temp <= self.setpoint:
            cooler.value(0)
            fan.value(0)
        else:
            cooler.value(1)
            fan.value(1)

        '''
        # --- DEBUG PRINTS START (at end of compute) ---
        print(f"  Error={error:.2f}, dt={dt:.2f}, Integral={self.integral:.2f}")
        print(f"  P={P:.2f}, I={I:.2f}, D={D:.2f}")
        print(f"  Unclamped Output={unclamped_output:.2f}, Clamped Output={clamped_output:.2f}")
        # --- DEBUG PRINTS END ---
        '''

        return clamped_output

# --- Main ---
oled = None
rgb = None
mqtt_client = None
temp_sens = None

# Timing variables
last_temp_read_time = 0
temp_read_interval = 5 # Read temperature every 5 seconds

last_rgb_read_time = 0
rgb_read_interval = 5 # Read RGB every 5 seconds

last_adafruit_publish_time = 0
adafruit_publish_interval = 15 # Publish to Adafruit every 15 seconds

last_ping = time.time()
ping_interval = 60 # MQTT Pings every 60 seconds

# Variables to hold the latest sensor readings and pump speed
latest_temp = None
latest_sub_pump_speed = 0
latest_r, latest_g, latest_b, latest_c = 0, 0, 0, 0


# Example constants for the PID controller
Kp_mus = 2.0
Ki_mus = 0.1
Kd_mus = 1.0
target_temp = 20.5


try:
    oled, rgb = init_i2c_devices()
    mqtt_client = connect_mqtt()
    temp_sens = read_temp.init_temp_sensor(Temp_PIN)
    pid_temp = PID(Kp_mus, Ki_mus, Kd_mus, setpoint=target_temp, output_limits=(0, 100))

    if oled:
        time.sleep(3) # Lets the OLED "Starting..." message show for 3 seconds

    while True:
        mqtt_client.check_msg()

        # --- MQTT Ping (runs independently every 60 seconds) ---
        if (time.time() - last_ping) > ping_interval:
            mqtt_client.ping()
            last_ping = time.time()

        # --- OLED Time Update all the time ---
        if oled:
            oled.fill(0) # Clears display
            oled.text("Time: " + str(int(time.time())), 0, 0)


        # --- Temperature Sensor Reading & Pump Control every 5 seconds ---
        if temp_sens and (time.time() - last_temp_read_time) > temp_read_interval:
            try:
                current_temp = read_temp.read_temp(temp_sens)
                print(f"Current Temperature: {current_temp:.2f} °C")
                
                # Compute PID output for submersible pump based on temperature
                sub_pump_speed_calculated = pid_temp.compute(current_temp)

                # Apply the computed speed IMMEDIATELY after calculation
                set_sub_pump_speed(sub_pump_speed_calculated)
                
                # Store latest values for later publishing to Adafruit
                latest_temp = current_temp
                latest_sub_pump_speed = sub_pump_speed_calculated

                # Log temperature data to file immediately after reading
                data_for_log = str(f"{int(time.time())}: {current_temp:.2f}, ")
                with open('data.txt', 'a') as f:
                    f.write(data_for_log)

                last_temp_read_time = time.time()

            except Exception as e:
                print(f"Temperature Sensor Error:")
                sys.print_exception(e)
                # You might want to update OLED here too for errors
                if oled:
                    oled.text("Temp Error", 0, 48) # Display error on OLED
                    oled.show()


        # --- RGB Sensor Reading Every 5 seconds ---
        if rgb and (time.time() - last_rgb_read_time) > rgb_read_interval:
            try:
                r, g, b, c = rgb.read(raw=True)
                
                # Store latest values for later publishing to Adafruit
                latest_r, latest_g, latest_b, latest_c = r, g, b, c

                # Update OLED with RGB data immediately after reading
                if oled:
                    oled.text(f"R:{r} G:{g}", 0, 16)
                    oled.text(f"B:{b} C:{c}", 0, 32)
                    oled.show() # Update OLED with current time, then with RGB
                
                # Log RGB data to file immediately after reading
                rgb_log_entry = f"{int(time.time())},{r},{g},{b},{c}\n"
                file = open("rgb_log.csv", "a")  #or data.txt?
                file.write(rgb_log_entry)

                last_rgb_read_time = time.time()

            except Exception as e:
                print(f"RGB Sensor Error:")
                sys.print_exception(e)
                if oled:
                    oled.text("RGB Error", 0, 16)
                    oled.show()


        # --- Adafruit Publishing every 15 seconds ---
        if mqtt_client and (time.time() - last_adafruit_publish_time) > adafruit_publish_interval:
            
            # Publish last known temperature if available
            if latest_temp is not None:
                mqtt_client.publish(TOPIC_temp.encode(), str(f"{latest_temp:.2f}"))
                mqtt_client.publish(TOPIC_sub_pump_speed.encode(), str(int(latest_sub_pump_speed)))
            
            # Publish last known RGB values if available
            # Check if latest_r is not 0 (initial value) or if r has been updated.
            # Using 'is not None' for latest_temp is better if 0 is a valid reading.
            # For RGB, checking if latest_r > 0 might be better if 0,0,0,0 is "not read yet".
            if latest_r > 0 or latest_g > 0 or latest_b > 0 or latest_c > 0: 
                mqtt_client.publish(TOPIC_R.encode(), str(latest_r))
                mqtt_client.publish(TOPIC_G.encode(), str(latest_g))
                mqtt_client.publish(TOPIC_B.encode(), str(latest_b))

            last_adafruit_publish_time = time.time()
            print("--- Data published to Adafruit ---") # Optional: confirm publish cycle

        time.sleep_ms(50) # Sleep for 50 milliseconds to yield CPU which is apparently important

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

