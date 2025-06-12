import network
import time
from machine import Pin, I2C, PWM
from umqttsimple import MQTTClient
import config
from Learn import ssd1306
from Learn import tcs34725

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
CLIENT_ID = "esp32_rgb_project"
AIO_user = config.username # Needs to be configured in config.py
AIO_key = config.key # Needs to be configured in config.py
MQTT_broker = "io.adafruit.com"
MQTT_port = 1883


# --- MQTT Topics ---
TOPIC_LED = f"{AIO_user}/feeds/esp32-led-command"
TOPIC_R = f"{AIO_user}/feeds/esp32-r"
TOPIC_G = f"{AIO_user}/feeds/esp32-g"
TOPIC_B = f"{AIO_user}/feeds/esp32-b"
MQTT_pump = f"{AIO_user}/feeds/esp32-pump-command"
TOPIC_PUMP_SPEED = f"{AIO_user}/feeds/esp32-pump-speed"


# --- Pin setup ---
led = Pin(LED_PIN, Pin.OUT)
led.value(0)

pump_pwm = PWM(Pin(33), freq=1000)  # 1 kHz PWM on pin 33
pump_pwm.duty(0)  # Start with pump off



# --- Pump definitions ---
def set_pump_speed(percent):
    # Clamp to 0–100
    percent = max(0, min(percent, 100))
    pwm_value = int(percent * 1023 / 100)
    pump_pwm.duty(pwm_value)
    print(f"Pump speed set to {percent}% ({pwm_value}/1023)")



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
        rgb.integration_time(154)  # This tells the sensor how long to collect light before converting it into a digital reading. In this case 154 milliseconds. Must be one of the allowed gain values: 1, 4, 16, or 60. Higher gain amplifies the signal more. Useful in dim light to get stronger readings
        rgb.gain(4)                # Higher gain amplifies the signal more.
    return oled, rgb


# --- MQTT Callback ---
def mqtt_callback(topic, message):
    topic_str = topic.decode()
    msg_str = message.decode().lower()

    if topic_str == TOPIC_LED:
        led.value(1 if msg_str == "on" else 0)

    elif topic_str == MQTT_pump:
        if msg_str == "on":
            set_pump_speed(100)  # Full speed
        elif msg_str == "off":
            set_pump_speed(0)    # Stop

    elif topic_str == TOPIC_PUMP_SPEED:
        try:
            percent = int(msg_str)
            set_pump_speed(percent)
        except:
            print("Invalid speed value")


# --- MQTT Connect ---
def connect_mqtt():
    client = MQTTClient(CLIENT_ID, MQTT_broker, port=MQTT_port, user=AIO_user, password=AIO_key)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_LED.encode())
    client.subscribe(MQTT_pump.encode())
    client.subscribe(TOPIC_PUMP_SPEED.encode())
    return client


# --- Main ---
try:
    oled, rgb = init_i2c_devices()
    mqtt_client = connect_mqtt()
    last_ping = time.time()
    ping_interval = 60

    if oled:
        time.sleep(3) # Lets the OLED "Starting..." message show for 3 seconds

    while True:
        mqtt_client.check_msg()

        if (time.time() - last_ping) > ping_interval:
            mqtt_client.ping()
            last_ping = time.time()

        if oled:
            oled.fill(0) # Clears display (black background)
            oled.text("Time: " + str(int(time.time())), 0, 0)

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

            except Exception:
                if oled:
                    oled.text("Sensor Error", 0, 16)
                    oled.show()

        # This part is to send keep-alive pings to the MQTT broker so the connection stays alive
        sleep_duration = 15 
        sleep_start = time.time()
        while time.time() - sleep_start < sleep_duration:
            mqtt_client.check_msg()
            time.sleep(0.1)


except Exception as e:
    print("Fatal Error:", e)
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