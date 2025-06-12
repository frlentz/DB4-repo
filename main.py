import network
import time
from machine import Pin, I2C
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
print("\nWiFi connected!" if wifi.isconnected() else "\nFailed to connect!")

# --- Constants ---
OLED_WIDTH, OLED_HEIGHT = 128, 64
SCL_PIN, SDA_PIN = 22, 23
OLED_ADDR, RGB_ADDR = 0x3c, 0x29
LED_PIN = 13
CLIENT_ID = "esp32_rgb_project"
AIO_USER = config.username
AIO_KEY = config.key
MQTT_BROKER = "io.adafruit.com"
MQTT_PORT = 1883

# --- MQTT Topics ---
TOPIC_LED = f"{AIO_USER}/feeds/esp32-led-command"
TOPIC_R = f"{AIO_USER}/feeds/esp32-r"
TOPIC_G = f"{AIO_USER}/feeds/esp32-g"
TOPIC_B = f"{AIO_USER}/feeds/esp32-b"
TOPIC_COLOR = f"{AIO_USER}/feeds/esp32-color"
TOPIC_PUMP = f"{AIO_USER}/feeds/esp32-pump-command"


# --- Pin Setup ---
led = Pin(LED_PIN, Pin.OUT)
led.value(0)

relay_pin = Pin(33, Pin.OUT)
relay_pin.off()  # Start OFF (change to on() if your relay is active LOW)


# --- Pump definitions ---
def start_pump():
    relay_pin.on()   # Use .off() if active LOW
    print("Pump started")

def stop_pump():
    relay_pin.off()  # Use .on() if active LOW
    print("Pump stopped")


# --- I2C + OLED + RGB Sensor Init ---
def init_i2c_devices():
    i2c = I2C(1, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
    oled = None
    rgb = None

    devices = i2c.scan()

    if OLED_ADDR in devices:
        oled = ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=OLED_ADDR)
        oled.fill(0)
        oled.text("Hello,", 0, 0)
        oled.text("DB4 Project", 0, 16)
        oled.text("Starting...", 0, 32)
        oled.show()

    if RGB_ADDR in devices:
        rgb = tcs34725.TCS34725(i2c)
        rgb.integration_time(154)
        rgb.gain(4)

    return oled, rgb

# --- MQTT Callback ---
def mqtt_callback(topic, message):
    if topic.decode() == TOPIC_LED:
        if message == b"on":
            led.value(1)
        elif message == b"off":
            led.value(0)

    elif topic.decode() == TOPIC_PUMP:
        if message == b"start":
            start_pump()
        elif message == b"stop":
            stop_pump()


# --- MQTT Connect ---
def connect_mqtt():
    client = MQTTClient(CLIENT_ID, MQTT_BROKER, port=MQTT_PORT, user=AIO_USER, password=AIO_KEY)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_LED.encode())
    client.subscribe(TOPIC_PUMP.encode())
    return client

# --- Main ---
try:
    oled, rgb = init_i2c_devices()
    mqtt_client = connect_mqtt()
    last_ping = time.time()
    ping_interval = 60

    if oled:
        time.sleep(3)

    while True:
        mqtt_client.check_msg()

        if (time.time() - last_ping) > ping_interval:
            mqtt_client.ping()
            last_ping = time.time()

        if oled:
            oled.fill(0)
            oled.text("Time: " + str(int(time.time())), 0, 0)

        if rgb:
            try:
                r, g, b, c = rgb.read(raw=True)

                if oled:
                    oled.text(f"R:{r} G:{g}", 0, 16)
                    oled.text(f"B:{b} C:{c}", 0, 32)
                    oled.show()

                # Publish RGB values at a safe rate
                mqtt_client.publish(TOPIC_R.encode(), str(r))
                mqtt_client.publish(TOPIC_G.encode(), str(g))
                mqtt_client.publish(TOPIC_B.encode(), str(b))
                hex_color = "#{:02X}{:02X}{:02X}".format(r >> 8, g >> 8, b >> 8)
                mqtt_client.publish(TOPIC_COLOR.encode(), hex_color)

            except Exception:
                if oled:
                    oled.text("Sensor Error", 0, 16)
                    oled.show()

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
        oled.text(str(e)[:16], 0, 16)
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
