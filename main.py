import network
import time
import socket
from machine import Pin
from umqttsimple import MQTTClient
import config

# Disable the access point wifi on esp32
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)


# Connect to wifi network of own choice. Important to configure config.py with your "credentials".
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(config.wifi_id, config.wifi_password)


# esp32 connecting to wifi
print("Connecting to WiFi...", end="")
while not wifi.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWiFi connected!" if wifi.isconnected() else "\nFailed to connect!")


# LED pin setup
LED_PIN = 13
led = Pin(LED_PIN, Pin.OUT)
led.value(0)


# Other sensor fx temperature pin setup
Temp_PIN = 27


# MQTT and adafruit configuration. Important to configure config.py with your "credentials".
MQTT_BROKER = "io.adafruit.com"
MQTT_PORT = 1883
AIO_USERNAME = config.username
AIO_KEY = config.key
CLIENT_ID = "esp32_control_device" 


# Different topics to control
LED_TOPIC = b'/feeds/esp32-led-command' # Used in dashboard to control the LED
TEMP_TOPIC = b'/feeds/esp32-temperature-command' # Used in dashboard to read the temperature

mqtt_client = None # Nessessary to declare globally so it can be accessed in the callback ..?


# Function to handle incoming MQTT messages
def subscribe_callback(topic, message):
    global led_state

    if topic == (AIO_USERNAME.encode() + LED_TOPIC):
        if message == b"on":
            led.value(1)
            print("LED set to ON")
        elif message == b"off":
            led.value(0)
            print("LED set to OFF")
        else:
            print("Unknown command:", message.decode())

    elif topic == (AIO_USERNAME.encode() + TEMP_TOPIC):
         if message == b"temp":
            print("working")
    
    else:
        print("Received message on unknown topic:", topic.decode())


# Function to connect to the MQTT broker and subscribe to the LED topic
def connect_and_subscribe():
    global mqtt_client
    print("Connecting to MQTT broker...")
    # Initialize MQTTClient: client_id, server, port, user, password, keepalive=0, ssl=False
    mqtt_client = MQTTClient(CLIENT_ID, MQTT_BROKER, MQTT_PORT, AIO_USERNAME, AIO_KEY)
    mqtt_client.set_callback(subscribe_callback) # Set the callback for received messages
    mqtt_client.connect()

    # Subscribe to the different topics
    LED = AIO_USERNAME.encode() + LED_TOPIC
    TEMP = AIO_USERNAME.encode() + TEMP_TOPIC
    mqtt_client.subscribe(LED)
    mqtt_client.subscribe(TEMP)
    print(f"Connected to MQTT broker: {MQTT_BROKER}")
    print(f"Subscribed to topics: {LED.decode()}, {TEMP.decode()}")


# Ping setup
last_ping_time = time.time()
PING_INTERVAL = 60


# Main loop
try:
    connect_and_subscribe()
    while True:
        try:
            mqtt_client.check_msg()
            # Need to ping the broker every 60 seconds to keep the connection alive
            if (time.time() - last_ping_time) >= PING_INTERVAL:
                mqtt_client.ping()
                last_ping_time = time.time()

            time.sleep(0.1)

        except OSError as e:
            print(f"MQTT OSError: {e}. Reconnecting...")
            time.sleep(2)
            try:
                mqtt_client.disconnect()
            except Exception as e:
                print(f"Error during disconnect: {e}")
            connect_and_subscribe() # Reconnect

except Exception as e:
    print(f"Fatal error: {e}")

finally: # Important to ensure cleanup on exit, fx for the device to not appear online even though it is turned off
    if mqtt_client:
        mqtt_client.disconnect()
        print("MQTT client disconnected.")
    if wifi.isconnected():
        wifi.disconnect()
        print("Wi-Fi disconnected.")