import network
import time
import socket
from machine import Pin # Import for GPIO control
from umqttsimple import MQTTClient # MQTT client library
import config

# --- Wi-Fi Connection (from your barebones code) ---
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(config.wifi_id, config.wifi_password)

# Wait for Wi-Fi connection
print("Connecting to WiFi...", end="")
while not wifi.isconnected():
    print(".", end="")
    time.sleep(1)
print("\nWiFi connected!" if wifi.isconnected() else "\nFailed to connect!")


# --- LED Setup (Built-in LED on ESP32 Dev Boards is usually on GPIO 2, but often also on 13 for breakouts) ---
# NOTE: Pin 13 is often connected to the built-in LED on ESP32 development boards.
# If your LED doesn't light up, try GPIO 2 instead: LED_PIN = 2
LED_PIN = 13
led = Pin(LED_PIN, Pin.OUT)
led_state = 0 # 0 for off, 1 for on. Let's start with it off.
led.value(led_state) # Set initial LED state

# --- MQTT Setup ---
# **IMPORTANT: REPLACE THESE WITH YOUR ACTUAL ADAFRUIT IO CREDENTIALS**
MQTT_BROKER = "io.adafruit.com"
MQTT_PORT = 1883 # Standard MQTT port (unencrypted)
AIO_USERNAME = config.username # <--- ENTER YOUR ADAFRUIT IO USERNAME HERE
AIO_KEY = config.key         # <--- ENTER YOUR ADAFRUIT IO KEY HERE

CLIENT_ID = "esp32_led_control_device" # Can be any unique string for your device
# Define your topics. Adafruit IO uses the format 'username/feeds/feedname'.
# For command:
COMMAND_TOPIC = b'/feeds/esp32-led-command' # Topic to subscribe to for ON/OFF commands
# For status:
STATUS_TOPIC = b'/feeds/esp32-led-status' # Topic to publish LED status to

# MQTT client instance
mqtt_client = None

# --- MQTT Message Callback Function ---
# This function is called automatically when a message is received on a subscribed topic.
def sub_cb(topic, msg):
    global led_state
    print(f"[{time.time()}] Received Topic: {topic.decode()}, Message: {msg.decode()}")

    if topic == (AIO_USERNAME.encode() + COMMAND_TOPIC): # Make sure to compare with the full topic path
        if msg == b"on":
            led.value(1) # Turn LED ON
            led_state = 1
            print("LED set to ON")
        elif msg == b"off":
            led.value(0) # Turn LED OFF
            led_state = 0
            print("LED set to OFF")
        else:
            print("Unknown command:", msg.decode())
        
        # After changing the LED state, publish its new status
        publish_led_status()
    else:
        print("Received message on unknown topic:", topic.decode())

# --- MQTT Connection & Publishing Functions ---
def connect_and_subscribe():
    global mqtt_client
    print("Connecting to MQTT broker...")
    # Initialize MQTTClient: client_id, server, port, user, password, keepalive=0, ssl=False
    mqtt_client = MQTTClient(CLIENT_ID, MQTT_BROKER, MQTT_PORT, AIO_USERNAME, AIO_KEY)
    mqtt_client.set_callback(sub_cb) # Set the callback for received messages
    mqtt_client.connect()
    
    # Subscribe to the command topic
    full_command_topic = AIO_USERNAME.encode() + COMMAND_TOPIC
    mqtt_client.subscribe(full_command_topic)
    print(f"Connected to MQTT broker: {MQTT_BROKER}")
    print(f"Subscribed to command topic: {full_command_topic.decode()}")
    
    # Publish initial LED state right after connecting
    publish_led_status()
    print("Initial LED status published.")

def publish_led_status():
    global mqtt_client, led_state
    try:
        status_message = b"on" if led_state == 1 else b"off"
        full_status_topic = AIO_USERNAME.encode() + STATUS_TOPIC
        mqtt_client.publish(full_status_topic, status_message)
        print(f"[{time.time()}] Published status: {status_message.decode()} to {full_status_topic.decode()}")
    except Exception as e:
        print(f"Error publishing: {e}")
        # In barebones, we don't handle reconnect here, but in a real app, you would.

# --- Main Program Loop ---
# This replaces your old HTTP request section
last_ping_time = time.time()
PING_INTERVAL = 60 # Send a PING every 60 seconds to keep connection alive

try:
    connect_and_subscribe() # Connect to Wi-Fi happens before this in barebones

    while True:
        try:
            # Check for new MQTT messages and process them via sub_cb
            mqtt_client.check_msg()
            
            # Send a ping to keep the connection alive
            if (time.time() - last_ping_time) >= PING_INTERVAL:
                mqtt_client.ping()
                last_ping_time = time.time()
                # print("MQTT Ping sent.") # Uncomment for more verbose ping logs

            time.sleep(0.1) # Small delay to prevent busy-waiting

        except OSError as e:
            print(f"MQTT OSError: {e}. Reconnecting...")
            time.sleep(2) # Wait a bit before attempting reconnect
            # In a barebones code, we'll just try to reconnect directly
            # For a robust solution, you'd add exponential backoff or more sophisticated handling
            try:
                mqtt_client.disconnect()
            except Exception as e:
                print(f"Error during disconnect: {e}")
            connect_and_subscribe() # Reconnect

except Exception as e:
    print(f"Fatal error: {e}")
finally:
    if mqtt_client:
        mqtt_client.disconnect()
        print("MQTT client disconnected.")
    if wifi.isconnected():
        wifi.disconnect()
        print("Wi-Fi disconnected.")