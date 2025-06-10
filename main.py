import network
import time
import socket
from machine import Pin
from umqttsimple import MQTTClient
import config

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

LED_PIN = 13
led = Pin(LED_PIN, Pin.OUT)
led_state = 0
led.value(led_state)

MQTT_BROKER = "io.adafruit.com"
MQTT_PORT = 1883
AIO_USERNAME = config.username # <--- ENTER YOUR ADAFRUIT IO USERNAME HERE
AIO_KEY = config.key         # <--- ENTER YOUR ADAFRUIT IO KEY HERE

CLIENT_ID = "esp32_led_control_device"

COMMAND_TOPIC = b'/feeds/esp32-led-command'

STATUS_TOPIC = b'/feeds/esp32-led-status'

mqtt_client = None


def sub_cb(topic, msg):
    global led_state
    print(f"[{time.time()}] Received Topic: {topic.decode()}, Message: {msg.decode()}")

    if topic == (AIO_USERNAME.encode() + COMMAND_TOPIC):
        if msg == b"on":
            led.value(1)
            led_state = 1
            print("LED set to ON")
        elif msg == b"off":
            led.value(0)
            led_state = 0
            print("LED set to OFF")
        else:
            print("Unknown command:", msg.decode())
        
        publish_led_status()
    else:
        print("Received message on unknown topic:", topic.decode())


def connect_and_subscribe():
    global mqtt_client
    print("Connecting to MQTT broker...")
    # Initialize MQTTClient: client_id, server, port, user, password, keepalive=0, ssl=False
    mqtt_client = MQTTClient(CLIENT_ID, MQTT_BROKER, MQTT_PORT, AIO_USERNAME, AIO_KEY)
    mqtt_client.set_callback(sub_cb) # Set the callback for received messages
    mqtt_client.connect()
    
    
    full_command_topic = AIO_USERNAME.encode() + COMMAND_TOPIC
    mqtt_client.subscribe(full_command_topic)
    print(f"Connected to MQTT broker: {MQTT_BROKER}")
    print(f"Subscribed to command topic: {full_command_topic.decode()}")
    

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

last_ping_time = time.time()
PING_INTERVAL = 60

try:
    connect_and_subscribe()

    while True:
        try:
            mqtt_client.check_msg()
            
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
finally:
    if mqtt_client:
        mqtt_client.disconnect()
        print("MQTT client disconnected.")
    if wifi.isconnected():
        wifi.disconnect()
        print("Wi-Fi disconnected.")