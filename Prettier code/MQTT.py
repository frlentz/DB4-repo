from umqttsimple import MQTTClient
from Devices import set_pump_speed, set_sub_pump_speed

# --- MQTT Topics ---
TOPIC_LED = f"{AIO_user}/feeds/esp32-led-command"
TOPIC_R = f"{AIO_user}/feeds/esp32-r"
TOPIC_G = f"{AIO_user}/feeds/esp32-g"
TOPIC_B = f"{AIO_user}/feeds/esp32-b"
TOPIC_pump_speed = f"{AIO_user}/feeds/esp32-pump-speed"
TOPIC_sub_pump_speed = f"{AIO_user}/feeds/esp32-sub-pump-speed"  
TOPIC_temp = f"{AIO_user}/feeds/esp32-temp"



class MQTTfuncs:
    def mqtt_led_callback(led, topic, message):
        topic_str = topic.decode()
        msg_str = message.decode().lower()

        if topic_str == TOPIC_LED:
            led.value(1 if msg_str == "on" else 0)


    def mqtt_pump_callback(topic, message):
        topic_str = topic.decode()
        msg_str = message.decode().lower()

        if topic_str == TOPIC_pump_speed:
            try:
                percent = int(msg_str)
                set_pump_speed(percent)

            except ValueError as e: 
                print("Invalid speed value for main pump")
                sys.print_exception(e) #


    def mqtt_led_callback(topic, message):
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

    def connect_mqtt(MQTT_broker, MQTT_port, AIO_user, AIO_key, Client_ID):
    client = MQTTClient(Client_ID, MQTT_broker, port=MQTT_port, user=AIO_user, password=AIO_key)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_LED.encode())
    client.subscribe(TOPIC_pump_speed.encode())
    client.subscribe(TOPIC_sub_pump_speed.encode())
    return client