from umqttsimple import MQTTClient
from Devices import motors

class MQTTfuncs:
    @staticmethod
    def mqtt_led_callback(topic, message):
        topic_str = topic.decode()
        msg_str = message.decode().lower()

        if topic_str == TOPIC_LED:
            led.value(1 if msg_str == "on" else 0)

        
    @staticmethod
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



    @staticmethod
    def connect_mqtt(MQTT_broker, MQTT_port, AIO_user, AIO_key, Client_ID):
        client = MQTTClient(Client_ID, MQTT_broker, port=MQTT_port, user=AIO_user, password=AIO_key)
        client.set_callback(mqtt_callback)
        client.connect()
        client.subscribe(TOPIC_LED.encode())
        client.subscribe(TOPIC_pump_speed.encode())
        client.subscribe(TOPIC_sub_pump_speed.encode())
        return client