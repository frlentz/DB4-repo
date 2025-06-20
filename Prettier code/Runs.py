import time
import config
from machine import Pin, I2C, PWM, ADC
import sys
from umqttsimple import MQTTClient
from Learn import ssd1306
from Learn import tcs34725
from Learn import read_temp
from Devices import motors, I2Cs
from PID_controller import PID
from MQTT import MQTTfuncs

def offline_run(SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height, Temp_PIN, sub_pump_pwm):
    # offline version: MQTT-dependent logic is replaced with local equivalents/logging 
    oled = None
    rgb = None
    mqtt_client = None
    temp_sens = None
    last_temp_publish_time = 0 
    temp_publish_interval = 30
    data_for_esp=("")

    # Defining optimal constants for PID

    ##### regulate temperature for mussels ####
    Kp_mus = 2.0
    Ki_mus = 0.1
    Kd_mus = 1.0
    target_temp = 20.5

    ##### regulate algae density ####
    # Kp_alg = 2.0
    # Ki_alg = 0.1
    # Kd_alg = 1.0
    # target_abs = 200 #change this


    try:
        oled, rgb = I2Cs.init_i2c_devices(SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height)
        last_ping = time.time()
        ping_interval = 60
        temp_sens = read_temp.init_temp_sensor(Temp_PIN)
        pid_temp = PID(Kp_mus, Ki_mus, Kd_mus, setpoint=target_temp, output_limits=(0, 100))  # 0-100% pump speed

        if oled:
            time.sleep(3) # Lets the OLED "Starting..." message show for 3 seconds

        while True:

            if oled:
                oled.fill(0) # Clears display (black background)
                oled.text("Time: " + str(int(time.time())), 0, 0)

            if temp_sens and (time.time() - last_temp_publish_time) > temp_publish_interval:
                try:
                    current_temp = read_temp.read_temp(temp_sens)

                    # Compute PID output
                    sub_pump_speed = pid_temp.compute(current_temp)

                    # Apply the computed speed
                    motors.set_sub_pump_speed(sub_pump_speed, sub_pump_pwm)

                    data=str(current_temp) 
                    data_for_esp=str(str(time.time()) + ": " + data + ", ")

                    #Logging the data
                    with open("data.txt", "a") as f: 
                        f.write(data_for_esp)
                    # f = open('data.txt', 'a')
                    # f.write(data_for_esp)
                    # f.close() 
                    # To see it: >>> f = open("data.txt")  >>> f.read(

                    
                    last_temp_publish_time = time.time() # Update the timer after successful read/publish

                except Exception as e:
                    print(f"Temperature Sensor Error:")
                    sys.print_exception(e) 


            if rgb: 
                try:
                    r, g, b, c = rgb.read(raw=True) # These are the direct 16-bit numbers that the sensor's Analog-to-Digital Converter (ADC) produces for each color channel (Red, Green, Blue) and the Clear (unfiltered) channel.
                                                    # they range from 0 (no light detected) up to 65535 (maximum light detected).
                    
                    # === Log RGB data for algae analysis ===
                    timestamp = int(time.time())
                    rgb_log_entry = f"{timestamp},{r},{g},{b},{c}\n"
                    with open("rgb_log.csv", "a") as rgb_file:
                        rgb_file.write(rgb_log_entry)

                    if oled:
                        oled.text(f"R:{r} G:{g}", 0, 16) # Red, Green, Blue, and Clear values
                        oled.text(f"B:{b} C:{c}", 0, 32)
                        oled.show()


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
        if oled:
            oled.fill(0)
            oled.text("Goodbye", 0, 0)
            oled.show()


def online_run(wifi, SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height, Temp_PIN, sub_pump_pwm, TOPIC_sub_pump_speed, TOPIC_temp, TOPIC_R, TOPIC_G, TOPIC_B, MQTT_broker, MQTT_port, AIO_user, AIO_key, Client_ID):
    oled = None
    rgb = None
    mqtt_client = None
    temp_sens = None
    last_temp_publish_time = 0 
    temp_publish_interval = 30 # We only want a temp readiing every 30 seconds
    data_for_esp=("")

    #Example constants for the PID controller
    #these need to be from the web server

    ##### regulate temperature for mussels ####
    Kp_mus = 2.0
    Ki_mus = 0.1
    Kd_mus = 1.0
    target_temp = 20.5

    ##### regulate algae density ####
    Kp_alg = 2.0
    Ki_alg = 0.1
    Kd_alg = 1.0
    target_abs = 200 #change this


    try:
        oled, rgb = motors.init_i2c_devices(SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width, OLED_height)
        mqtt_client = MQTTfuncs.connect_mqtt(MQTT_broker, MQTT_port, AIO_user, AIO_key, Client_ID)
        last_ping = time.time()
        ping_interval = 60
        temp_sens = read_temp.init_temp_sensor(Temp_PIN)
        pid_temp = PID(Kp_mus, Ki_mus, Kd_mus, setpoint=target_temp, output_limits=(0, 100))  # 0-100% pump speed

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
                    motors.set_sub_pump_speed(sub_pump_speed, sub_pump_pwm)

                    # publish PID output
                    mqtt_client.publish(TOPIC_sub_pump_speed.encode(), str(int(sub_pump_speed)))
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
                    
                    # === Log RGB data for algae analysis ===
                    timestamp = int(time.time())
                    rgb_log_entry = f"{timestamp},{r},{g},{b},{c}\n"
                    with open("rgb_log.csv", "a") as rgb_file:
                        rgb_file.write(rgb_log_entry)

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

