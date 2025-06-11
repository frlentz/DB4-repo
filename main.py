import network
import time
import socket
from machine import Pin, I2C
from umqttsimple import MQTTClient
import config
from Learn import ssd1306

#OLED testing

OLED_WIDTH = 128
OLED_HEIGHT = 64

# Define the I2C pins for the OLED on your ESP32
# (SCL=Serial Clock Line, SDA=Serial Data Line)
OLED_SCL_PIN = 22 # Connect display's SCL to ESP32's GPIO 22
OLED_SDA_PIN = 23 # Connect display's SDA to ESP32's GPIO 21

# Define the I2C address of your OLED
# Common addresses are 0x3c (60 decimal) or 0x3d (61 decimal)
OLED_I2C_ADDR = 0x3c


# --- Function to initialize and display a welcome message on OLED ---
def init_oled_display():
    try:
        # Initialize the I2C bus
        # I2C(id, scl, sda, freq) - id=1 is a common choice for ESP32
        i2c = I2C(1, scl=Pin(OLED_SCL_PIN), sda=Pin(OLED_SDA_PIN), freq=400000)

        # Scan for I2C devices to verify the OLED is connected and detected
        print('Scanning I2C bus for devices...')
        devices = i2c.scan()
        if OLED_I2C_ADDR in devices:
            print(f'OLED (0x{OLED_I2C_ADDR:x}) found on I2C bus!')
        else:
            print(f'OLED (0x{OLED_I2C_ADDR:x}) NOT found. Check wiring/address.')
            print(f'Detected I2C devices: {[hex(d) for d in devices]}')
            return None # Return None if OLED not found

        # Initialize the OLED display object
        oled = ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=OLED_I2C_ADDR)

        # Clear the display buffer (fill with black)
        oled.fill(0)

        # Write text to the display buffer
        # oled.text(string, x_pixel, y_pixel, color=1 (white))
        oled.text("Hello,", 0, 0)       # First line (y=0)
        oled.text("DB4 Project", 0, 16) # Second line (y=16, 8 pixels per line)
        oled.text("Starting...", 0, 32) # Third line (y=32)

        # Update the physical display to show the buffer content
        oled.show() 

        print("OLED display initialized and showing welcome message.")
        return oled # Return the oled object for later use
        
    except Exception as e:
        print(f"Error initializing OLED display: {e}")
        return None # Return None on error

# --- How to use this in your main code ---

# Call the function at the beginning of your script, after imports
# You can store the returned oled object if you want to use it later
# e.g., in your MQTT callback or main loop to display status.
my_oled = init_oled_display()

# You can then use my_oled if it's not None
if my_oled:
    # Example: Clear and show time after 3 seconds
    time.sleep(3)
    my_oled.fill(0)
    my_oled.text("Time: " + str(time.time()), 0, 0)
    my_oled.show()
    time.sleep(3)
    my_oled.fill(0)
    my_oled.text("Time: " + str(time.time()), 0, 0)
    my_oled.show()

# The rest of your main.py code would follow here (WiFi, MQTT, etc.)
# If you pass the `my_oled` object around, you can update the display
# in your `sub_cb` or `publish_sensor_data` functions as well.