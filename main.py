import network
import time
from machine import Pin, I2C
# Removed socket as it's not used here
# from umqttsimple import MQTTClient # Removed as it's not used here
import config # For Wi-Fi credentials (if you want to connect to Wi-Fi)
from Learn import ssd1306
from Learn import tcs34725 # This is your provided driver file


OLED_WIDTH = 128
OLED_HEIGHT = 64


# Define the I2C pins for the OLED (SCL=Serial Clock Line, SDA=Serial Data Line)
OLED_SCL_PIN = 22
OLED_SDA_PIN = 23


# Common addresses are 0x3c (60 decimal) for OLED, 0x29 for RGB sensor
OLED_I2C_ADDR = 0x3c
RGB_I2C_ADDR = 0x29


# --- Function to initialize I2C devices (OLED and RGB Sensor) ---
def init_i2c_devices():
    # Initialize variables to None. They will be assigned actual objects if found.
    i2c = None
    oled = None
    rgb = None

    try:
        # Initialize the I2C bus once for both devices
        i2c = I2C(1, scl=Pin(OLED_SCL_PIN), sda=Pin(OLED_SDA_PIN), freq=400000)

        # Scan for I2C devices
        print('Scanning I2C bus for devices...')
        devices = i2c.scan()
        print(f'Detected I2C devices: {[hex(d) for d in devices]}')


        # --- Attempt to initialize OLED ---
        if OLED_I2C_ADDR in devices:
            print(f'OLED (0x{OLED_I2C_ADDR:x}) found on I2C bus!')
            oled = ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=OLED_I2C_ADDR)
            
            # Initial display on OLED if found
            oled.fill(0) # Clear display buffer
            oled.text("Hello,", 0, 0)
            oled.text("DB4 Project", 0, 16)
            oled.text("Starting...", 0, 32)
            oled.show()
        else:
            print(f'OLED (0x{OLED_I2C_ADDR:x}) NOT found. OLED display will not be used.')


        # --- Attempt to initialize RGB Sensor ---
        if RGB_I2C_ADDR in devices:
            print(f'RGB (0x{RGB_I2C_ADDR:x}) found on I2C bus!')
            rgb = tcs34725.TCS34725(i2c) # Initialize RGB sensor, passing the I2C bus object
            
            # --- FIX FOR YOUR SPECIFIC DRIVER ---
            # Pass the time in milliseconds and the gain multiplier directly
            rgb.integration_time(154) # Sets integration time to 154 milliseconds
            rgb.gain(4)               # Sets gain to 4x
            print("RGB sensor initialized with 154ms integration and 4x gain.")
        else:
            print(f'RGB (0x{RGB_I2C_ADDR:x}) NOT found. RGB sensor will not be used.')

        # Return both initialized objects. One or both could be None if not found.
        return oled, rgb
        
    except Exception as e:
        print(f"Error during I2C device initialization: {e}")
        # If any error occurs during I2C setup, return None for both
        return None, None


# --- Main execution starts here ---

try:
    # Call the initialization function and capture both returned objects
    oled_instance, rgb_instance = init_i2c_devices()

    # Now, use the captured instances. Check if they are not None before using.
    if oled_instance:
        time.sleep(5) # Let the "Starting..." message show for 5 seconds
        
        for i in range(100): # Loop 100 times
            time.sleep(0.2) # Wait 0.2 seconds between updates

            oled_instance.fill(0) # Clear the entire display for fresh content

            # Always display time on the OLED
            oled_instance.text("Time: " + str(time.time()), 0, 0)

            # --- Read and Print RGB values to Terminal (and optionally display on OLED) ---
            if rgb_instance: # Check if the RGB sensor was successfully initialized
                try:
                    # Read raw Red, Green, Blue, and Clear values
                    r, g, b, c = rgb_instance.read(raw=True)
                    
                    # Print to terminal
                    print(f"Loop {i+1}: R={r}, G={g}, B={b}, C={c}")

                    # Optionally, display on OLED as well
                    oled_instance.text(f"R:{r} G:{g}", 0, 16) # Display on line 2 (y=16)
                    oled_instance.text(f"B:{b} C:{c}", 0, 32) # Display on line 3 (y=32)

                except Exception as e:
                    # Handle errors during RGB reading
                    print(f"Error reading RGB sensor: {e}")
                    oled_instance.text("RGB Error!", 0, 16)
                    oled_instance.text(str(e)[:16], 0, 32) # Display part of the error message
            else:
                # If RGB sensor was not initialized, print/display a message
                print("RGB sensor not initialized.")
                oled_instance.text("No RGB Sensor", 0, 16)


            oled_instance.show() # Update the physical display with all new content

        # After the loop finishes, display a "Loop Finished" message
        oled_instance.fill(0)
        oled_instance.text("Loop Finished", 0, 0)
        oled_instance.show()
    else:
        print("OLED display not initialized. Cannot display any output on screen.")

except Exception as e:
    print(f"A general error occurred: {e}")
    if oled_instance:
        oled_instance.fill(0)
        oled_instance.text("FATAL ERROR!", 0, 0)
        oled_instance.text(str(e)[:16], 0, 16)
        oled_instance.show()
    time.sleep(5)
finally:
    print("Script finished or encountered an unhandled error.")
    if oled_instance:
        oled_instance.fill(0)
        oled_instance.text("Goodbye!", 0, 0)
        oled_instance.show()
        time.sleep(1)