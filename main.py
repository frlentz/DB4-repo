import network
import time
from machine import Pin, I2C
import config 
from Learn import ssd1306
from Learn import tcs34725

# OLED dimensions (probably)
OLED_width = 128
OLED_height = 64


# I2C pins for the OLED (SCL=Serial Clock Line, SDA=Serial Data Line)
SCL_pin = 22
SDA_pin = 23


# hexidecimal addresses for the I2C devices (when scanning using i2c.scan() it gives us 61 and 40)
OLED_I2C_addr = 0x3c
RGB_I2C_addr = 0x29


# Initializing I2C devices
def init_i2c_devices():
    i2c = None
    oled = None
    rgb = None

    try:
        # Initializing the I2C bus once for all devices
        i2c = I2C(1, scl=Pin(SCL_pin), sda=Pin(SDA_pin), freq=400000)

        # Scanning for I2C devices
        print('Scanning I2C bus for devices...')
        devices = i2c.scan()
        print(f'Detected I2C devices: {[hex(d) for d in devices]}')


        # initializing OLED
        if OLED_I2C_addr in devices:
            print(f'OLED ({OLED_I2C_addr:x}) found.')
            oled = ssd1306.SSD1306_I2C(OLED_width, OLED_height, i2c, addr=OLED_I2C_addr)
            
            # Initial display on OLED
            oled.fill(0) # Clears display (black background)
            oled.text("Hello,", 0, 0) # The second parameter is the x position, third is y
            oled.text("DB4 Project", 0, 16)
            oled.text("Starting...", 0, 32)
            oled.show()
        else:
            print(f'OLED ({OLED_I2C_addr:x}) not found.')


        # Initializing RGB Sensor
        if RGB_I2C_addr in devices:
            print(f'RGB (0x{RGB_I2C_addr:x}) found.')
            rgb = tcs34725.TCS34725(i2c) # Initializing RGB sensor
            
            # Pass the time in milliseconds and the gain multiplier directly
            rgb.integration_time(154) # This tells the sensor how long to collect light before converting it into a digital reading. In this case 154 milliseconds.
            rgb.gain(4)  # Must be one of the allowed gain values: 1, 4, 16, or 60. Higher gain amplifies the signal more. Useful in dim light to get stronger readings
            print("RGB sensor initialized.")
        else:
            print(f'RGB (0x{RGB_I2C_addr:x}) not found.')

        # Return initialized objects. One or both could be None if not found.
        return oled, rgb
        
    except Exception as e:
        print(f"Error during I2C device initialization: {e}") # If any error occurs during I2C setup, return None for both
        return None, None


# Main execution
try:
    oled_instance, rgb_instance = init_i2c_devices() # Call the initialization function and capture both returned objects
    
    if oled_instance: # Checks if they are not None before using.
        time.sleep(5) # Lets the OLED "Starting..." message show for 5 seconds
        
        for i in range(100): # Loop 100 times
            time.sleep(0.2) # Wait 0.2 seconds between updates

            oled_instance.fill(0) # Clear the entire display for fresh content
            oled_instance.text("Time: " + str(time.time()), 0, 0)  # Always display time on the OLED

            if rgb_instance: # Checks if the RGB sensor was successfully initialized
                try:
                    r, g, b, c = rgb_instance.read(raw=True) # Read raw Red, Green, Blue, and Clear values
                    
                    print(f"Loop {i+1}: R={r}, G={g}, B={b}, C={c}") # These are the direct 16-bit numbers that the sensor's Analog-to-Digital Converter (ADC) produces for each color channel (Red, Green, Blue) and the Clear (unfiltered) channel.
                                                                     # they range from 0 (no light detected) up to 65535 (maximum light detected).
                    oled_instance.text(f"R:{r} G:{g}", 0, 16)
                    oled_instance.text(f"B:{b} C:{c}", 0, 32)

                except Exception as e:
                    print(f"Error reading RGB sensor: {e}")
                    oled_instance.text("RGB Error", 0, 16)
            else:
                print("RGB sensor not initialized.") # If RGB sensor was not initialized
                oled_instance.text("No RGB Sensor", 0, 16)

            oled_instance.show() # Update the physical display with all new content


        oled_instance.fill(0) # After loop finishes, it displays "Loop Finished"
        oled_instance.text("Loop Finished", 0, 0)
        oled_instance.show()
    else:
        print("OLED display not initialized.")

except Exception as e:
    print(f"An error occurred: {e}")
    if oled_instance:
        oled_instance.fill(0)
        oled_instance.text("ERROR", 0, 0)
        oled_instance.show()
    time.sleep(5)

finally:
    print("Ffinished.")
    if oled_instance:
        oled_instance.fill(0)
        oled_instance.text("Goodbye", 0, 0)
        oled_instance.show()
        time.sleep(1)