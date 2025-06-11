# This code is designed to run on a microcontroller with a relay connected to GPIO 26.
# It starts and stops a pump by controlling the relay.
# Adjust the GPIO pin if necessary.

from machine import Pin, I2C
from time import sleep
import ssd1306

# === Relay setup ===
relay_pin = Pin(26, Pin.OUT)

def start_pump():
    relay_pin.off()  # switch from .on() to .off()
    print("Pump started")
    update_display("ON")

def stop_pump():
    relay_pin.on()  # switch from .off() to .on()
    print("Pump stopped")
    update_display("OFF")

# === OLED setup ===
# I2C: SDA = 21, SCL = 22 (change if needed)
i2c = I2C(scl=Pin(22), sda=Pin(21))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

def update_display(status):
    oled.fill(0)  # Clear screen
    oled.text("Pump Status:", 0, 10)
    oled.text("[ {} ]".format(status), 0, 30)
    oled.show()

# === Command Interface ===
def main():
    update_display("OFF")  # Default display
    while True:
        cmd = input("Enter 'start', 'stop', or 'exit': ").strip().lower()
        if cmd == "start":
            start_pump()
        elif cmd == "stop":
            stop_pump()
        elif cmd == "exit":
            stop_pump()
            print("Exiting.")
            break
        else:
            print("Unknown command.")

if __name__ == "__main__":
    main()