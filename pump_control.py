# This code is designed to run on a microcontroller with a relay connected to GPIO 26.
# It starts and stops a pump by controlling the relay.
# Adjust the GPIO pin if necessary.

from machine import Pin
from time import sleep

# Relay control pin
relay_pin = Pin(33, Pin.OUT)

def start_pump():
    relay_pin.on()   # Use .off() if your relay is active LOW
    print("Pump started")

def stop_pump():
    relay_pin.off()   # Use .on() to turn OFF if relay is active LOW
    print("Pump stopped")

def main():
    while True:
        command = input("Type 'start', 'stop', or 'exit': ").strip().lower()
        if command == 'start':
            start_pump()
        elif command == 'stop':
            stop_pump()
        elif command == 'exit':
            stop_pump()
            print("Exiting program.")
            break
        else:
            print("Invalid command.")

if __name__ == "__main__":
    main()
