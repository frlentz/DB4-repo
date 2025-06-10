from machine import Pin
from time import sleep_ms
import _thread as thread

# Define motor control pins
PIN_OUTS = {
    "enable": Pin(12, Pin.OUT),
    "dir": Pin(33, Pin.OUT),
    "step": Pin(27, Pin.OUT),
}

# Stepper motor control class
class MOTOR:
    def __init__(self):
        self.dir_pin = PIN_OUTS["dir"]
        self.step_pin = PIN_OUTS["step"]
        self.enable_pin = PIN_OUTS["enable"]

        self.set_direction(clockwise=True)
        self.enable()

        self.speed_rate = 5  # ms between steps
        self.loop = True     # controls thread loop

    def enable(self):
        self.enable_pin.off()  # Enable is active LOW

    def disable(self):
        self.enable_pin.on()

    def set_direction(self, clockwise=True):
        if clockwise:
            self.dir_pin.off()
        else:
            self.dir_pin.on()

    def start_movement_thread(self):
        thread.start_new_thread(self.set_speed, [])

    def set_speed(self):
        while self.loop:
            self.step_pin.on()
            sleep_ms(self.speed_rate)
            self.step_pin.off()

    def stop(self):
        self.loop = False
        self.disable()

# Create motor instance
motor = MOTOR()

# User interface
def main():
    print("Commands:")
    print(" run       -> start pump")
    print(" speed <n> -> set delay in ms per step")
    print(" dir       -> change direction")
    print(" high      -> set STEP pin HIGH")
    print(" low       -> set STEP pin LOW")
    print(" exit      -> stop and exit")

    while True:
        cmd = input(">>> ").strip().lower()

        if cmd == "run":
            motor.loop = True
            motor.enable()
            motor.start_movement_thread()

        elif cmd.startswith("speed"):
            try:
                _, value = cmd.split()
                motor.speed_rate = max(1, int(value))
                print(f"Speed set to {motor.speed_rate} ms/step")
            except:
                print("Usage: speed <number>")

        elif cmd == "dir":
            # Flip direction
            current = motor.dir_pin.value()
            motor.set_direction(clockwise=not current)
            print("Direction toggled")

        elif cmd == "high":
            PIN_OUTS["step"].on()
            print("STEP pin set to HIGH")

        elif cmd == "low":
            PIN_OUTS["step"].off()
            print("STEP pin set to LOW")

        elif cmd == "exit":
            motor.stop()
            print("Pump stopped. Goodbye!")
            break

        else:
            print("Unknown command.")

if __name__ == "__main__":
    main()
