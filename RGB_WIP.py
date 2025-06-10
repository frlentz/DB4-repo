from machine import Pin, SoftI2C, ADC
from time import sleep_ms
import _thread as thread
import tcs34725
import time

#Variables for salinity sensor
raw = 0
analogPin = 34                  # Setting the analog pin number for ADC
dtime = 500                     # Time interval for changing the LED status (in ms)
Vin = 3.3                      # Input voltage for the sensor
Vout = 0
R1 = 10000.0                     # Known resistance value (in the circuit)
R2 = 0
samples = 5000
                  # Number of ADC samples (iterations) for calculating the avg resistance

led15 = Pin(15, Pin.OUT)        # An instance of the Pin class with GPIO pins 26 and 32 set as output pins.
                                # This can be used to control LEDs or other devices
                                # Available Pin: 15,32,26
led14 = Pin(14, Pin.OUT)
adc = ADC(Pin(analogPin))       # An instance of a class, initialized using analogPin to perform ADC.
adc.width(ADC.WIDTH_12BIT)
adc.atten(ADC.ATTN_11DB)

#Class for motor (pump)
class MOTOR:

    def __init__(self):
        self.dir_pin = PIN_OUTS["dir"]
        self.step_pin = PIN_OUTS["step"]
        self.enable_pin = PIN_OUTS["enable"]

        self.set_direction(clockwise=True)
        self.enable()

        self.speed_rate = 5
        self.loop = True

    def enable(self):
        self.enable_pin.off()

    def disable(self):
        self.enable_pin.on()

    def set_direction(self, clockwise = True):
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



PIN_OUTS = {
    # led
    "led": Pin(13, Pin.OUT),

    # MOTOR
    "enable": Pin(12, Pin.OUT),
    "dir": Pin(33, Pin.OUT),
    "step": Pin(27, Pin.OUT),

    # UV SENSOR (BLUE)
    "i2c": SoftI2C(scl=Pin(22), sda=Pin(23)),

}

rgb_sensor = tcs34725.TCS34725(PIN_OUTS["i2c"])

def read_rgb():
    while True:
        r, g, b, c = rgb_sensor.read(raw=True)
        
        # Raw values
        print("Raw RGB:", "R:", r, "G:", g, "B:", b, "Clear:", c)
        
        # Convert to 0–255 scale and format as HTML color
        if c == 0:
            print("Clear value is 0 — can't normalize RGB.")
        else:
            red = int((r / c) * 255)
            green = int((g / c) * 255)
            blue = int((b / c) * 255)
            print("HTML color: #{:02x}{:02x}{:02x}".format(red, green, blue))

        sleep_ms(500)



# Uncomment the following lines if you want to use an OLED display

# I2C = SoftI2C(scl=Pin(22), sda=Pin(23), freq=100000)
# display = ssd1306.SSD1306_I2C(128, 64, I2C)

# rgb_sensor = tcs34725.TCS34725(PIN_OUTS["i2c"])
motor = MOTOR()
#uv_sensor = si1145.SI1145(PIN_OUTS["i2c"])
#s = uv_sensor
# print(type(uv_sensor), type(s))
# import i2c_test

#UV sensor
#def read_uv():
#    while True:
#        uv = s.read_uv
#        visible = s.read_visible
#        ir = s.read_ir
#        prox = s.read_prox
#        print(f"uv: {uv} | visible: {visible} | ir: {ir} | proximity: {prox}")
#        PIN_OUTS["led"].on()
#        sleep_ms(500)
#        PIN_OUTS["led"].off()
#        sleep_ms(100)
        
#motor.start_movement_thread()

#Functions for salinity sensor
def setup():                    # led26, led32 initializing
    led15.init(Pin.OUT)         # Initialize pin 26 as output
    led14.init(Pin.OUT)         # Initialize pin 32 as output


def loop():                     # Main function implementation
    tot = 0.0                   # The variable on which to accumulate resistance calculations
    for i in range(samples):
        led15.value(1)          # digitalWrite(26, HIGH)
        led14.value(0)          # digitalWrite(32, LOW)
        time.sleep_us(dtime)

        led15.value(0)          # digitalWrite(26, LOW)
        led14.value(1)          # digitalWrite(32, HIGH)
        time.sleep_us(dtime)

        raw = adc.read()       # This function reads an analog value from analogPin
                                # Subsequent calculations determine the resistance (R2) based on the voltage (Vout).
        
        if raw:
            buff = raw * Vin
            Vout = buff / 4095.0
            var = (Vin / Vout) - 1
            R2 = R1 * var
            tot += R2           # This calculation is derived from 'Voltage Divider Rule'
            
    avg = tot / samples
    salt = (avg-8663.3)/11053
    print("The average resistance is: {:.2f} Ohm".format(avg))
    if salt < 0:
        print("The average salinity level is: 0%")
    else:
        print("The average salinity level is: {:.2f}".format((avg-8663.3)/11053))
    


#Interface setting
def main():
    while True:
        start_input = int(input("Enter the function (Salinity Sensor=1, Motor=2, Optics=3, Exit=0): "))
        
        if start_input ==1:
            setup()
            while True:
                loop()
                time.sleep(1)
        
        elif start_input == 2:
            motor.start_movement_thread()
            
            while True:
                speed_input = input("Change speed rate (or type 'exit' to go back): ")
                if speed_input.lower() == 'exit':
                    motor.stop()
                    break
                try:
                    motor.speed_rate = int(speed_input)
                except ValueError:
                    print("Please enter a valid integer for speed rate.")
            


            
        elif start_input == 0:
            print("Exiting program.")
            break
        
        else:
            print("Wrong statement, please try again.")

if __name__ == "__main__":
    main()