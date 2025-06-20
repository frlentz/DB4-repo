from machine import Pin, I2C, PWM, ADC
from Learn import tcs34725
from Learn import ssd1306

class motors:
    @staticmethod
    def set_pump_speed(percent, pump_pwm):
        percent = max(0, min(percent, 100))
        # Remap to usable PWM range: 0–100% → 0.7–1.0
        if percent == 0:
            pwm_value = 0
        else:
            scaled = 0.7 + (percent / 100) * 0.3
            pwm_value = int(scaled * 1023)
        pump_pwm.duty(pwm_value)
        print(f"Pump speed set to {percent}% → PWM: {pwm_value}/1023")

    @staticmethod
    def set_sub_pump_speed(percent, sub_pump_pwm):
        percent = max(0, min(percent, 100))
        # Remap to usable PWM range: 0–100% → 0.3–1
        if percent == 0:
            pwm_value = 0
        else:
            scaled = 0.4 + (percent / 100) * 0.6
            pwm_value = int(scaled * 1023)
        sub_pump_pwm.duty(pwm_value)
        print(f"Submersible pump speed set to {percent}% → PWM: {pwm_value}/1023")



class I2Cs:
    @staticmethod
    def init_i2c_devices(SCL_PIN, SDA_PIN, OLED_addr, RGB_addr, OLED_width=128, OLED_height=64):
        i2c = I2C(1, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000) # Initializing the I2C bus once for all devices
        oled = None
        rgb = None

        # Scanning for I2C devices
        print('Scanning I2C bus for devices...')
        devices = i2c.scan()
        print(f'Detected I2C devices: {[hex(d) for d in devices]}')

        if OLED_addr in devices: # Initial display on OLED
            oled = ssd1306.SSD1306_I2C(OLED_width, OLED_height, i2c, addr=OLED_addr)
            oled.fill(0)
            oled.text("Hello,", 0, 0) # The second parameter is the x position, third is y
            oled.text("DB4 Project", 0, 16)
            oled.text("Starting...", 0, 32)
            oled.show()

        if RGB_addr in devices:
            rgb = tcs34725.TCS34725(i2c) # Initializing RGB sensor
            rgb.integration_time(400)  # This tells the sensor how long to collect light before converting it into a digital reading. In this case 154 milliseconds. Must be one of the allowed gain values: 1, 4, 16, or 60. Higher gain amplifies the signal more. Useful in dim light to get stronger readings
            rgb.gain(4)                # Higher gain amplifies the signal more.
        return oled, rgb