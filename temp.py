from machine import ADC, Pin
import math
import time

# Thermistor setup (connected to pin 34)
adc = ADC(Pin(34))
adc.atten(ADC.ATTN_11DB)  # Full range: 0–3.3V

# Constants
BETA = 3950          # B value of the thermistor
R0 = 10000           # 10k Ohm at 25°C
T0 = 298.15          # 25°C in Kelvin
R_FIXED = 10000      # 10k Ohm fixed resistor

def read_temperature():
    adc_val = adc.read()
    voltage = adc_val / 4095 * 3.3

    # Calculate thermistor resistance
    resistance = R_FIXED * (3.3 / voltage - 1)

    # Steinhart-Hart (Beta version)
    temp_K = 1 / (1 / T0 + (1 / BETA) * math.log(resistance / R0))
    temp_C = temp_K - 273.15
    return round(temp_C, 2)

# Loop to print every 2 seconds
while True:
    temp = read_temperature()
    print("Temperature: {} °C".format(temp))
    time.sleep(2)
