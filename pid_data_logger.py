from machine import Pin, PWM
import time
from Learn import read_temp

# Hardware Pins
TEMP_PIN = 36
SUB_PUMP_PIN = 32

# Cooling Setup
sub_pwm = PWM(Pin(SUB_PUMP_PIN), freq=1000)
sub_pwm.duty(1023)  # Full cooling for step response

# Temperature Sensor
temp_sensor = read_temp.init_temp_sensor(TEMP_PIN)

# File Setup 
filename = "step_response_log.csv"
with open(filename, "w") as f:
    f.write("Time_s,Temperature_C\n")

print("Starting step response logging...")

# Logging Loop 
start_time = time.time()
log_interval = 5  # seconds

total_duration = 20 * 60  # Run for 20 minutes

try:
    while time.time() - start_time < total_duration:
        current_time = int(time.time() - start_time)
        current_temp = read_temp.read_temp(temp_sensor)

        log_line = f"{current_time},{current_temp:.2f}\n"
        print(log_line.strip())

        with open(filename, "a") as f:
            f.write(log_line)

        time.sleep(log_interval)

except Exception as e:
    print("Error during step response logging:", e)

finally:
    sub_pwm.duty(0)
    print("Logging complete. Cooling off.")
