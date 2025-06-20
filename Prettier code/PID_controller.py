import time

# --- PID controller ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(0, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral = 0
        self.last_error = 0
        self.last_time = time.ticks_ms()
        self.last_output = 0 # To store the last computed output for dt=0 case

    def compute(self, measurement, current_temp, ):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000

        # --- DEBUG PRINTS START (at start of compute) ---
        print(f"PID Debug (Setpoint={self.setpoint:.2f}, Meas={measurement:.2f}):")
        # --- DEBUG PRINTS END ---

        if dt == 0:
            print("  dt=0, returning last output.")
            return self.last_output

        error = measurement - self.setpoint

        P = self.Kp * error

        derivative_term = 0
        if dt > 0:
            derivative_term = (error - self.last_error) / dt

        # Calculate the potential output based on P, current I, and D (before clamping)
        unclamped_output = P + (self.Ki * self.integral) + (self.Kd * derivative_term)

        # --- ANTI-WINDUP LOGIC (Conditional Integration) ---
        # Integrate only if the unclamped output is within limits,
        # OR if it's outside limits but the error is trying to bring it back within limits.
        
        integrate_cond = False
        if (unclamped_output >= self.output_limits[0] and unclamped_output <= self.output_limits[1]):
            integrate_cond = True # Case 1: Within limits, always integrate
        elif (unclamped_output > self.output_limits[1] and error < 0):
            integrate_cond = True # Case 2: Above max, but error is trying to reduce it
        elif (unclamped_output < self.output_limits[0] and error > 0):
            integrate_cond = True # Case 3: Below min, but error is trying to increase it

        if integrate_cond:
            self.integral += error * dt
        # --- END ANTI-WINDUP LOGIC ---

        I = self.Ki * self.integral # Recalculate I for the actual output and debug prints
        D = self.Kd * derivative_term # Corrected to use derivative_term

        # Final output calculation
        output = P + I + D
        clamped_output = max(self.output_limits[0], min(output, self.output_limits[1]))

        self.last_error = error
        self.last_time = now
        self.last_output = clamped_output

        if current_temp <= self.setpoint:
            cooler.value(0)
            fan.value(0)
        else:
            cooler.value(1)
            fan.value(1)

        '''
        # --- DEBUG PRINTS START (at end of compute) ---
        print(f"  Error={error:.2f}, dt={dt:.2f}, Integral={self.integral:.2f}")
        print(f"  P={P:.2f}, I={I:.2f}, D={D:.2f}")
        print(f"  Unclamped Output={unclamped_output:.2f}, Clamped Output={clamped_output:.2f}")
        # --- DEBUG PRINTS END ---
        '''

        return clamped_output