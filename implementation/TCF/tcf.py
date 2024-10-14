import os
import time
from threading import Event, Thread, Lock

# Define the paths for the pipes
TEMP_INFO_PIPE = "/tmp/TEMP_INFO_PIPE"
RESPONSE_PIPE = "/tmp/RESPONSE_PIPE"

# Ensure the named pipes exist
if not os.path.exists(TEMP_INFO_PIPE):
    os.mkfifo(TEMP_INFO_PIPE)

if not os.path.exists(RESPONSE_PIPE):
    os.mkfifo(RESPONSE_PIPE)

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired temperature
        self.integral = 0
        self.prev_error = 0

    def compute(self, current_temp):
        error = self.setpoint - current_temp
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

    def update_setpoint(self, new_setpoint):
        """Updates the setpoint value for the PID controller."""
        self.setpoint = new_setpoint

def read_temp_info():
    """Reads temperature and heater power information from TEMP_INFO_PIPE."""
    with open(TEMP_INFO_PIPE, 'r') as temp_pipe:
        temp_info = temp_pipe.read().strip()
        return temp_info

def format_heater_response(heater_powers):
    """Formats the heater power control signals for RESPONSE_PIPE."""
    return ";".join(map(str, heater_powers))

def write_heater_response(heater_powers):
    """Writes the heater power response to RESPONSE_PIPE."""
    response_message = format_heater_response(heater_powers)
    with open(RESPONSE_PIPE, 'w') as response_pipe:
        response_pipe.write(response_message)
        print(f"TCF Writing to RESPONSE_PIPE: {response_message}")

def process_temperature_data(temp_info, pid_controllers):
    """Processes the temperature data and uses PID control to determine heater power."""
    heater_powers = []
    sensors = temp_info.split(";")

    for i, sensor in enumerate(sensors):
        temp, _ = sensor.split("-")
        temp = float(temp)

        # Calculate heater control signal using PID controller
        heater_output = pid_controllers[i].compute(temp)

        # If PID output > 0, turn on heater (1), otherwise turn it off (0)
        if heater_output > 0:
            heater_powers.append(1)
        else:
            heater_powers.append(0)

    return heater_powers

def get_initial_heater_status():
    """Acquires the initial power status of the heaters HTR-01 to HTR-04 from TEMP_INFO_PIPE."""
    temp_info = read_temp_info()
    sensors = temp_info.split(";")
    heater_status = [int(sensor.split("-")[1]) for sensor in sensors]
    print("Initial Heater Status:")
    for i, status in enumerate(heater_status):
        print(f"HTR-0{i+1}: {'ON' if status == 1 else 'OFF'}")
    return heater_status

def run_tcf(frequency, pid_controllers, enabled_event):
    """Runs the TCF and processes temperature data at a given frequency."""
    period = 1.0 / frequency

    while enabled_event.is_set():
        # 1. Read temperature data from TEMP_INFO_PIPE
        temp_info = read_temp_info()
        print(f"TCF Received from TEMP_INFO_PIPE: {temp_info}")

        # 2. Process the temperature data and control heaters using PID
        heater_powers = process_temperature_data(temp_info, pid_controllers)

        # 3. Write heater control response to RESPONSE_PIPE
        write_heater_response(heater_powers)

        # 4. Wait for the next cycle (based on the frequency)
        time.sleep(period)

def enable_tcf(frequency, Kp, Ki, Kd, setpoints, pid_controllers_lock):
    """Enables the TCF and starts the control loop."""
    enabled_event.set()

    # Create PID controllers for each thermistor
    with pid_controllers_lock:
        pid_controllers.clear()
        pid_controllers.extend([PIDController(Kp, Ki, Kd, setpoint) for setpoint in setpoints])

    # Start a new thread to run the TCF
    tcf_thread = Thread(target=run_tcf, args=(frequency, pid_controllers, enabled_event))
    tcf_thread.start()

def disable_tcf():
    """Disables the TCF and ensures all heaters are turned off."""
    enabled_event.clear()
    # Ensure heaters are turned off
    write_heater_response([0, 0, 0, 0])
    print("TCF Disabled. All heaters turned off.")

def update_setpoint(pid_controllers, pid_controllers_lock):
    """Allows the user to update the setpoint of specific thermistors at runtime."""
    while enabled_event.is_set():
        try:
            user_input = input("Enter 'thermistor_index new_setpoint' or 'all new_setpoint' to change setpoints (e.g., '1 10' or 'all 5'): ")
            if user_input.lower() == "exit":
                break
            args = user_input.split()

            if args[0].lower() == "all":
                new_setpoint = float(args[1])
                if -20 <= new_setpoint <= 20:
                    with pid_controllers_lock:
                        for pid in pid_controllers:
                            pid.update_setpoint(new_setpoint)
                    print(f"Setpoints updated to {new_setpoint} for all thermistors.")
                else:
                    print("Setpoint must be between -20 and 20.")
            else:
                thermistor_index = int(args[0]) - 1  # Assuming thermistor indices are 1-based
                new_setpoint = float(args[1])
                if -20 <= new_setpoint <= 20:
                    with pid_controllers_lock:
                        pid_controllers[thermistor_index].update_setpoint(new_setpoint)
                    print(f"Setpoint updated for THERM-{thermistor_index + 1} to {new_setpoint}.")
                else:
                    print("Setpoint must be between -20 and 20.")
        except (IndexError, ValueError):
            print("Invalid input. Please enter 'thermistor_index new_setpoint' or 'all new_setpoint'.")

if __name__ == "__main__":
    enabled_event = Event()
    pid_controllers = []
    pid_controllers_lock = Lock()  # Lock to ensure thread-safe access to PID controllers

    try:
        # Acquire the initial heater status before enabling TCF
        initial_heater_status = get_initial_heater_status()

        # User inputs for PID parameters and frequency
        frequency = float(input("Enter the frequency (1-5 Hz) for the TCF to run: "))
        Kp = float(input("Enter the Proportional Gain (Kp): "))
        Ki = float(input("Enter the Integral Gain (Ki): "))
        Kd = float(input("Enter the Derivative Gain (Kd): "))
        setpoints = [float(input(f"Enter setpoint for THERM-{i+1}: ")) for i in range(4)]

        # Enable TCF
        enable_tcf(frequency, Kp, Ki, Kd, setpoints, pid_controllers_lock)

        # Start a thread to listen for runtime setpoint updates
        setpoint_thread = Thread(target=update_setpoint, args=(pid_controllers, pid_controllers_lock))
        setpoint_thread.start()

        input("Press Enter to disable the TCF...\n")

        # Disable TCF
        disable_tcf()

        # Ensure setpoint_thread ends gracefully
        setpoint_thread.join()

    except KeyboardInterrupt:
        print("TCF process interrupted. Exiting...")



