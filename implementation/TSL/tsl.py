import os
import time

# Define the paths for the pipes
TEMP_INFO_PIPE = "/tmp/TEMP_INFO_PIPE"
RESPONSE_PIPE = "/tmp/RESPONSE_PIPE"

# Create named pipes if they don't exist
if not os.path.exists(TEMP_INFO_PIPE):
    os.mkfifo(TEMP_INFO_PIPE)

if not os.path.exists(RESPONSE_PIPE):
    os.mkfifo(RESPONSE_PIPE)

# Simulated temperature and heater data
thermistor_temps = [30.5, 28.7, 32.1, 29.8]  # Example temperatures for 4 thermistors
heater_power_status = [1, 0, 1, 0]  # Example power statuses for 4 heaters

# Initialize the monotonic clock
monotonic_clock = 0

def format_temp_info_message(temps, powers, clock):
    """
    Formats the temperature, heater power data, and clock into the correct message format.
    """
    message_parts = [f"{clock:04x}"]  # Add the clock value in hexadecimal format
    for i in range(4):
        part = f"{temps[i]:.1f}-{powers[i]}"
        message_parts.append(part)
    return ";".join(message_parts)


def set_heater_status(heater_id, status):
    """
    Sets the status of a given heater.
    """
    if 0 <= heater_id < len(heater_power_status):
        heater_power_status[heater_id] = status
        return f"HEATER-{heater_id+1:02d} set to {status}"
    return "ERROR: Invalid heater ID"


def read_heater_response():
    """
    Reads the heater power response from RESPONSE_PIPE or handles requests.
    """
    with open(RESPONSE_PIPE, 'r') as response_pipe:
        response = response_pipe.read().strip()
        if response:
            return response
        return ""
def handle_heater_status_request(request):
    """
    Handles API requests to get or set the current heater power status.
    """
    if request == "GET_HEATER_STATUS":
        return "; ".join([f"HEATER-{i+1:02d}:{status}" for i, status in enumerate(heater_power_status)])
    elif request.startswith("SET_HEATER_"):
        try:
            parts = request.split('_')
            heater_id = int(parts[2]) - 1
            status = int(parts[3])
            return set_heater_status(heater_id, status)
        except (IndexError, ValueError):
            return "ERROR: Invalid command format"
    return "ERROR: Invalid request"


def handle_temperature_request(request):
    """
    Handles API requests to get the current temperature values.
    """
    if request == "GET_TEMP_ALL":
        # Format all thermistor temperatures
        return "; ".join([f"THERM-{i+1:02d}:{thermistor_temps[i]:.1f}" for i in range(4)])
    elif request.startswith("GET_TEMP_"):
        # Extract specific thermistor ID and return its temperature
        try:
            therm_id = int(request.split('_')[-1]) - 1
            if 0 <= therm_id < len(thermistor_temps):
                return f"THERM-{therm_id+1:02d}:{thermistor_temps[therm_id]:.1f}"
        except ValueError:
            return "ERROR: Invalid thermistor ID"
    return "ERROR: Invalid request"

def read_heater_response():
    """
    Reads the heater power response from RESPONSE_PIPE or handles requests.
    """
    with open(RESPONSE_PIPE, 'r') as response_pipe:
        response = response_pipe.read().strip()
        if response:
            return response
        return ""

def update_temperatures(thermistor_temps):
    """
    Updates the temperature values based on environmental conditions.
    """
    for i in range(len(thermistor_temps)):
        if i % 2 == 0:
            thermistor_temps[i] += 0.1
        else:
            thermistor_temps[i] -= 0.1

def check_period(clock):
    """
    Checks the lower 8 bits of the clock to determine the current period.
    """
    lower_8_bits = clock & 0xFF
    if 0x00 <= lower_8_bits <= 0x1F or 0x60 <= lower_8_bits <= 0xFF:
        return "NORMAL"
    elif 0x20 <= lower_8_bits <= 0x3F:
        return "ECLIPSE"
    elif 0x40 <= lower_8_bits <= 0x5F:
        return "SUN_EXPOSURE"
    else:
        return "OTHER"

def update_temperatures_based_on_heater(thermistor_temps, heater_power_status, period):
    """
    Updates the temperature values based on the heater power state and the current period.
    """
    for i in range(len(thermistor_temps)):
        if period == "ECLIPSE":
            if heater_power_status[i] == 1:
                thermistor_temps[i] += 4
            else:
                thermistor_temps[i] -= 7
        elif period == "SUN_EXPOSURE":
            if heater_power_status[i] == 1:
                thermistor_temps[i] += 7
            else:
                thermistor_temps[i] -= 1
        else:  # NORMAL or OTHER periods
            if heater_power_status[i] == 1:
                thermistor_temps[i] += 1
            else:
                thermistor_temps[i] -= 1

def run_tsl():
    global monotonic_clock
    while True:
        # Increment the monotonic clock
        monotonic_clock = (monotonic_clock + 1) % 0x10000  # 16-bit overflow

        # Simulate temperature updates based on environmental factors
        update_temperatures(thermistor_temps)

        # Check the current period based on the monotonic clock
        current_period = check_period(monotonic_clock)
        print(f"Current period: {current_period}")

        # Update temperatures based on heater power state and current period
        update_temperatures_based_on_heater(thermistor_temps, heater_power_status, current_period)

        # 1. Read commands from RESPONSE_PIPE (could be a heater response or temperature request)
        request = read_heater_response()
        if request:
            print(f"TSL Received: {request}")
            if request.startswith("GET_TEMP"):
                # Handle temperature request
                response_message = handle_temperature_request(request)
            elif request.startswith("SET_HEATER_") or request == "GET_HEATER_STATUS":
                # Handle heater status request
                response_message = handle_heater_status_request(request)
            else:
                # Assume it's a heater command
                heater_power_status[:] = list(map(int, request.split(';')))  # Update heater power status
                response_message = "Heater status updated"

            with open(TEMP_INFO_PIPE, 'w') as temp_pipe:
                print(f"TSL Responding with: {response_message}")
                temp_pipe.write(response_message)

        # Simulate a delay for the cyclic process (5Hz frequency)
        time.sleep(0.2)

if __name__ == "__main__":
    try:
        run_tsl()
    except KeyboardInterrupt:
        print("TSL process interrupted. Exiting...")
