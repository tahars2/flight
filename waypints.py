import asyncio
import threading
from pynput import keyboard
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

# Global variables
input_queue = asyncio.Queue()
drone = None
loop = None

# Movement states
movement = {"w": False, "a": False, "s": False, "d": False}
altitude_change = {"8": False, "2": False}
yaw_change = {"4": False, "6": False}
takeoff_flag = False
landing_flag = False

# Function to connect the drone
async def connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    return drone

# Control drone logic
async def mavsdk_control():
    global loop, drone, takeoff_flag, landing_flag
    loop = asyncio.get_event_loop()
    drone = await connect_drone()

    try:
        velocity = 2.0  # Default velocity (in meters per second)
        altitude = -5.0  # Default altitude
        yaw = 0.0  # Default yaw
        x, y = 0.0, 0.0

        while True:
            # Check for takeoff
            if takeoff_flag:
                print("Taking off...")
                await drone.action.arm()
                await drone.action.takeoff()
                await asyncio.sleep(10)
                print("Starting offboard mode...")
                await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, altitude, yaw))
                await drone.offboard.start()
                takeoff_flag = False

            # Check for landing
            if landing_flag:
                print("Landing...")
                await drone.action.land()
                await asyncio.sleep(10)
                landing_flag = False

            # Check movement states and update positions
            if movement["w"]:
                x += velocity * 0.1
            if movement["s"]:
                x -= velocity * 0.1
            if movement["a"]:
                y -= velocity * 0.1
            if movement["d"]:
                y += velocity * 0.1

            # Check for altitude changes
            if altitude_change["8"]:
                altitude -= 0.1  # Move up
            if altitude_change["2"]:
                altitude += 0.1  # Move down

            # Check for yaw changes
            if yaw_change["4"]:
                yaw += 5.0  # Rotate to the right
            if yaw_change["6"]:
                yaw -= 5.0  # Rotate to the left

            # Set new position and yaw
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, altitude, yaw))
            await asyncio.sleep(0.1)  # Smooth movement

    except Exception as e:
        print(f"An error occurred: {e}")

# Function to handle key presses
def on_press(key):
    global takeoff_flag, landing_flag
    try:
        if key.char == "w":
            movement["w"] = True
        elif key.char == "s":
            movement["s"] = True
        elif key.char == "a":
            movement["a"] = True
        elif key.char == "d":
            movement["d"] = True
        elif key.char == "8":
            altitude_change["8"] = True
        elif key.char == "2":
            altitude_change["2"] = True
        elif key.char == "4":
            yaw_change["4"] = True
        elif key.char == "6":
            yaw_change["6"] = True
        elif key.char == "t":
            takeoff_flag = True
        elif key.char == "l":
            landing_flag = True
        elif key.char == "q":
            print("Emergency stop. Returning to launch.")
            asyncio.run_coroutine_threadsafe(input_queue.put("q"), loop)
    except AttributeError:
        pass  # Ignore special keys

def on_release(key):
    try:
        if key.char == "w":
            movement["w"] = False
        elif key.char == "s":
            movement["s"] = False
        elif key.char == "a":
            movement["a"] = False
        elif key.char == "d":
            movement["d"] = False
        elif key.char == "8":
            altitude_change["8"] = False
        elif key.char == "2":
            altitude_change["2"] = False
        elif key.char == "4":
            yaw_change["4"] = False
        elif key.char == "6":
            yaw_change["6"] = False
        elif key.char == "q":
            print("Emergency stop. Returning to launch.")
            asyncio.run_coroutine_threadsafe(input_queue.put("q"), loop)
            return False  # Stop the listener when 'q' is pressed
    except AttributeError:
        pass  # Ignore special keys

# Function to listen for key presses in a separate thread
def listen_for_keys():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# Main function to run the program
async def main():
    # Run the keyboard listener in a separate thread
    listener_thread = threading.Thread(target=listen_for_keys)
    listener_thread.start()

    # Run the drone control logic
    control_task = asyncio.create_task(mavsdk_control())
    await control_task

if __name__ == "__main__":
    asyncio.run(main())
