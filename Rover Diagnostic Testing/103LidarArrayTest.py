from qset_lib import Rover
from time import sleep
import signal

def main():
    rover = Rover()  # Initializes the rover and connects to its data.
    signal.signal(signal.SIGINT, signal.default_int_handler)  # Allows for KeyboardInterrupt to work.

    try:
        # Fetch and print the LiDAR data once.
        lidar_data = rover.laser_distances
        num_beams = len(lidar_data)  # Determine the number of beams in the LiDAR array.
        print(f"Number of beams in the LiDAR array: {num_beams}")
        
        # Optionally, print the first few distances to get an idea of the data.
        print("Sample LiDAR distances (first 10 beams):")
        for i in range(min(10, num_beams)):  # Ensures the loop doesn't exceed the array length.
            print(f"Beam {i+1}: {lidar_data[i]} meters")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")

    finally:
        # Ensures that the rover is stopped before exiting the program.
        rover.send_command(0, 0)
        print("Rover stopped.")

if __name__ == "__main__":
    main()
