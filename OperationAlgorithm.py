import math
import time
import signal
from qset_lib import Rover

# Configuration values for rover behavior
turn_gain = 0.5  # Gain for turning speed adjustment
forward_gain = 0.1  # Gain for forward speed adjustment
angular_linear_weight = 8  # Weight for balancing angular and linear movements
min_fwd_vel = 1.55  # Minimum forward velocity
max_fwd_vel = 1.85  # Maximum forward velocity
x_goal = 23  # X-coordinate of the goal
y_goal = 23  # Y-coordinate of the goal
k = 185  # Constant for potential field calculation
sqr_error = 0.5  # Square of the acceptable error margin to the goal
critical_distance = 1  # Minimum distance to obstacle before taking avoidance action
obstacle_memory = {'left': False, 'right': False}  # Memory of obstacles on the left and right sides
heading_threshold = 0.14  # Acceptable threshold for heading alignment in radians
non_critical_distance = 8

rover = Rover() # Globaly initialize the Rover object

def signal_handler(sig, frame):
    """
    Handles the SIGINT signal (typically triggered by Ctrl+C) by stopping the rover.
    """
    print("\nRover stopped by user. Exiting...")
    rover.send_command(0, 0)  # Stop the rover
    exit(0)  # Exit the program gracefully


def direct_obstacle_avoidance(rover):
    """
    Checks for obstacles and returns heading adjustments rather than executing turn commands.
    """
    front_angles = range(-15, 16)
    front_left_angles = range(-15, 0)
    front_right_angles = range(1, 16)

    front_left_distances = [rover.laser_distances[angle] for angle in front_left_angles if 0 <= angle < len(rover.laser_distances)]
    front_right_distances = [rover.laser_distances[angle] for angle in front_right_angles if 0 <= angle < len(rover.laser_distances)]

    min_left_distance = min(front_left_distances) if front_left_distances else float('inf')
    min_right_distance = min(front_right_distances) if front_right_distances else float('inf')

    heading_adjustment = 0
    if min_left_distance < non_critical_distance and min_left_distance != float('inf'):
        heading_adjustment = turn_gain  # Adjust heading slightly to the right
        print ("Adjusting heading to the right due to obstacle on the left")
    elif min_right_distance < non_critical_distance and min_right_distance != float('inf'):
        heading_adjustment = -turn_gain  # Adjust heading slightly to the left
        print ("Adjusting heading to the left due to obstacle on the right")

    return heading_adjustment

def update_obstacle_memory(rover, left_angles, right_angles):
    """
    Updates the obstacle memory by checking for obstacles within specified angle ranges.
    """
    global obstacle_memory
    # Calculate distances to obstacles on the left and right
    left_distances = [rover.laser_distances[angle] for angle in left_angles if 0 <= angle < len(rover.laser_distances)]
    right_distances = [rover.laser_distances[angle] for angle in right_angles if 0 <= angle < len(rover.laser_distances)]

    # Update obstacle memory based on detected obstacles
    obstacle_memory['left'] = any(distance < critical_distance for distance in left_distances)
    obstacle_memory['right'] = any(distance < critical_distance for distance in right_distances)

def traverse_adjusted_for_memory(rover, target_x, target_y):
    """
    Adjusts navigation to incorporate heading adjustments directly into final commands.
    """
    # Calculate heading adjustment for non-critical obstacles
    heading_adjustment = direct_obstacle_avoidance(rover)

    curr_x, curr_y = rover.x, rover.y
    curr_heading = math.radians(rover.heading)
    delta_x, delta_y = target_x - curr_x, target_y - curr_y
    target_heading = math.atan2(delta_y, delta_x)
    delta_heading = (target_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi + heading_adjustment
    delta_dist = math.sqrt(delta_x ** 2 + delta_y ** 2)

    if delta_dist < sqr_error:
        rover.send_command(0, 0)
        print("Goal reached or very close.")
        return True

    turn_cmd = delta_heading * turn_gain
    fwd_cmd = max(min(delta_dist * forward_gain - angular_linear_weight * abs(delta_heading), max_fwd_vel), min_fwd_vel)
    right_cmd, left_cmd = fwd_cmd + turn_cmd, fwd_cmd - turn_cmd
    print("Sending commands: Left - {}, Right - {}".format(left_cmd, right_cmd))
    time.sleep(0.2)
    rover.send_command(left_cmd, right_cmd)

    return False

def fields(rover):
    """
    Calculates a potential field for navigation based on the goal position, rover's heading,
    and detected obstacles, with a preference for keeping obstacles to the side.
    """
    print("Calculating potential field...")
    lidar_filtered = [dist if dist < 15 else 0 for dist in rover.laser_distances]
    obstacles_x = [dist * math.sin(angle * math.pi / 180) for angle, dist in enumerate(lidar_filtered) if dist != 0]
    obstacles_y = [dist * math.cos(angle * math.pi / 180) for angle, dist in enumerate(lidar_filtered) if dist != 0]

    # Adjust the potential field calculation
    goal_attraction_x = x_goal - rover.x
    goal_attraction_y = y_goal - rover.y
    field_total_x = goal_attraction_x + sum(k / (x**2 + y**2) * math.cos(math.atan2(y, x) - rover.heading * math.pi / 180) for x, y in zip(obstacles_x, obstacles_y))
    field_total_y = goal_attraction_y + sum(k / (x**2 + y**2) * math.sin(math.atan2(y, x) - rover.heading * math.pi / 180) for x, y in zip(obstacles_x, obstacles_y))

    return field_total_x, field_total_y

def initial_orientation_turn(rover, target_x, target_y):
    """
    Adjusts the rover's heading towards the target coordinates without forward movement.
    """
    print("Adjusting initial orientation...")
    curr_x, curr_y = rover.x, rover.y
    curr_heading = math.radians(rover.heading)
    target_heading = math.atan2(target_y - curr_y, target_x - curr_x)
    delta_heading = (target_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
    
    while abs(delta_heading) > heading_threshold:
        turn_cmd = delta_heading * turn_gain
        rover.send_command(-turn_cmd, turn_cmd)  # Only turning, no forward movement
        time.sleep(0.1)  # Short delay for continuous adjustment
        print("Adjusting...")
        
        # Update current heading and recalculate delta_heading
        curr_heading = math.radians(rover.heading)
        delta_heading = (target_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
    
    rover.send_command(0,0)
    print("Rover Done Initial Turn")
    time.sleep(0.2)


def main():
    rover = Rover()  # Initialize the rover
    time.sleep(1)  # Wait for the rover to be fully ready
    signal.signal(signal.SIGINT, signal_handler)
    initial_orientation_turn(rover, x_goal, y_goal)

    try:
        while True:
            # Calculate the potential field-based target and navigate towards it
            field_x, field_y = fields(rover)
            if traverse_adjusted_for_memory(rover, field_x, field_y):
                break  # Goal reached, exit loop
            time.sleep(0.1)  # Short delay between navigation cycles
    except KeyboardInterrupt:
        rover.send_command(0, 0)  # Stop the rover if interrupted
        print("\nRover stopped by user. Exiting...")
        pass
    finally:
        rover.send_command(0, 0)  # Ensure the rover is stopped on exit
        print("Cleanup complete, program terminated gracefully.")

if __name__ == "__main__":
    main()
