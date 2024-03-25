import math
import time
import signal
from qset_lib import Rover

# Configuration values for rover behavior
turn_gain = 0.5  # Gain for turning speed adjustment
forward_gain = 0.1  # Gain for forward speed adjustment
angular_linear_weight = 8  # Weight for balancing angular and linear movements
min_fwd_vel = 0.4  # Minimum forward velocity
max_fwd_vel = 2.0  # Maximum forward velocity
x_goal = 20  # X-coordinate of the goal
y_goal = 20  # Y-coordinate of the goal
k = 185  # Constant for potential field calculation
sqr_error = 0.5  # Square of the acceptable error margin to the goal
critical_distance = 0.9  # Minimum distance to obstacle before taking avoidance action
obstacle_memory = {'left': False, 'right': False}  # Memory of obstacles on the left and right sides
last_obstacle_time = time.time()  # Track the time since the last obstacle was detected
obstacle_avoidance_interval = 1  # Initial interval for obstacle avoidance checks

def signal_handler(sig, frame):
    from qset_lib import Rover
    print("\nRover stopped by user. Exiting...")
    rover.send_command(0, 0)  # Stop the rover
    exit(0)

def direct_obstacle_avoidance(rover):
    global obstacle_memory
    front_angles = range(-15, 16)
    left_angles = range(-60, -15)
    right_angles = range(16, 60)
    
    update_obstacle_memory(rover, left_angles, right_angles)
    front_distances = [rover.laser_distances[angle] for angle in front_angles if 0 <= angle < len(rover.laser_distances)]
    if not front_distances:
        return False, 0, 0

    min_distance = min(front_distances)
    if min_distance < critical_distance:
        avoidance_turn = max_fwd_vel
        return True, -avoidance_turn, avoidance_turn

    return False, 0, 0

def update_obstacle_memory(rover, left_angles, right_angles):
    global obstacle_memory
    left_distances = [rover.laser_distances[angle] for angle in left_angles if 0 <= angle < len(rover.laser_distances)]
    right_distances = [rover.laser_distances[angle] for angle in right_angles if 0 <= angle < len(rover.laser_distances)]

    obstacle_memory['left'] = any(distance < critical_distance for distance in left_distances)
    obstacle_memory['right'] = any(distance < critical_distance for distance in right_distances)

def traverse_adjusted_for_memory(rover, target_x, target_y):
    global obstacle_memory, last_obstacle_time, obstacle_avoidance_interval
    avoidance_needed, left_avoid_cmd, right_avoid_cmd = direct_obstacle_avoidance(rover)
    if avoidance_needed:
        rover.send_command(left_avoid_cmd, right_avoid_cmd)
        last_obstacle_time = time.time()  # Update the time since the last obstacle was detected
        obstacle_avoidance_interval = 0.5  # Reduce interval due to obstacle detection
        return False

    curr_x, curr_y = rover.x, rover.y
    curr_heading = math.radians(rover.heading)
    delta_x, delta_y = target_x - curr_x, target_y - curr_y
    target_heading = math.atan2(delta_y, delta_x)
    delta_heading = (target_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
    delta_dist = math.sqrt(delta_x ** 2 + delta_y ** 2)

    if delta_dist < sqr_error:
        rover.send_command(0, 0)
        return True

    if obstacle_memory['left']:
        delta_heading -= math.pi / 4
    if obstacle_memory['right']:
        delta_heading += math.pi / 4

    turn_cmd = delta_heading * turn_gain
    fwd_cmd = max(min(delta_dist * forward_gain - angular_linear_weight * abs(delta_heading), max_fwd_vel), min_fwd_vel)
    right_cmd, left_cmd = fwd_cmd + turn_cmd, fwd_cmd - turn_cmd

    rover.send_command(left_cmd, right_cmd)
    
    # Adjust interval based on time since last obstacle
    if time.time() - last_obstacle_time > 5:
        obstacle_avoidance_interval = 1.5  # Increase interval if no recent obstacles

    time.sleep(obstacle_avoidance_interval)  # Dynamic sleep interval based on obstacle presence
    return False

def fields(rover):
    lidar_filtered = [dist if dist < 15 else 0 for dist in rover.laser_distances]
    obstacles_x = [dist * math.sin(angle * math.pi / 180) for angle, dist in enumerate(lidar_filtered) if dist != 0]
    obstacles_y = [dist * math.cos(angle * math.pi / 180) for angle, dist in enumerate(lidar_filtered) if dist != 0]

    field_total_x = x_goal + sum(k / (x**2 + y**2) * math.cos(math.atan2(y, x)) for x, y in zip(obstacles_x, obstacles_y))
    field_total_y = y_goal + sum(k / (x**2 + y**2) * math.sin(math.atan2(y, x)) for x, y in zip(obstacles_x, obstacles_y))

    return field_total_x, field_total_y

def main():
    rover = Rover()
    time.sleep(1)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        while True:
            field_x, field_y = fields(rover)
            if traverse_adjusted_for_memory(rover, field_x, field_y):
                break
    except KeyboardInterrupt:
        rover.send_command(0, 0)
        print("\nRover stopped by user. Exiting...")
    finally:
        rover.send_command(0, 0)
        print("Cleanup complete, program terminated gracefully.")

if __name__ == "__main__":
    main()
