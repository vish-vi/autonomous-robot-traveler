from fairis_tools.my_robot import MyRobot
import math

robot = MyRobot()

maze_files = ['../../worlds/Spring26/maze0.xml',
              '../../worlds/Spring26/maze1.xml',
              '../../worlds/Spring26/maze2.xml',
              '../../worlds/Spring26/maze3.xml',
              '../../worlds/Spring26/maze4.xml',
              '../../worlds/Spring26/maze5.xml',
              '../../worlds/Spring26/maze6.xml',
              '../../worlds/Spring26/maze7.xml']

robot.load_environment(maze_files[2])
robot.move_to_start()

DT = robot.timestep / 1000.0
wRadius = 0.045
maxspeed = robot.max_motor_velocity
lfront = 180


def wheelDist():
    enc = robot.get_encoder_readings()
    return enc[0] * wRadius, enc[1] * wRadius


def get_lfront():
    val = robot.get_lidar_range_image()[lfront]
    if math.isinf(val) or math.isnan(val):
        return 10.0
    return val


def direction():
    return robot.get_compass_reading() % 360.0


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def clamp(val, limit):
    return max(-limit, min(limit, val))


def pause(steps=10):
    for _ in range(steps):
        robot.experiment_supervisor.step(robot.timestep)

from fairis_tools.my_robot import MyRobot
import math

robot = MyRobot()

maze_files = ['../../worlds/Spring26/maze0.xml',
              '../../worlds/Spring26/maze1.xml',
              '../../worlds/Spring26/maze2.xml',
              '../../worlds/Spring26/maze3.xml',
              '../../worlds/Spring26/maze4.xml',
              '../../worlds/Spring26/maze5.xml',
              '../../worlds/Spring26/maze6.xml',
              '../../worlds/Spring26/maze7.xml']

robot.load_environment(maze_files[2])
robot.move_to_start()

DT = robot.timestep / 1000.0
wRadius = 0.045
maxspeed = robot.max_motor_velocity
lfront = 180


def wheelDist():
    enc = robot.get_encoder_readings()
    return enc[0] * wRadius, enc[1] * wRadius


def direction():
    return robot.get_compass_reading() % 360.0


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def clamp(val, limit):
    return max(-limit, min(limit, val))


def pause(steps=10):
    for _ in range(steps):
        robot.experiment_supervisor.step(robot.timestep)

def get_lfront():
    val = robot.get_lidar_range_image()[lfront]
    if math.isinf(val) or math.isnan(val):
        return 10.0
    return val



def rotate_pid(deg, Kp=0.02, Ki=0.005, Kd=0.0, tol=0.5):
    pause(10)
    robot.set_right_motor_velocity(0)
    robot.set_left_motor_velocity(0)
    
    start = direction()
    target = (start + deg) % 360.0
    
    prev_err = angle_diff(target, start)
    stable = 0
    step_count = 0
    MAX_STEPS = int(15.0 / DT)
    integral = 0.0

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        step_count += 1
        current = direction()
        error = angle_diff(target, current)

        integral += error * DT
        integral = clamp(integral, 5.0) 

        derivative = (error - prev_err) / DT
        prev_err = error

        u = Kp * error + Ki * integral + Kd * derivative
        u = clamp(u, maxspeed)

        robot.set_left_motor_velocity(-u)
        robot.set_right_motor_velocity(u)

        if abs(error) < tol:
            robot.set_left_motor_velocity(0)
            robot.set_right_motor_velocity(0)
            stable += 1
            if stable >= 25: 
                break
        else:
            stable = 0

        if step_count >= MAX_STEPS:
 
            break

    robot.stop()
    pause(15) 



def lidar_pid(target_dist, Kp=6.0, Ki=0.01, Kd=1.2, tol=0.02):
    robot.stop()
    pause(20) 
    
    locked_heading = direction()
    K_align = 0.1 
    
    initial_val = get_lfront()
    prev_err = initial_val - target_dist
    stable = 0
    integral = 0.00

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        error = get_lfront() - target_dist

        integral += error * DT
        integral = clamp(integral, 1.0)

        derivative = (error - prev_err) / DT
        prev_err = error

        u = Kp * error + Ki * integral + Kd * derivative
        u = clamp(u, maxspeed)

        current_h = direction()
        h_error = angle_diff(locked_heading, current_h)
        
        correction = h_error * K_align

        robot.set_left_motor_velocity(clamp(u - correction, maxspeed))
        robot.set_right_motor_velocity(clamp(u + correction, maxspeed))

        if abs(error) < tol:
            stable += 1
            if stable > 15: break
        else:
            stable = 0
            
    robot.stop()
    pause(5)

def encoder_pid(target_m, Kp=8.0, Ki=0.05, Kd=1.0, tol=0.01):
    pause(10)
    start_l, start_r = wheelDist()
    
    locked_heading = direction() 
    K_straight = 0.1 
    
    integral = 0.0
    prev_err = target_m 
    stable = 0

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        cur_l, cur_r = wheelDist()
        dist = ((cur_l - start_l) + (cur_r - start_r)) / 2.0
        error = target_m - dist

        integral += error * DT
        integral = clamp(integral, 1.0)
        derivative = (error - prev_err) / DT
        prev_err = error

        u = Kp * error + Ki * integral + Kd * derivative
        u = clamp(u, maxspeed)

        current_h = direction()
        heading_error = angle_diff(locked_heading, current_h)
        correction = heading_error * K_straight

        robot.set_left_motor_velocity(clamp(u - correction, maxspeed))
        robot.set_right_motor_velocity(clamp(u + correction, maxspeed))

        if abs(error) < tol:
            stable += 1
            if stable > 15: break
        else:
            stable = 0
    robot.stop()


lidar_pid(1.0)
print("task one done")
encoder_pid(0.5)
print("task two done")
lidar_pid(1.0)
print("task three done")
rotate_pid(180.0)
print("task four done")
lidar_pid(1.0)
print("task five done")
encoder_pid(0.5)
print("task six done")
lidar_pid(1.0)
print("task seven done")
rotate_pid(-180.0)
print("task eight done")
encoder_pid(2.5)
print("task nine done")

pause(3)
