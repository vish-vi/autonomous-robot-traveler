from fairis_tools.my_robot import MyRobot
import math

wRadius = 0.045  
length = 0.184    
maxspeed = 0.81    
maxrspeed = 18  

l = 2.0  
w = 4.0   
r1 = 1.0  
r2 = 2.0

L = 3
W = 1
R1 = 1.5
R2 = 0.5


lspeed = 0.5   
rspeed = 2   

robot = MyRobot()


maze_files = ['../../worlds/Spring26/maze0.xml',
              '../../worlds/Spring26/maze1.xml',
              '../../worlds/Spring26/maze2.xml',
              '../../worlds/Spring26/maze3.xml',
              '../../worlds/Spring26/maze4.xml',
              '../../worlds/Spring26/maze5.xml',
              '../../worlds/Spring26/maze6.xml',
              '../../worlds/Spring26/maze7.xml']

robot.load_environment(maze_files[1])  

def setspeed(lval, rval):
    robot.left_motor.setVelocity(lval)
    robot.right_motor.setVelocity(rval)


def stop():
    setspeed(0, 0)


def get_compass_d():
    
    d = robot.get_compass_reading()
    
    while d < 0:
        d += 360
    while d >= 360:
        d -= 360
    
    return d


def encoderdist():
    left_rad = robot.get_left_motor_encoder_reading()
    right_rad = robot.get_right_motor_encoder_reading()
    
    lDistance = left_rad * wRadius
    rDistance = right_rad * wRadius
    
    return lDistance, rDistance


def straight(distance):

    il, ir = encoderdist()
    
    o = lspeed / wRadius
    setspeed(o, o)
    
    while robot.step() != -1:
        left_dist, right_dist = encoderdist()
        traveled = ((left_dist - il) + (right_dist - ir)) / 2
        
        if traveled >= distance:
            break
    
    stop()
    

def circle(radius, clockwise=False):
    
    v = lspeed
    o = v / radius  
    
    v_left = v - (length / 2) * o
    v_right = v + (length / 2) * o
    
    if clockwise:
        v_left, v_right = v_right, v_left
    
    o_left = v_left / wRadius
    o_right = v_right / wRadius
    
    setspeed(o_left, o_right)
    
    cc = 2 * math.pi * radius
    duration = cc / v
    steps = int(duration * 1000 / robot.timestep)
    
    direction = "clockwise" if clockwise else "counterclockwise"
    
    for _ in range(steps):
        if robot.step() == -1:
            break
    
    stop()



def turn(angle_deg):
   
    initial_d = get_compass_d()
    target_d = (initial_d + angle_deg) % 360
    
    whspeed = ((rspeed * length / 2) / wRadius)/2
    
    if angle_deg > 0:  
        setspeed(-whspeed, whspeed)
    else:  
        setspeed(whspeed, -whspeed)
    
    while robot.step() != -1:
        current_d = get_compass_d()
        
        diff = target_d - current_d
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
            
        if abs(diff) < 1:  
            break
    
    stop()


straight(w / 2)
turn(-90)  
    
straight(l)
turn(-90) 
    
straight(w)
turn(-90)

straight(l)
turn(-90)

straight(w / 2)    


circle(r1, clockwise=False)


circle(r2, clockwise=True)

straight(W / 2)
turn(-90)  
    
straight(L)
turn(-90) 
    
straight(W)
turn(-90)

straight(L)
turn(-90)

straight(W / 2)    


circle(R1, clockwise=False)


circle(R2, clockwise=True)
