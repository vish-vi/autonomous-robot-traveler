from fairis_tools.my_robot import MyRobot
import math
import sys
import numpy as np
from collections import deque

sys.path.insert(0, r'C:\Users\wishy\FAIRIS-Lite')
from fairis_tools.my_robot import MyRobot

robot = MyRobot()
robot.load_environment(r'C:\Users\wishy\FAIRIS-Lite\WebotsSim\worlds\Spring26\maze8.xml')
robot.move_to_start()
timestep = robot.timestep



# Configuration 

TARGET_CELL = 21     
FWD_SPEED       = 4.0
TURN_SPEED      = 1.5
HEADING_TOL_DEG = 3.0

CARDINAL       = ['N', 'E', 'S', 'W']
CARDINAL_IDX   = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
DIR_HEADING    = {'N': 90.0, 'E': 0.0, 'S': 270.0, 'W': 180.0}
STEP_OFFSET    = {'N': -5, 'E': 1, 'S': 5, 'W': -1}

# Grid Layout 
GRID_WALLS = {
     1: (1,  0,  0,  1),
     2: (1,  0,  1,  0),
     3: (1,  0,  1,  0),
     4: (1,  0,  1,  0),
     5: (1,  1,  0,  0),
     6: (0,  0,  1,  1),
     7: (1,  0,  1,  0),
     8: (1,  1,  0,  0),
     9: (1,  0,  1,  1),
    10: (0,  1,  1,  0),
    11: (1,  0,  0,  1),
    12: (1,  1,  0,  0),
    13: (0,  0,  1,  1),
    14: (1,  0,  1,  0),
    15: (1,  1,  0,  0),
    16: (0,  1,  0,  1),
    17: (0,  0,  1,  1),
    18: (1,  0,  1,  0),
    19: (1,  0,  1,  0),
    20: (0,  1,  1,  0),
    21: (0,  0,  1,  1),
    22: (1,  0,  1,  0),
    23: (1,  0,  1,  0),
    24: (1,  0,  1,  0),
    25: (1,  1,  1,  0),
}

# Grid Utilities

def grid_position(cid):
    return (cid - 1) // 5, (cid - 1) % 5

def coords_to_cell(wx, wy):
    col = max(0, min(4, int(round(wx)) + 2))
    row = max(0, min(4, 2 - int(round(wy))))
    return row * 5 + col + 1

def passable_neighbors(cid):
    walls = GRID_WALLS[cid]
    row, col = grid_position(cid)
    reachable = []
    for idx, d in enumerate(CARDINAL):
        if walls[idx] == 1:
            continue                        
        if d == 'N' and row == 0: continue   
        if d == 'S' and row == 4: continue
        if d == 'E' and col == 4: continue
        if d == 'W' and col == 0: continue
        reachable.append((cid + STEP_OFFSET[d], d))
    return reachable

def step_direction(src, dst):
    delta = dst - src
    lookup = {-5: 'N', 1: 'E', 5: 'S', -1: 'W'}
    if delta not in lookup:
        raise ValueError(f"Cells {src} and {dst} are not adjacent (delta={delta})")
    return lookup[delta]

# Motor & Movement Helpers 
def halt():
    robot.set_left_motor_velocity(0)
    robot.set_right_motor_velocity(0)
    robot.experiment_supervisor.step(timestep)

def compass_direction():
    hdg = robot.get_compass_reading() % 360
    if   45  <= hdg < 135: return 'N'
    elif 135 <= hdg < 225: return 'W'
    elif 225 <= hdg < 315: return 'S'
    else:                  return 'E'

def face_direction(target):
    goal_hdg = DIR_HEADING[target]
    while robot.experiment_supervisor.step(timestep) != -1:
        hdg  = robot.get_compass_reading()
        err  = (goal_hdg - hdg + 360.0) % 360.0
        if err > 180.0:
            err -= 360.0
        if abs(err) < HEADING_TOL_DEG:
            break
        spd = TURN_SPEED if err > 0 else -TURN_SPEED
        robot.set_left_motor_velocity(-spd)
        robot.set_right_motor_velocity(spd)
    halt()

def advance_one_cell():
    anchor = robot.experiment_supervisor.getSelf().getPosition()
    robot.set_left_motor_velocity(FWD_SPEED)
    robot.set_right_motor_velocity(FWD_SPEED)
    while robot.experiment_supervisor.step(timestep) != -1:
        cur  = robot.experiment_supervisor.getSelf().getPosition()
        dist = math.sqrt((cur[0] - anchor[0])**2 + (cur[1] - anchor[1])**2)
        if dist >= 0.98:
            break
    halt()
    
# BFS Path Planner 
def plan_route(origin, destination):
    if origin == destination:
        return [origin]

    came_from = {destination: None}
    frontier  = deque([destination])

    while frontier:
        node = frontier.popleft()
        for adj, _ in passable_neighbors(node):
            if adj in came_from:
                continue
            came_from[adj] = node
            if adj == origin:
                route, cur = [], origin
                while cur is not None:
                    route.append(cur)
                    cur = came_from[cur]
                return route          
            frontier.append(adj)

    return None 

# start 

print(" ────────────────────────────────────── ──────────────────────────────────────")
print(" ──────────────────────────────────────  Lab 5  |  Task 1: Mapping & Path Planning (maze8.xml)")
print(" ────────────────────────────────────── ──────────────────────────────────────")

robot.experiment_supervisor.step(timestep)

try:
    ox = robot.starting_position.x
    oy = robot.starting_position.y
except AttributeError:
    pos  = robot.experiment_supervisor.getSelf().getPosition()
    ox, oy = pos[0], pos[1]

origin_cell = coords_to_cell(ox, oy)

print(f"\n ──────────────────────────────────────  Starting cell  : {origin_cell}")
print(f" ──────────────────────────────────────  Target cell  : {TARGET_CELL}")

print("\n ──────────────────────────────────────  Computing route")
route = plan_route(origin_cell, TARGET_CELL)

if route is None:
    print("\n ────────────────────────────────────── No valid path found")
    while robot.experiment_supervisor.step(timestep) != -1:
        pass

else:
    print(f"\n ──────────────────────────────────────── Route:")
    print(f" ──────────────────────────────────────  Cells: {route}")
    print(f" ──────────────────────────────────────  Total Steps: {len(route) - 1}")
    print(f"─────────────────────────────────────────────────────")

    print("\n── ────────────────────────────────────── Navigation Log")
    for i in range(len(route) - 1):
        src, dst = route[i], route[i + 1]
        heading  = step_direction(src, dst)

        print(f" ──────────────────────────────────────  Move: Cell {src:2d} tp Cell {dst:2d}")

        if compass_direction() != heading:
            face_direction(heading)

        advance_one_cell()

    print(f"\n ──────────────────────────────────────")
    print(f" ──────────────────────────────────────  Target cell reached — {TARGET_CELL} ")
    print(f" ──────────────────────────────────────")

    halt()

    while robot.experiment_supervisor.step(timestep) != -1:
        pass
