from fairis_tools.my_robot import MyRobot
import math
import sys
import random
from collections import Counter

sys.path.insert(0, r'C:\Users\wishy\FAIRIS-Lite')
from fairis_tools.my_robot import MyRobot

# Setup
robot = MyRobot()
robot.load_environment(r'C:\Users\wishy\FAIRIS-Lite\WebotsSim\worlds\Spring26\maze8.xml')
robot.move_to_start()
timestep = robot.timestep

# ── Configuration ──────────────────────────────────────────────────────────────
NUM_PARTICLES   = 300
CONVERGE_PCT    = 0.80
WALL_DIST       = 0.6
MAX_STEPS       = 40
FWD_SPEED       = 4.0
TURN_SPEED      = 1.5

# ── Direction constants ────────────────────────────────────────────────────────
DIRS      = ['N', 'E', 'S', 'W']
DIR_IDX   = {d: i for i, d in enumerate(DIRS)}
DIR_DELTA = {'N': -5, 'E': 1, 'S': 5, 'W': -1}
DIR_HDG   = {'N': 90.0, 'E': 0.0, 'S': 270.0, 'W': 180.0}

# ── Sensor model p(obs | wall_state) ──────────────────────────────────────────
#    key: (observed, true)
SENSOR_MODEL = {
    (0, 0): 0.6,
    (1, 0): 0.4,
    (0, 1): 0.2,
    (1, 1): 0.8,
}

# ── Maze map: cell -> (N, E, S, W) walls ──────────────────────────────────────
MAZE = {
     1: (1, 0, 0, 1),  2: (1, 0, 0, 0),  3: (1, 0, 0, 0),
     4: (1, 0, 0, 0),  5: (1, 1, 0, 0),  6: (0, 0, 0, 1),
     7: (0, 0, 0, 0),  8: (0, 0, 0, 0),  9: (0, 0, 0, 0),
    10: (0, 1, 0, 0), 11: (0, 0, 0, 1), 12: (0, 0, 0, 0),
    13: (0, 0, 0, 0), 14: (0, 0, 0, 0), 15: (0, 1, 0, 0),
    16: (0, 0, 0, 1), 17: (0, 0, 0, 0), 18: (0, 0, 0, 0),
    19: (0, 0, 0, 0), 20: (0, 1, 0, 0), 21: (0, 0, 1, 1),
    22: (0, 0, 1, 0), 23: (0, 0, 1, 0), 24: (0, 0, 1, 0),
    25: (0, 1, 1, 0),
}

# ══════════════════════════════════════════════════════════════════════════════
# Cell utilities

def row_col(cell):
    return (cell - 1) // 5, (cell - 1) % 5

def can_move(cell, direction):
    r, c = row_col(cell)
    bounds = {'N': r > 0, 'S': r < 4, 'E': c < 4, 'W': c > 0}
    return bounds[direction] and MAZE[cell][DIR_IDX[direction]] == 0

def next_cell(cell, direction):
    if not can_move(cell, direction):
        return cell
    return cell + DIR_DELTA[direction]

# ══════════════════════════════════════════════════════════════════════════════
# ParticleSet class — wraps all filter operations


class ParticleSet:
    def __init__(self):
        self.particles = self._init()

    def _init(self):
        per_cell = NUM_PARTICLES // 25
        pts = [[c, random.choice(DIRS)]
               for c in range(1, 26)
               for _ in range(per_cell)]
        while len(pts) < NUM_PARTICLES:
            pts.append([random.randint(1, 25), random.choice(DIRS)])
        random.shuffle(pts)
        return pts

    def _likelihood(self, cell, obs):
        walls = MAZE[cell]
        score = 1.0
        for i in range(4):
            score *= SENSOR_MODEL.get((obs[i], walls[i]), 0.1)
        return score

    def _normalise(self, raw):
        total = sum(raw)
        if total == 0:
            return [1.0 / NUM_PARTICLES] * NUM_PARTICLES
        return [w / total for w in raw]

    def update_weights(self, obs):
        raw = [self._likelihood(p[0], obs) for p in self.particles]
        return self._normalise(raw)

    def resample(self, weights):
        step  = 1.0 / NUM_PARTICLES
        r     = random.uniform(0.0, step)
        cumul = weights[0]
        i     = 0
        new   = []
        for _ in range(NUM_PARTICLES):
            while r > cumul and i < NUM_PARTICLES - 1:
                i += 1
                cumul += weights[i]
            new.append([self.particles[i][0], random.choice(DIRS)])
            r += step
        self.particles = new

    def apply_motion(self, direction):
        for p in self.particles:
            p[0] = next_cell(p[0], direction)
            p[1] = direction

    def best_guess(self):
        counts = Counter(p[0] for p in self.particles)
        cell, n = counts.most_common(1)[0]
        return cell, n / NUM_PARTICLES

# ══════════════════════════════════════════════════════════════════════════════
# Display


def show_grid(pset, step):
    counts = Counter(p[0] for p in pset.particles)
    print("────────────────────────────────────────")
    print("        col1    col2    col3    col4    col5")
    for row in range(5):
        line = f"  row{row+1} "
        for col in range(5):
            line += f"  {counts.get(row*5+col+1, 0):3d}"
        print(line)
    cell, pct = pset.best_guess()
    print(f"\n  Mode cell  : {cell}   ({int(pct*NUM_PARTICLES)}/{NUM_PARTICLES} = {pct*100:.1f}%)")
    print(f"  Localised  : {'Yes' if pct >= CONVERGE_PCT else 'not yet'}")
    return cell, pct >= CONVERGE_PCT

# ══════════════════════════════════════════════════════════════════════════════
# Robot sensor helpers

def read_walls():
    lidar   = robot.get_lidar_range_image()
    heading = robot.get_compass_reading()
    n_rays  = len(lidar)
    compass = {'N': 90, 'E': 0, 'S': 270, 'W': 180}
    obs = []
    for d in DIRS:
        idx = int(round((heading + 180 - compass[d]) % 360)) % n_rays
        obs.append(1 if lidar[idx] < WALL_DIST else 0)
    return tuple(obs)

def heading_to_dir(h):
    h = h % 360
    if 45  <= h < 135: return 'N'
    if 135 <= h < 225: return 'W'
    if 225 <= h < 315: return 'S'
    return 'E'

# ══════════════════════════════════════════════════════════════════════════════
# Robot motion helpers


def halt():
    robot.set_left_motor_velocity(0)
    robot.set_right_motor_velocity(0)

def rotate_to(target_hdg):
    while robot.experiment_supervisor.step(timestep) != -1:
        hdg  = robot.get_compass_reading()
        diff = (target_hdg - hdg + 360.0) % 360.0
        if diff > 180.0:
            diff -= 360.0
        if abs(diff) < 2.0:
            break
        spd = TURN_SPEED if diff > 0 else -TURN_SPEED
        robot.set_left_motor_velocity(-spd)
        robot.set_right_motor_velocity(spd)
    halt()

def snap():
    hdg     = robot.get_compass_reading()
    snapped = round(hdg / 90) * 90 % 360
    rotate_to(float(snapped))

def forward_one():
    pos0 = robot.experiment_supervisor.getSelf().getPosition()
    robot.set_left_motor_velocity(FWD_SPEED)
    robot.set_right_motor_velocity(FWD_SPEED)
    while robot.experiment_supervisor.step(timestep) != -1:
        pos = robot.experiment_supervisor.getSelf().getPosition()
        d   = math.sqrt((pos[0]-pos0[0])**2 + (pos[2]-pos0[2])**2)
        lidar = robot.get_lidar_range_image()
        if d >= 0.93 or lidar[180] < 0.25:
            break
    halt()

def front_open():
    robot.experiment_supervisor.step(timestep)
    lidar = robot.get_lidar_range_image()
    return min(lidar[180], lidar[170], lidar[190]) > 0.8

def pick_and_move():
    snap()
    candidates = [0, 90, 180, 270]
    random.shuffle(candidates)
    for target in candidates:
        rotate_to(float(target))
        snap()
        for _ in range(3):
            robot.experiment_supervisor.step(timestep)
        if front_open():
            d = heading_to_dir(robot.get_compass_reading())
            print(f"────────────── Moving {d} (heading {robot.get_compass_reading():.1f}°)")
            forward_one()
            snap()
            return d
        print(f"────────────── Blocked {target}°, next direction")
    print("────────────── All 4 Directions blocked")
    return None

# ══════════════════════════════════════════════════════════════════════════════
# Main loop


def run():
    partm      = ParticleSet()
    step      = 0
    localised = False

    while not localised and step < MAX_STEPS:
        if robot.experiment_supervisor.step(timestep) == -1:
            break

        obs = read_walls()
        print(f"\n────────────── Observation (N,E,S,W): {obs}")

        w = partm.update_weights(obs)
        partm.resample(w)

        best_cell, localised = show_grid(partm, step + 1)

        if not localised:
            moved = pick_and_move()
            if moved:
                partm.apply_motion(moved)

        step += 1

    print(f"\n{'='*42}")
    if localised:
        print(f"────────────── Localized at cell {pset.best_guess()}")
    else:
        print(f"────────────── Best guess: cell {pset.best_guess()}")
    print(f"────────────── {'='*42}\n")

if __name__ == "__main__":
    run()

