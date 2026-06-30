from fairis_tools.my_robot import MyRobot
import math
import sys
import numpy as np

sys.path.insert(0, r'C:\Users\wishy\FAIRIS-Lite')
from fairis_tools.my_robot import MyRobot

robot = MyRobot()
robot.load_environment(r'C:\Users\wishy\FAIRIS-Lite\WebotsSim\worlds\Spring26\maze7.xml')
robot.move_to_start()
timestep = robot.timestep


#Configuration
SPIN_SPEED    = 5.0
MIN_LANDMARKS = 3

#Known landmark positions (x, y) 
LANDMARK_POS = {
    'yellow': (-2.5,  2.5),
    'red':    ( 2.5,  2.5),
    'green':  (-2.5, -2.5),
    'blue':   ( 2.5, -2.5),
}

# ══════════════════════════════════════════════════════════════════════════════
# Helpers

def classify_color(r, g, b):
    if r > 0.5 and g > 0.5 and b < 0.3: return 'yellow'
    if r > 0.5 and g < 0.3 and b < 0.3: return 'red'
    if r < 0.3 and g > 0.5 and b < 0.3: return 'green'
    if r < 0.3 and g < 0.3 and b > 0.5: return 'blue'
    return None

def solve_position(distances):
    pairs        = list(distances.items())
    ref, d_ref   = pairs[0]
    x_ref, y_ref = LANDMARK_POS[ref]
    A_rows, b_vals = [], []
    for name, d_i in pairs[1:]:
        xi, yi = LANDMARK_POS[name]
        A_rows.append([2.0 * (xi - x_ref), 2.0 * (yi - y_ref)])
        b_vals.append(d_ref**2 - d_i**2 + xi**2 + yi**2 - x_ref**2 - y_ref**2)
    A   = np.array(A_rows, dtype=float)
    b   = np.array(b_vals, dtype=float)
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[0]), float(sol[1])


def scan_frame(found):
    for obj in robot.camera.getRecognitionObjects():
        try:
            r, g, b = obj.getColors()[0], obj.getColors()[1], obj.getColors()[2]
        except Exception:
            continue
        name = classify_color(r, g, b)
        if name and name not in found:
            pos  = obj.getPosition()
            dist = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            found[name] = dist
            print(f"────────────────────────────────────────{name} distance = {dist:.3f} m")

def xy_coord_position_to_cell(x, y):
    col = max(0, min(4, int(round(x)) + 2))
    row = max(0, min(4, 2 - int(round(y))))
    return row * 5 + col + 1

# ══════════════════════════════════════════════════════════════════════════════
# Main

def trilateration_run():
    print("────────────────────────────────────────Scanning for landmarks\n")

    found     = {}
    total_rot = 0.0
    prev_hdg  = None

    robot.set_left_motor_velocity(-SPIN_SPEED)
    robot.set_right_motor_velocity(SPIN_SPEED)

    while robot.experiment_supervisor.step(timestep) != -1:
        heading = robot.get_compass_reading()

        if prev_hdg is not None:
            delta = (heading - prev_hdg + 360.0) % 360.0
            if delta > 180.0:
                delta -= 360.0
            total_rot += abs(delta)
        prev_hdg = heading

        scan_frame(found)

        if total_rot >= 355.0 or len(found) >= 4:
            robot.set_left_motor_velocity(0)
            robot.set_right_motor_velocity(0)
            if len(found) < MIN_LANDMARKS:
                print("\n  [ERROR] Need at least 3 landmarks.")
            else:
                x_est, y_est = solve_position(found)
                cell_idx     = xy_coord_position_to_cell(x_est, y_est)
                print("────────────────────────────────────────Result")
                print(f"────────────────────────────────────────Estimated position : ({x_est:.4f},  {y_est:.4f})")
                print(f"────────────────────────────────────────Estimated cell : {cell_idx}")
                print("─────────────────────────────────────────────────────────────────────────────────────────────")
            break

if __name__ == "__main__":
    trilateration_run()
