import tkinter as tk
import threading
import sys
import time
import enum

CANVAS_WIDTH = 800
CANVAS_HEIGHT = 800
CANVAS_SHIFT = 16
SCALING_FACTOR = 10
ENABLE_LINE_TO_TARGET = True

# Limit on y.
MAP_WIDTH = 0
# Limit on x.
MAP_HEIGHT = 0

FRAMES_PER_STEP = float(5)
ROBOT_RADIUS = 4.5
DEBUG_MAP = False


def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x - r, y - r, x + r, y + r, **kwargs)


def _move_circle(self, elem, x, y, r):
    return self.coords(elem, x - r, y - r, x + r, y + r)


tk.Canvas.create_circle = _create_circle
tk.Canvas.move_circle = _move_circle


# Update the coordinates to fit with python canvas.
def transfer_coordinates(x, y):
    return y * SCALING_FACTOR + CANVAS_SHIFT, x * SCALING_FACTOR + CANVAS_SHIFT


class MoveType(enum.Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    WAIT = 4


class Robot(object):
    id = 0
    x = 0
    y = 0
    # Immediate target.
    tx = 0
    ty = 0
    # Final target.
    target_x = 0
    target_y = 0
    # diff per step.
    diff_x = 0
    diff_y = 0

    canvas = None
    elem = None
    line_to_target = None

    def __init__(self, canvas, id, x, y, target_x, target_y):
        self.canvas = canvas
        self.id = id
        self.x = x
        self.y = y
        self.tx = self.x
        self.ty = self.y
        self.target_x = target_x
        self.target_y = target_y
        self.elem = self.canvas.create_circle(
            *(transfer_coordinates(x, y)), ROBOT_RADIUS, fill="blue")
        self.line_to_target = self.canvas.create_line(
            *(transfer_coordinates(self.x, self.y)),
            *(transfer_coordinates(self.target_x, self.target_y)), fill="red")

    def move(self, type):
        if type == MoveType.UP:
            self.tx -= 1
            self.diff_x = -1 / FRAMES_PER_STEP
            self.diff_y = 0
        elif type == MoveType.DOWN:
            self.tx += 1
            self.diff_x = 1 / FRAMES_PER_STEP
            self.diff_y = 0
        elif type == MoveType.LEFT:
            self.ty -= 1
            self.diff_x = 0
            self.diff_y = -1 / FRAMES_PER_STEP
        elif type == MoveType.RIGHT:
            self.ty += 1
            self.diff_x = 0
            self.diff_y = 1 / FRAMES_PER_STEP
        elif type == MoveType.WAIT:
            self.diff_x = 0
            self.diff_y = 0
            pass
        else:
            sys.exit("Invalid type.")

    def draw(self):
        self.x += self.diff_x
        self.y += self.diff_y
        self.canvas.move_circle(self.elem,
                                *(transfer_coordinates(self.x, self.y)), ROBOT_RADIUS)
        self.canvas.coords(self.line_to_target,
                           *(transfer_coordinates(self.x, self.y)),
                           *(transfer_coordinates(self.target_x, self.target_y)))
        return

    def final_draw(self):
        self.canvas.move_circle(self.elem,
                                *(transfer_coordinates(self.tx, self.ty)), ROBOT_RADIUS)
        self.x = self.tx
        self.y = self.ty
        return


class RobotManager(object):
    canvas = None
    robots = {}

    def __init__(self, canvas, robot_tasks):
        self.canvas = canvas

        robot_id = 0
        for line in robot_tasks:
            elems = line.split()
            x = int(elems[0])
            y = int(elems[1])
            target_x = int(elems[2])
            target_y = int(elems[3])
            self.robots[robot_id] = Robot(canvas, robot_id, x, y, target_x, target_y)
            robot_id += 1

    def run(self, step):
        return


def draw_special_point(canvas, x, y):
    canvas.create_rectangle(*(transfer_coordinates(x - 0.2, y - 0.2)),
                            *(transfer_coordinates(x + 0.2, y + 0.2)), fill="red")


def draw_map(canvas, map_file_path):
    fp = open(map_file_path, "r")

    # Skip a dummy line.
    fp.readline()

    tmp_line = fp.readline()
    MAP_HEIGHT = int(tmp_line.split()[1])

    tmp_line = fp.readline()
    MAP_WIDTH = int(tmp_line.split()[1])

    # Skip a dummy line.
    fp.readline()

    list_of_lines = []
    for i in range(0, MAP_HEIGHT):
        list_of_lines.append(fp.readline())

    fp.close()

    for i in range(0, MAP_HEIGHT):
        for j in range(0, MAP_WIDTH):
            if list_of_lines[i][j] == '.':
                x = i
                y = j
                canvas.create_rectangle(*(transfer_coordinates(x, y)),
                                        *(transfer_coordinates(x, y)), fill="black")

    # draw_special_point(canvas, 7, 8)
    # draw_special_point(canvas, 18, 25)
    return


def run(canvas):
    schedule_file = open(sys.argv[1], "r")
    map_file_path = schedule_file.readline().strip()
    draw_map(canvas, map_file_path)

    if DEBUG_MAP:
        return

    robot_count = int(schedule_file.readline().strip())
    robot_tasks = []
    tmp_robot_count = robot_count
    while tmp_robot_count > 0:
        tmp_line = schedule_file.readline()
        if tmp_line.startswith('#'):
            continue
        robot_tasks.append(tmp_line)
        tmp_robot_count -= 1

    rm = RobotManager(canvas, robot_tasks)

    for line in schedule_file:
        elems = line.split()
        step_counter = int(elems[0])
        for i in range(1, len(elems), 2):
            robot_id = int(elems[i])
            if robot_id >= robot_count:
                break
            move_type = MoveType(int(elems[i + 1]))
            rm.robots[robot_id].move(move_type)

        # Process this one line.
        for iteration in range(int(FRAMES_PER_STEP)):
            for k, v in rm.robots.items():
                v.draw()
            time.sleep(1 / FRAMES_PER_STEP)

        for k, v in rm.robots.items():
            v.final_draw()
    schedule_file.close()


def main():
    root = tk.Tk()

    canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
    canvas.pack()

    t = threading.Thread(name='run', target=run, args=(canvas,))
    t.start()

    root.mainloop()


if __name__ == '__main__':
    main()
