# intelligent_robot_controller.py
# Version: Fuzzy behavior selection + anti-chattering + A* + improved obstacle avoidance & low-battery-priority charging
# Enhancement: Behavior tree structure + logging/counters + stuck replanning

from controller import Supervisor
import math
import heapq
import os

TIME_STEP = 64

# ===== Dynamic obstacle parameters =====
WALL_DEF_NAME = "MOVING_WALL"
WALL_AMPLITUDE = 1.2      # Left-right swing amplitude (m)
WALL_SPEED = 0.4          # Swing angular speed (rad/s)
WALL2_DEF_NAME = "MOVING_WALL2"
WALL2_AMPLITUDE = 0.2     # Swing amplitude of the second wall
WALL2_SPEED = 0.6         # Swing angular speed of the second wall
WALL3_DEF_NAME = "MOVING_WALL3"
WALL3_AMPLITUDE = 0.3     # Swing amplitude of the third wall
WALL3_SPEED = 0.7         # Swing angular speed of the third wall

START_X = -3.0
START_Y = -3.0

GOAL_POSITION = (1.0, 1.0)
CHARGING_STATION = (3.0, -3.0)

GOAL_REACHED_DIST = 0.15
WAYPOINT_REACHED_DIST = 0.12
CHARGING_REACHED_DIST = 0.15

MAP_MIN_X = -4.0
MAP_MAX_X = 4.0
MAP_MIN_Y = -4.0
MAP_MAX_Y = 4.0
GRID_RES = 0.1

GRID_WIDTH = int(round((MAP_MAX_X - MAP_MIN_X) / GRID_RES)) + 1
GRID_HEIGHT = int(round((MAP_MAX_Y - MAP_MIN_Y) / GRID_RES)) + 1


# ========= Behavior tree base nodes =========
class BTNode:
    def tick(self, context):
        raise NotImplementedError


class Selector(BTNode):
    """优先级选择节点：按顺序 tick 子节点，返回第一个非 None 的结果"""
    def __init__(self, children):
        self.children = children

    def tick(self, context):
        for child in self.children:
            res = child.tick(context)
            if res is not None:
                return res
        return None


class ConditionNode(BTNode):
    """条件节点：条件满足才执行子节点"""
    def __init__(self, condition_fn, child):
        self.condition_fn = condition_fn
        self.child = child

    def tick(self, context):
        if self.condition_fn(context):
            return self.child.tick(context)
        return None


class ActionNode(BTNode):
    """动作节点：直接返回一个行为名"""
    def __init__(self, behavior_name):
        self.behavior_name = behavior_name

    def tick(self, context):
        return self.behavior_name


class RobotState:
    def __init__(self):
        self.x = START_X
        self.y = START_Y
        self.theta = 0.0

        self.battery_level = 100.0

        self.distance_to_goal = 0.0
        self.heading_error = 0.0

        self.obstacle_danger = 0.0
        self.left_obstacle = 0.0
        self.right_obstacle = 0.0

        self.active_behavior = "GOAL"   # "GOAL" / "AVOID" / "CHARGE"
        self.last_behavior = "GOAL"
        self.last_switch_tick = 0

        self.path = []
        self.path_index = 0
        self.current_high_level_target = "GOAL"

        self.last_target_dist = None
        self.no_progress_ticks = 0

        self.mission_done = False

        # ===== Experimental metrics / logging related =====
        self.last_x = START_X
        self.last_y = START_Y
        self.distance_travelled = 0.0   # Total travelled distance

        self.behavior_switch_count = 0  # Number of behavior switches

        self.collision_count = 0        # Number of high-danger events

        self.stuck_replan_count = 0     # Number of stuck replanning times

        self.start_battery = 100.0
        self.end_battery = 100.0

        self.summary_printed = False    # Prevent printing SUMMARY multiple times


class IntelligentRobotController(Supervisor):
    def __init__(self):
        super().__init__()

        self.robot_node = self.getSelf()

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Sensors
        sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        self.distance_sensors = []
        for name in sensor_names:
            s = self.getDevice(name)
            if s is not None:
                s.enable(TIME_STEP)
                self.distance_sensors.append(s)

        self.state = RobotState()
        self.tick = 0

        self.behavior_cooldown = 20
        self.hysteresis_margin = 0.1

        # ===== Static map =====
        self.occupancy = self.build_static_occupancy()
        self.plan_path_to_high_level_target("GOAL")

        # ===== Dynamic obstacle MOVING_WALL =====
        self.wall_node = self.getFromDef(WALL_DEF_NAME)
        if self.wall_node:
            self.wall_translation_field = self.wall_node.getField("translation")
            self.wall_start_pos = self.wall_translation_field.getSFVec3f()
            print(f"[INFO] 动态墙初始位置: {self.wall_start_pos}")
        else:
            print("[WARN] 未找到 MOVING_WALL，请检查场景树 DEF 名字")
            self.wall_translation_field = None
            self.wall_start_pos = None

        self.sim_time = 0.0  # For computing wall motion

        # ===== Dynamic obstacle MOVING_WALL2 =====
        self.wall2_node = self.getFromDef(WALL2_DEF_NAME)
        if self.wall2_node is not None:
            self.wall2_translation_field = self.wall2_node.getField("translation")
            self.wall2_start_pos = self.wall2_translation_field.getSFVec3f()
            print(f"[INFO] MOVING_WALL2 found, start pos = {self.wall2_start_pos}")
        else:
            print("[WARN] MOVING_WALL2 not found in scene tree.")
            self.wall2_translation_field = None
            self.wall2_start_pos = None

        # ===== Dynamic obstacle MOVING_WALL3 =====
        self.wall3_node = self.getFromDef(WALL3_DEF_NAME)
        if self.wall3_node is not None:
            self.wall3_translation_field = self.wall3_node.getField("translation")
            self.wall3_start_pos = self.wall3_translation_field.getSFVec3f()
            print(f"[INFO] MOVING_WALL3 found, start pos = {self.wall3_start_pos}")
        else:
            print("[WARN] MOVING_WALL3 not found in scene tree.")
            self.wall3_translation_field = None
            self.wall3_start_pos = None

        # ===== Log file =====
        self.log_file = None
        self.init_log_file()

        # ===== Build behavior tree: CHARGE priority > AVOID > GOAL =====
        self.behavior_tree = Selector([
            ConditionNode(
                lambda ctx: ctx["mu_low_batt"] > 0.6,  # Low battery condition
                ActionNode("CHARGE")
            ),
            ConditionNode(
                lambda ctx: ctx["mu_obs"] > 0.5,       # Obstacle danger condition
                ActionNode("AVOID")
            ),
            ActionNode("GOAL")                        # Default behavior
        ])

    # ===== Logging related =====
    def init_log_file(self):
        fname = "run_log.csv"
        new_file = not os.path.exists(fname)
        self.log_file = open(fname, "a")
        if new_file:
            self.log_file.write(
                "tick,x,y,theta,behavior,batt,dist_goal,"
                "danger,left_obs,right_obs,dist_travelled,"
                "switch_count,collision_count,stuck_replan_count\n"
            )
            self.log_file.flush()

    def log_status(self):
        if self.log_file is None:
            return
        s = self.state
        self.log_file.write(
            f"{self.tick},{s.x:.4f},{s.y:.4f},{s.theta:.4f},"
            f"{s.active_behavior},{s.battery_level:.2f},{s.distance_to_goal:.4f},"
            f"{s.obstacle_danger:.3f},{s.left_obstacle:.3f},{s.right_obstacle:.3f},"
            f"{s.distance_travelled:.4f},{s.behavior_switch_count},"
            f"{s.collision_count},{s.stuck_replan_count}\n"
        )
        self.log_file.flush()

    # ===== Dynamic obstacle motion =====
    def update_dynamic_wall(self):
        self.sim_time += TIME_STEP / 1000.0  # Simulation time (seconds)
        t = self.sim_time

        # First wall: move left-right along X axis
        if self.wall_translation_field is not None and self.wall_start_pos is not None:
            x0, y0, z0 = self.wall_start_pos
            offset = WALL_AMPLITUDE * math.sin(WALL_SPEED * t)
            new_x = x0 + offset
            self.wall_translation_field.setSFVec3f([new_x, y0, z0])

        # Second wall: move forward-back along Z axis
        if self.wall2_translation_field is not None and self.wall2_start_pos is not None:
            x0, y0, z0 = self.wall2_start_pos
            offset2 = WALL2_AMPLITUDE * math.sin(WALL2_SPEED * t + math.pi / 2.0)
            new_z2 = z0 + offset2
            self.wall2_translation_field.setSFVec3f([x0, y0, new_z2])

        # Third wall: move left-right along X with phase shift
        if self.wall3_translation_field is not None and self.wall3_start_pos is not None:
            x0, y0, z0 = self.wall3_start_pos
            offset3 = WALL3_AMPLITUDE * math.sin(WALL3_SPEED * t + math.pi)
            new_x3 = x0 + offset3
            self.wall3_translation_field.setSFVec3f([new_x3, y0, z0])

    # ===== Grid conversion =====
    def world_to_grid(self, x: float, y: float):
        gx = int(round((x - MAP_MIN_X) / GRID_RES))
        gy = int(round((y - MAP_MIN_Y) / GRID_RES))
        gx = max(0, min(GRID_WIDTH - 1, gx))
        gy = max(0, min(GRID_HEIGHT - 1, gy))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int):
        x = MAP_MIN_X + gx * GRID_RES
        y = MAP_MIN_Y + gy * GRID_RES
        return x, y

    def build_static_occupancy(self):
        occ = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

        # The following are exactly your original obstacle mappings
        self.mark_obstacle_rect(occ, -0.644828, -0.344828, 0.55576, 1.65576)
        self.mark_obstacle_rect(occ, 0.822293, 1.122293, -0.832034, 0.267966)
        self.mark_obstacle_rect(occ, -0.261627, 0.038373, -0.257186, 0.842814)
        self.mark_obstacle_rect(occ, -0.659825, -0.359825, 0.61688, 1.71688)
        self.mark_obstacle_rect(occ, -2.8944, -2.5944, -2.88934, -1.78934)
        self.mark_obstacle_rect(occ, 0.759048, 1.059048, 0.83008, 1.93008)
        self.mark_obstacle_rect(occ, 0.116012, 0.416012, -1.80274, -0.70274)
        self.mark_obstacle_rect(occ, 1.78989, 2.08989, -3.19774, -2.09774)
        self.mark_obstacle_rect(occ, -1.93915, -1.63915, -1.64284, -0.54284)
        self.mark_obstacle_rect(occ, -1.011625, -0.711625, -2.58397, -1.48397)
        self.mark_obstacle_rect(occ, -3.36813, -3.06813, -0.315541, 0.784459)
        # New obstacle A
        self.mark_obstacle_rect(occ, -0.274335, 0.025665, -1.405721, -0.305721)
        # New obstacle B
        self.mark_obstacle_rect(occ, -2.02502, -1.72502, -2.85412, -1.75412)

        return occ

    def mark_obstacle_rect(self, occ, xmin, xmax, ymin, ymax):
        gx_min, gy_min = self.world_to_grid(xmin, ymin)
        gx_max, gy_max = self.world_to_grid(xmax, ymax)
        for gy in range(min(gy_min, gy_max), max(gy_min, gy_max) + 1):
            for gx in range(min(gx_min, gx_max), max(gx_min, gx_max) + 1):
                occ[gy][gx] = 1

    # ===== A* =====
    def astar(self, start_xy, goal_xy):
        sx, sy = start_xy
        gx, gy = goal_xy

        start_gx, start_gy = self.world_to_grid(sx, sy)
        goal_gx, goal_gy = self.world_to_grid(gx, gy)

        if self.occupancy[goal_gy][goal_gx] == 1:
            print(">>> [A*] Goal grid occupied, path may fail.")

        open_set = []
        heapq.heappush(open_set, (0.0, (start_gx, start_gy)))
        came_from = {}
        g_score = {(start_gx, start_gy): 0.0}

        def heuristic(ax, ay, bx, by):
            return abs(ax - bx) + abs(ay - by)

        closed = set()
        max_iter = GRID_WIDTH * GRID_HEIGHT * 4

        while open_set and max_iter > 0:
            max_iter -= 1
            _, (cx, cy) = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == goal_gx and cy == goal_gy:
                path = []
                cur = (cx, cy)
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append((start_gx, start_gy))
                path.reverse()
                wp = [self.grid_to_world(px, py) for (px, py) in path]
                return wp

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx = cx + dx
                    ny = cy + dy
                    if not (0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT):
                        continue
                    if self.occupancy[ny][nx] == 1:
                        continue

                    tentative_g = g_score[(cx, cy)] + math.hypot(dx, dy)
                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tentative_g
                        f = tentative_g + heuristic(nx, ny, goal_gx, goal_gy)
                        heapq.heappush(open_set, (f, (nx, ny)))
                        came_from[(nx, ny)] = (cx, cy)

        print(">>> [A*] Failed to find path.")
        return []

    def plan_path_to_high_level_target(self, target_type: str):
        if target_type == "CHARGE":
            target_xy = CHARGING_STATION
        else:
            target_xy = GOAL_POSITION

        start_xy = (self.state.x, self.state.y)
        path = self.astar(start_xy, target_xy)

        if not path:
            print(f">>> [A*] No path found to {target_type}, use direct point.")
            self.state.path = [target_xy]
            self.state.path_index = 0
        else:
            self.state.path = path
            self.state.path_index = 0
            print(f">>> [A*] Path to {target_type} planned with {len(path)} waypoints.")

        self.state.current_high_level_target = target_type
        self.state.last_target_dist = None
        self.state.no_progress_ticks = 0

    def get_current_path_target_point(self):
        if self.state.current_high_level_target == "CHARGE":
            fallback = CHARGING_STATION
        else:
            fallback = GOAL_POSITION

        if not self.state.path:
            return fallback
        if self.state.path_index >= len(self.state.path):
            return fallback
        return self.state.path[self.state.path_index]

    # ===== Pose update + distance accumulation + stuck replanning =====
    def update_pose(self):
        trans = self.robot_node.getField("translation").getSFVec3f()
        rot = self.robot_node.getField("rotation").getSFRotation()

        self.state.x = trans[0]
        self.state.y = trans[1]

        axis_x, axis_y, axis_z, angle = rot
        self.state.theta = angle

        # Accumulate travelled distance
        dx_travel = self.state.x - self.state.last_x
        dy_travel = self.state.y - self.state.last_y
        step_dist = math.hypot(dx_travel, dy_travel)
        self.state.distance_travelled += step_dist
        self.state.last_x = self.state.x
        self.state.last_y = self.state.y

        tx, ty = self.get_current_path_target_point()
        dx = tx - self.state.x
        dy = ty - self.state.y
        dist = math.hypot(dx, dy)
        self.state.distance_to_goal = dist

        target_theta = math.atan2(dy, dx)
        heading_err = target_theta - self.state.theta
        heading_err = (heading_err + math.pi) % (2.0 * math.pi) - math.pi
        self.state.heading_error = heading_err

        gx, gy = GOAL_POSITION
        dist_final_goal = math.hypot(self.state.x - gx, self.state.y - gy)
        if self.state.current_high_level_target != "CHARGE" and dist_final_goal < GOAL_REACHED_DIST:
            if not self.state.mission_done:
                print(">>> [TASK] Final goal reached, mission done.")
            self.state.mission_done = True
            self.state.end_battery = self.state.battery_level

        # Waypoint update
        if dist < WAYPOINT_REACHED_DIST and self.state.path:
            if self.state.path_index < len(self.state.path) - 1:
                self.state.path_index += 1

        # Target distance change, used for stuck detection
        if self.state.current_high_level_target == "CHARGE":
            tx2, ty2 = CHARGING_STATION
        else:
            tx2, ty2 = GOAL_POSITION
        dist_to_high_target = math.hypot(self.state.x - tx2, self.state.y - ty2)
        if self.state.last_target_dist is None:
            self.state.last_target_dist = dist_to_high_target
        else:
            if abs(dist_to_high_target - self.state.last_target_dist) < 0.005:
                self.state.no_progress_ticks += 1
            else:
                self.state.no_progress_ticks = 0
            self.state.last_target_dist = dist_to_high_target

        # Stuck replanning
        if self.state.no_progress_ticks > 200:
            print(">>> [REPLAN] Stuck detected, re-planning path.")
            self.plan_path_to_high_level_target(self.state.current_high_level_target)
            self.state.stuck_replan_count += 1
            self.state.no_progress_ticks = 0

        return self.state.current_high_level_target

    # ===== Sensors =====
    def update_sensors(self):
        if not self.distance_sensors:
            return

        values = [s.getValue() for s in self.distance_sensors]
        max_val = max(values)

        baseline = 60.0
        max_range = 80.0
        danger_raw = (max_val - baseline) / max_range
        danger = max(0.0, min(1.0, danger_raw))
        self.state.obstacle_danger = danger

        if len(values) == 8:
            left_sum = sum(values[0:4])
            right_sum = sum(values[4:8])
        else:
            mid = len(values) // 2
            left_sum = sum(values[:mid])
            right_sum = sum(values[mid:])

        norm = 400.0
        self.state.left_obstacle = min(1.0, left_sum / norm)
        self.state.right_obstacle = min(1.0, right_sum / norm)

        # High-danger event counting (can be seen as slight collision / wall rubbing)
        if danger > 0.95:
            if not self.state.in_collision:
                self.state.collision_count += 1
                self.state.in_collision = True
        else:
            self.state.in_collision = False

    # ===== Battery =====
    def update_battery(self, moving: bool):
        if self.state.mission_done:
            return

        if moving:
            self.state.battery_level -= 0.02
        else:
            self.state.battery_level -= 0.005

        if self.state.battery_level < 0.0:
            self.state.battery_level = 0.0
        if self.state.battery_level > 100.0:
            self.state.battery_level = 100.0

        if self.state.active_behavior == "CHARGE":
            dist_to_charge = math.hypot(
                self.state.x - CHARGING_STATION[0],
                self.state.y - CHARGING_STATION[1]
            )
            if dist_to_charge < CHARGING_REACHED_DIST:
                self.state.battery_level += 0.5
                if self.state.battery_level >= 100.0:
                    self.state.battery_level = 100.0
                    print(">>> [BAT] Fully charged at station.")

    # ===== Fuzzy membership =====
    def fuzzy_low_battery(self, batt: float) -> float:
        if batt >= 60.0:
            return 0.0
        if batt <= 20.0:
            return 1.0
        return (60.0 - batt) / 40.0

    def fuzzy_obstacle_near(self, danger: float) -> float:
        return max(0.0, min(1.0, danger))

    def fuzzy_far_from_goal(self, dist: float) -> float:
        if dist <= 0.1:
            return 0.0
        if dist >= 1.0:
            return 1.0
        return (dist - 0.1) / 0.9

    # ===== Behavior selection: behavior tree + fuzzy + anti-chattering =====
    def select_behavior(self):
        batt = self.state.battery_level
        danger = self.state.obstacle_danger
        dist = self.state.distance_to_goal

        mu_low_batt = self.fuzzy_low_battery(batt)
        mu_obs = self.fuzzy_obstacle_near(danger)
        mu_far = self.fuzzy_far_from_goal(dist)

        act_charge = mu_low_batt
        act_avoid = mu_obs
        act_goal = (1.0 - mu_low_batt) * max(0.0, 1.0 - mu_obs) * mu_far

        activations = {
            "CHARGE": act_charge,
            "AVOID": act_avoid,
            "GOAL": act_goal
        }

        # High-level decision by behavior tree
        bt_context = {
            "mu_low_batt": mu_low_batt,
            "mu_obs": mu_obs,
            "mu_far": mu_far,
            "batt": batt,
            "danger": danger,
            "dist": dist
        }
        desired_behavior = self.behavior_tree.tick(bt_context)
        if desired_behavior is None:
            desired_behavior = "GOAL"

        # Force low-battery priority
        if batt < 25.0 or mu_low_batt > 0.8:
            desired_behavior = "CHARGE"

        current = self.state.active_behavior
        current_act = activations[current]
        best_act = activations[desired_behavior]
        switched = False

        # Anti-chattering
        if (self.tick - self.state.last_switch_tick) < self.behavior_cooldown:
            if best_act > current_act + self.hysteresis_margin:
                self.state.last_behavior = current
                self.state.active_behavior = desired_behavior
                self.state.last_switch_tick = self.tick
                switched = True
        else:
            if current_act + self.hysteresis_margin < best_act or desired_behavior != current:
                self.state.last_behavior = current
                self.state.active_behavior = desired_behavior
                self.state.last_switch_tick = self.tick
                switched = True

        if switched:
            self.state.behavior_switch_count += 1
            if self.state.active_behavior == "CHARGE" and self.state.current_high_level_target != "CHARGE":
                self.plan_path_to_high_level_target("CHARGE")
            elif self.state.active_behavior == "GOAL" and self.state.current_high_level_target != "GOAL":
                self.plan_path_to_high_level_target("GOAL")

        return activations

    # ===== Control command (keep your original avoidance and tracking logic) =====
    def compute_command(self):
        if self.state.mission_done or self.state.battery_level <= 0.0:
            return 0.0, 0.0

        d = self.state.distance_to_goal
        e = self.state.heading_error
        danger = self.state.obstacle_danger
        left_o = self.state.left_obstacle
        right_o = self.state.right_obstacle

        max_speed = 6.28
        behavior = self.state.active_behavior

        if behavior == "CHARGE":
            dist_to_charge = math.hypot(
                self.state.x - CHARGING_STATION[0],
                self.state.y - CHARGING_STATION[1]
            )
            if dist_to_charge < CHARGING_REACHED_DIST:
                return 0.0, 0.0

        # High danger + both sides very close: regarded as “stuck to wall / in narrow gap”, first back straight
        if danger > 0.9 and left_o > 0.9 and right_o > 0.9:
            v = -2.0
            w = 0.0
        else:
            if behavior == "AVOID":
                if danger > 0.6:
                    turn_bias = 3.0 if left_o > right_o else -3.0
                    v = -2.0   # Slightly smaller to avoid excessive spin
                    w = turn_bias
                else:
                    v = 1.5
                    w = 4.0 * (right_o - left_o)
            elif behavior == "CHARGE":
                v = 3.0 * min(d, 1.0)
                w = 3.0 * e
            else:  # GOAL
                if danger > 0.5:
                    v = 2.0
                    w = 3.0 * (right_o - left_o)
                else:
                    v = 4.0 * min(d, 1.0)
                    w = 3.0 * e

        v_left = v - w
        v_right = v + w

        v_left = max(-max_speed, min(max_speed, v_left))
        v_right = max(-max_speed, min(max_speed, v_right))

        return v_left, v_right

    # ===== Main loop =====
    def run_controller(self):
        last_log_tick = -999

        while self.step(TIME_STEP) != -1:
            self.tick += 1

            # Update dynamic obstacles
            self.update_dynamic_wall()

            if self.state.battery_level <= 0.0:
                print(">>> [BAT] Battery empty, stop.")
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                continue

            if self.state.mission_done:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                if not self.state.summary_printed:
                    print(
                        f">>> [SUMMARY] dist_travelled={self.state.distance_travelled:.3f} m, "
                        f"behavior_switches={self.state.behavior_switch_count}, "
                        f"collisions={self.state.collision_count}, "
                        f"stuck_replans={self.state.stuck_replan_count}, "
                        f"energy_used={self.state.start_battery - self.state.end_battery:.2f}"
                    )
                    self.state.summary_printed = True
                continue

            target_name = self.update_pose()
            self.update_sensors()

            acts = self.select_behavior()

            v_left, v_right = self.compute_command()
            moving = (abs(v_left) > 0.05 or abs(v_right) > 0.05)

            self.update_battery(moving)

            self.left_motor.setVelocity(v_left)
            self.right_motor.setVelocity(v_right)

            if self.tick - last_log_tick >= 20:
                last_log_tick = self.tick
                print(
                    f"[t={self.tick}] pos=({self.state.x:.2f},{self.state.y:.2f}) "
                    f"theta={self.state.theta:.2f} "
                    f"HL_target={self.state.current_high_level_target} "
                    f"dist={self.state.distance_to_goal:.2f} "
                    f"batt={self.state.battery_level:.1f} "
                    f"danger={self.state.obstacle_danger:.2f} "
                    f"Lobs={self.state.left_obstacle:.2f} "
                    f"Robs={self.state.right_obstacle:.2f} "
                    f"behavior={self.state.active_behavior}"
                )
                self.log_status()


if __name__ == "__main__":
    controller = IntelligentRobotController()
    controller.run_controller()
