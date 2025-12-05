# intelligent_robot_controller.py
# 版本：模糊行为选择 + 抖动抑制 + A* + 改进避障 & 低电优先充电

from controller import Supervisor
import math
import heapq

TIME_STEP = 64

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

        # 卡死检测（可扩展用）
        self.last_target_dist = None
        self.no_progress_ticks = 0

        self.mission_done = False


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

        self.occupancy = self.build_static_occupancy()
        self.plan_path_to_high_level_target("GOAL")

    # ===== 栅格转换 =====
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
    
        # 统一的安全余量（可以根据效果调大/调小）
        # margin = 0.05  # 上面是计算用的，这里直接写死数值了
    
        # 每一条注释对应你发的一行：  x y z // size // rotation
    
        # 1) -0.494828 1.10576 0 // 0.2 1 2.4 // 0 0 1 -2.879...
        #   → 大概在上方的一块竖着的墙
        self.mark_obstacle_rect(
            occ,
            -0.644828, -0.344828,   # xmin, xmax
            0.55576, 1.65576        # ymin, ymax
        )
    
        # 2) 0.972293 -0.282034 0 // 0.2 1 2.4 // 0 0 1 -2.356...
        self.mark_obstacle_rect(
            occ,
            0.822293, 1.122293,
            -0.832034, 0.267966
        )
    
        # 3) -0.111627 0.292814 0 // 0.2 1 2.4 // 0 0 1 -1.5707...
        self.mark_obstacle_rect(
            occ,
            -0.261627, 0.038373,
            -0.257186, 0.842814
        )
    
        # 4) -0.509825 1.16688 0 // 0.2 1 2.4 // 0 0 1 -2.0943...
        self.mark_obstacle_rect(
            occ,
            -0.659825, -0.359825,
            0.61688, 1.71688
        )
    
        # 5) -2.7444 -2.33934 -5.3e-15 // 0.2 1 2.4 // 0 0 -1 1.309
        self.mark_obstacle_rect(
            occ,
            -2.8944, -2.5944,
            -2.88934, -1.78934
        )
    
        # 6) 0.909048 1.38008 -0.207658 // 0.2 1 2.4 // ... 1.54404
        self.mark_obstacle_rect(
            occ,
            0.759048, 1.059048,
            0.83008, 1.93008
        )
    
        # 7) 0.266012 -1.25274 -0.207658 // 0.2 1 2.4 // ... -2.84768
        self.mark_obstacle_rect(
            occ,
            0.116012, 0.416012,
            -1.80274, -0.70274
        )
    
        # 8) 1.93989 -2.64774 -0.207658 // 0.2 1 2.4 // ... -2.84768
        self.mark_obstacle_rect(
            occ,
            1.78989, 2.08989,
            -3.19774, -2.09774
        )
    
        # 9) -1.78915 -1.09284 0.02 // 0.2 1 2.4 // 0 0 -1 1.309
        self.mark_obstacle_rect(
            occ,
            -1.93915, -1.63915,
            -1.64284, -0.54284
        )
    
        # 10) -0.861625 -2.03397 -0.08 // 0.2 1 2.4 // 0 0 1 -2.356...
        self.mark_obstacle_rect(
            occ,
            -1.011625, -0.711625,
            -2.58397, -1.48397
        )
    
        # 11) -3.21813 0.234459 -0.16 // 0.2 1 2.4 // 0 0 1 -1.0471...
        self.mark_obstacle_rect(
            occ,
            -3.36813, -3.06813,
            -0.315541, 0.784459
        )
        # --- New obstacle A ---
        # -0.124335 -0.855721 -0.1305 // 0.2 1 2.4 // rotation...
        self.mark_obstacle_rect(
            occ,
            -0.274335, 0.025665,     # xmin, xmax
            -1.405721, -0.305721     # ymin, ymax
        )
    
        # --- New obstacle B ---
        # -1.87502 -2.30412 -0.1305 // 0.2 1 2.4 // rotation...
        self.mark_obstacle_rect(
            occ,
            -2.02502, -1.72502,      # xmin, xmax
            -2.85412, -1.75412       # ymin, ymax
        )
        
    
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

    # ===== 姿态更新 =====
    def update_pose(self):
        trans = self.robot_node.getField("translation").getSFVec3f()
        rot = self.robot_node.getField("rotation").getSFRotation()

        self.state.x = trans[0]
        self.state.y = trans[1]

        axis_x, axis_y, axis_z, angle = rot
        self.state.theta = angle

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

        # 路径点更新
        if dist < WAYPOINT_REACHED_DIST and self.state.path:
            if self.state.path_index < len(self.state.path) - 1:
                self.state.path_index += 1

        # 记录到高层目标的距离，用于卡死检测（可扩展用）
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

        return self.state.current_high_level_target

    # ===== 传感器 =====
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

    # ===== 电量 =====
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

    # ===== 模糊 membership =====
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

    # ===== 行为选择（改：低电优先 CHARGE） =====
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

        # 低电优先级：当电量 < 25% 或 低电模糊度 > 0.8 时，强制 CHARGE
        if batt < 25.0 or mu_low_batt > 0.8:
            if self.state.current_high_level_target != "CHARGE":
                self.plan_path_to_high_level_target("CHARGE")
            self.state.active_behavior = "CHARGE"
            return activations

        best_behavior = max(activations, key=activations.get)
        best_act = activations[best_behavior]

        current = self.state.active_behavior
        current_act = activations[current]
        switched = False

        if (self.tick - self.state.last_switch_tick) < self.behavior_cooldown:
            if best_act > current_act + self.hysteresis_margin:
                self.state.last_behavior = current
                self.state.active_behavior = best_behavior
                self.state.last_switch_tick = self.tick
                switched = True
        else:
            if current_act + self.hysteresis_margin < best_act:
                self.state.last_behavior = current
                self.state.active_behavior = best_behavior
                self.state.last_switch_tick = self.tick
                switched = True

        if switched:
            if self.state.active_behavior == "CHARGE" and self.state.current_high_level_target != "CHARGE":
                self.plan_path_to_high_level_target("CHARGE")
            elif self.state.active_behavior == "GOAL" and self.state.current_high_level_target != "GOAL":
                self.plan_path_to_high_level_target("GOAL")

        return activations

    # ===== 控制命令（改：高危险双侧障碍时直线后退） =====
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

        # 高危险 + 左右都很近：判定为“贴墙/夹缝”，先直线后退
        if danger > 0.9 and left_o > 0.9 and right_o > 0.9:
            v = -2.0
            w = 0.0
        else:
            if behavior == "AVOID":
                if danger > 0.6:
                    turn_bias = 3.0 if left_o > right_o else -3.0
                    v = -2.0   # 稍微小一点，避免过分 spin
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

    # ===== 主循环 =====
    def run_controller(self):
        last_log_tick = -999

        while self.step(TIME_STEP) != -1:
            self.tick += 1

            if self.state.battery_level <= 0.0:
                print(">>> [BAT] Battery empty, stop.")
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                continue

            if self.state.mission_done:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
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
                    f"behavior={self.state.active_behavior} "
                    f"acts={acts} "
                    f"wp_idx={self.state.path_index}/{len(self.state.path)} "
                    f"cmd=({v_left:.2f},{v_right:.2f})"
                )


if __name__ == "__main__":
    controller = IntelligentRobotController()
    controller.run_controller()
