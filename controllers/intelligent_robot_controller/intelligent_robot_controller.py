# intelligent_robot_controller.py
# 版本：模糊行为选择 + 抖动抑制 + 分离的 A* + 改进避障 & 低电优先充电

from controller import Supervisor
import math
import heapq

TIME_STEP = 64

# ===== 动态障碍物参数 =====
WALL_DEF_NAME = "MOVING_WALL"
WALL_AMPLITUDE = 1.2      # 左右摆动幅度 (米)
WALL_SPEED = 0.4          # 摆动速度 (rad/s)
WALL2_DEF_NAME = "MOVING_WALL2"
WALL2_AMPLITUDE = 0.2     # 第二堵墙的摆动幅度
WALL2_SPEED = 0.6         # 第二堵墙的摆动速度
WALL3_DEF_NAME = "MOVING_WALL3"
WALL3_AMPLITUDE = 0.3     # 第三堵墙的摆动幅度
WALL3_SPEED = 0.7         # 第三堵墙的摆动速度

# ===== 地图参数 =====
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


# ========= 全局栅格坐标转换函数（A* 和占据栅格复用） =========
def world_to_grid(x: float, y: float):
    gx = int(round((x - MAP_MIN_X) / GRID_RES))
    gy = int(round((y - MAP_MIN_Y) / GRID_RES))
    gx = max(0, min(GRID_WIDTH - 1, gx))
    gy = max(0, min(GRID_HEIGHT - 1, gy))
    return gx, gy


def grid_to_world(gx: int, gy: int):
    x = MAP_MIN_X + gx * GRID_RES
    y = MAP_MIN_Y + gy * GRID_RES
    return x, y


# ========= A* 路径规划器（已分离） =========
class GridAStarPlanner:
    """
    简单的栅格 A*：
    - 使用全局 MAP_MIN/MAX 和 GRID_RES；
    - 起点 / 终点落在障碍格子时，会在附近搜索最近的空格子；
    - 8 邻域（含对角线），对角线代价为 sqrt(2)。
    """

    def __init__(self, occupancy):
        """
        occupancy: 2D list[gy][gx]，0=空，1=障碍
        """
        self.occupancy = occupancy
        self.height = len(occupancy)
        self.width = len(occupancy[0]) if self.height > 0 else 0

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _is_free(self, gx, gy):
        return self._in_bounds(gx, gy) and self.occupancy[gy][gx] == 0

    def _find_nearest_free(self, gx, gy, max_radius=6):
        """
        如果 (gx, gy) 是障碍，就在 max_radius 范围内找最近的空格。
        找不到就返回 None。
        """
        if self._is_free(gx, gy):
            return gx, gy

        best = None
        best_dist2 = None

        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if not self._is_free(nx, ny):
                        continue
                    d2 = dx * dx + dy * dy
                    if best is None or d2 < best_dist2:
                        best = (nx, ny)
                        best_dist2 = d2
            if best is not None:
                break

        return best

    def plan(self, start_xy, goal_xy):
        sx, sy = start_xy
        gx, gy = goal_xy

        start_gx, start_gy = world_to_grid(sx, sy)
        goal_gx, goal_gy = world_to_grid(gx, gy)

        # 起点 / 终点若落在障碍上，试图挪到最近空格
        s_free = self._find_nearest_free(start_gx, start_gy, max_radius=3)
        g_free = self._find_nearest_free(goal_gx, goal_gy, max_radius=6)

        if s_free is None:
            print(">>> [A*] Start region fully blocked, no path.")
            return []

        if g_free is None:
            print(">>> [A*] Goal region fully blocked, no path.")
            return []

        start_gx, start_gy = s_free
        goal_gx, goal_gy = g_free

        if (start_gx, start_gy) != world_to_grid(sx, sy):
            sx2, sy2 = grid_to_world(start_gx, start_gy)
            print(f">>> [A*] Adjusted start to nearest free cell: ({sx2:.2f}, {sy2:.2f})")

        if (goal_gx, goal_gy) != world_to_grid(gx, gy):
            gx2, gy2 = grid_to_world(goal_gx, goal_gy)
            print(f">>> [A*] Adjusted goal to nearest free cell: ({gx2:.2f}, {gy2:.2f})")

        # A* 主体
        open_set = []
        heapq.heappush(open_set, (0.0, (start_gx, start_gy)))
        came_from = {}
        g_score = {(start_gx, start_gy): 0.0}

        def heuristic(ax, ay, bx, by):
            # 用欧氏距离的轻微放大，减少 tie
            return math.hypot(ax - bx, ay - by) * 1.001

        closed = set()
        max_iter = self.width * self.height * 4

        # 8 邻域（dx, dy, cost）
        neighbors = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0)),
        ]

        while open_set and max_iter > 0:
            max_iter -= 1
            _, (cx, cy) = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == goal_gx and cy == goal_gy:
                # 回溯路径
                path_cells = []
                cur = (cx, cy)
                while cur in came_from:
                    path_cells.append(cur)
                    cur = came_from[cur]
                path_cells.append((start_gx, start_gy))
                path_cells.reverse()
                # 转回世界坐标
                waypoints = [grid_to_world(px, py) for (px, py) in path_cells]
                return waypoints

            for dx, dy, step_cost in neighbors:
                nx = cx + dx
                ny = cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                if self.occupancy[ny][nx] == 1:
                    continue

                tentative_g = g_score[(cx, cy)] + step_cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + heuristic(nx, ny, goal_gx, goal_gy)
                    heapq.heappush(open_set, (f, (nx, ny)))
                    came_from[(nx, ny)] = (cx, cy)

        print(">>> [A*] Failed to find path.")
        return []


# ========= 机器人状态 =========
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

        # 传感器
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

        # ===== 静态地图 + A* 路径规划器 =====
        self.occupancy = self.build_static_occupancy()
        self.planner = GridAStarPlanner(self.occupancy)
        self.plan_path_to_high_level_target("GOAL")

        # ===== 动态障碍物 MOVING_WALL =====
        self.wall_node = self.getFromDef(WALL_DEF_NAME)
        if self.wall_node:
            self.wall_translation_field = self.wall_node.getField("translation")
            self.wall_start_pos = self.wall_translation_field.getSFVec3f()
            print(f"[INFO] MOVING_WALL start pos: {self.wall_start_pos}")
        else:
            print("[WARN] MOVING_WALL not found.")
            self.wall_translation_field = None
            self.wall_start_pos = None

        # MOVING_WALL2
        self.wall2_node = self.getFromDef(WALL2_DEF_NAME)
        if self.wall2_node is not None:
            self.wall2_translation_field = self.wall2_node.getField("translation")
            self.wall2_start_pos = self.wall2_translation_field.getSFVec3f()
            print(f"[INFO] MOVING_WALL2 start pos: {self.wall2_start_pos}")
        else:
            print("[WARN] MOVING_WALL2 not found.")
            self.wall2_translation_field = None
            self.wall2_start_pos = None

        # MOVING_WALL3
        self.wall3_node = self.getFromDef(WALL3_DEF_NAME)
        if self.wall3_node is not None:
            self.wall3_translation_field = self.wall3_node.getField("translation")
            self.wall3_start_pos = self.wall3_translation_field.getSFVec3f()
            print(f"[INFO] MOVING_WALL3 start pos: {self.wall3_start_pos}")
        else:
            print("[WARN] MOVING_WALL3 not found.")
            self.wall3_translation_field = None
            self.wall3_start_pos = None

        self.sim_time = 0.0  # 用于动态墙运动

    # ========== 动态障碍物移动 ==========
    def update_dynamic_wall(self):
        self.sim_time += TIME_STEP / 1000.0
        t = self.sim_time

        # MOVING_WALL：沿 X 轴
        if self.wall_translation_field is not None and self.wall_start_pos is not None:
            x0, y0, z0 = self.wall_start_pos
            offset = WALL_AMPLITUDE * math.sin(WALL_SPEED * t)
            new_x = x0 + offset
            self.wall_translation_field.setSFVec3f([new_x, y0, z0])

        # MOVING_WALL2：沿 Z 轴
        if self.wall2_translation_field is not None and self.wall2_start_pos is not None:
            x0, y0, z0 = self.wall2_start_pos
            offset2 = WALL2_AMPLITUDE * math.sin(WALL2_SPEED * t + math.pi / 2.0)
            new_z2 = z0 + offset2
            self.wall2_translation_field.setSFVec3f([x0, y0, new_z2])

        # MOVING_WALL3：沿 X 轴，带相位差
        if self.wall3_translation_field is not None and self.wall3_start_pos is not None:
            x0, y0, z0 = self.wall3_start_pos
            offset3 = WALL3_AMPLITUDE * math.sin(WALL3_SPEED * t + math.pi)
            new_x3 = x0 + offset3
            self.wall3_translation_field.setSFVec3f([new_x3, y0, z0])

    # ===== 占据栅格构建 =====
    def build_static_occupancy(self):
        occ = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

        # 下面每一个 mark_obstacle_rect 对应一个静态障碍物
        # 这些数值是你之前发给我的障碍坐标 & 大致范围

        # 1) -0.494828 1.10576 0
        self.mark_obstacle_rect(
            occ,
            -0.644828, -0.344828,
            0.55576, 1.65576
        )

        # 2) 0.972293 -0.282034 0
        self.mark_obstacle_rect(
            occ,
            0.822293, 1.122293,
            -0.832034, 0.267966
        )

        # 3) -0.111627 0.292814 0
        self.mark_obstacle_rect(
            occ,
            -0.261627, 0.038373,
            -0.257186, 0.842814
        )

        # 4) -0.509825 1.16688 0
        self.mark_obstacle_rect(
            occ,
            -0.659825, -0.359825,
            0.61688, 1.71688
        )

        # 5) -2.7444 -2.33934
        self.mark_obstacle_rect(
            occ,
            -2.8944, -2.5944,
            -2.88934, -1.78934
        )

        # 6) 0.909048 1.38008
        self.mark_obstacle_rect(
            occ,
            0.759048, 1.059048,
            0.83008, 1.93008
        )

        # 7) 0.266012 -1.25274
        self.mark_obstacle_rect(
            occ,
            0.116012, 0.416012,
            -1.80274, -0.70274
        )

        # 8) 1.93989 -2.64774
        self.mark_obstacle_rect(
            occ,
            1.78989, 2.08989,
            -3.19774, -2.09774
        )

        # 9) -1.78915 -1.09284
        self.mark_obstacle_rect(
            occ,
            -1.93915, -1.63915,
            -1.64284, -0.54284
        )

        # 10) -0.861625 -2.03397
        self.mark_obstacle_rect(
            occ,
            -1.011625, -0.711625,
            -2.58397, -1.48397
        )

        # 11) -3.21813 0.234459
        self.mark_obstacle_rect(
            occ,
            -3.36813, -3.06813,
            -0.315541, 0.784459
        )

        # New obstacle A
        self.mark_obstacle_rect(
            occ,
            -0.274335, 0.025665,
            -1.405721, -0.305721
        )

        # New obstacle B
        self.mark_obstacle_rect(
            occ,
            -2.02502, -1.72502,
            -2.85412, -1.75412
        )

        return occ

    def mark_obstacle_rect(self, occ, xmin, xmax, ymin, ymax):
        gx_min, gy_min = world_to_grid(xmin, ymin)
        gx_max, gy_max = world_to_grid(xmax, ymax)
        for gy in range(min(gy_min, gy_max), max(gy_min, gy_max) + 1):
            for gx in range(min(gx_min, gx_max), max(gx_min, gx_max) + 1):
                occ[gy][gx] = 1

    # ===== 使用分离的 A* 进行路径规划 =====
    def plan_path_to_high_level_target(self, target_type: str):
        if target_type == "CHARGE":
            target_xy = CHARGING_STATION
        else:
            target_xy = GOAL_POSITION

        start_xy = (self.state.x, self.state.y)
        path = self.planner.plan(start_xy, target_xy)

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
       
        self._read_pose_from_simulation()
        self._update_goal_distance_and_heading()
        self._check_final_goal_reached()
        self._update_waypoint_progress()
        self._update_no_progress_monitor()

        return self.state.current_high_level_target

    def _read_pose_from_simulation(self):
       
        trans = self.robot_node.getField("translation").getSFVec3f()
        rot = self.robot_node.getField("rotation").getSFRotation()

        self.state.x = trans[0]
        self.state.y = trans[1]
       
        self.state.theta = rot[3]

    def _update_goal_distance_and_heading(self):
        
        tx, ty = self.get_current_path_target_point()
        dx = tx - self.state.x
        dy = ty - self.state.y

        dist = math.hypot(dx, dy)
        self.state.distance_to_goal = dist

        target_theta = math.atan2(dy, dx)
        heading_err = target_theta - self.state.theta
        
        heading_err = (heading_err + math.pi) % (2.0 * math.pi) - math.pi
        self.state.heading_error = heading_err

    def _check_final_goal_reached(self):
        
        gx, gy = GOAL_POSITION
        dist_final_goal = math.hypot(self.state.x - gx, self.state.y - gy)

        if (
            self.state.current_high_level_target != "CHARGE"
            and dist_final_goal < GOAL_REACHED_DIST
        ):
            if not self.state.mission_done:
                print(">>> [TASK] Final goal reached, mission done.")
            self.state.mission_done = True

    def _update_waypoint_progress(self):
        
        if not self.state.path:
            return

        if self.state.distance_to_goal < WAYPOINT_REACHED_DIST:
            if self.state.path_index < len(self.state.path) - 1:
                self.state.path_index += 1

    def _update_no_progress_monitor(self):
        
        if self.state.current_high_level_target == "CHARGE":
            tx2, ty2 = CHARGING_STATION
        else:
            tx2, ty2 = GOAL_POSITION

        dist_to_high_target = math.hypot(self.state.x - tx2, self.state.y - ty2)

        if self.state.last_target_dist is None:
            self.state.last_target_dist = dist_to_high_target
            return

        if abs(dist_to_high_target - self.state.last_target_dist) < 0.005:
            self.state.no_progress_ticks += 1
        else:
            self.state.no_progress_ticks = 0

        self.state.last_target_dist = dist_to_high_target


    # ===== 传感器 =====
    def update_sensors(self):
        if not self.distance_sensors:
            return

        values = [s.getValue() for s in self.distance_sensors]
        # 单独方法：归一化到 0–1
        def _normalize(val, baseline=60.0, rng=80.0):
            return max(0.0, min(1.0, (val - baseline) / rng))

        # 危险度（最大值决定）
        self.state.obstacle_danger = _normalize(max(values))

        # 左右分组自动处理
        mid = len(values) // 2
        left_sum = sum(values[:mid])
        right_sum = sum(values[mid:])

        # 使用统一归一化方法（保持原始行为：400 为经验值）
        def _normalize_group(sum_val, group_norm=400.0):
            return max(0.0, min(1.0, sum_val / group_norm))

        self.state.left_obstacle = _normalize_group(left_sum)
        self.state.right_obstacle = _normalize_group(right_sum)

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
    def _linear_membership(self, value, start, end, invert=False):
        """
        通用模糊线性隶属度计算：
        - invert=False: start→0, end→1
        - invert=True:  start→1, end→0
        """
        if start == end:
            return 0.0
        t = (value - start) / (end - start)
        t = max(0.0, min(1.0, t))
        return 1.0 - t if invert else t

    def fuzzy_low_battery(self, batt: float) -> float:
        # 原逻辑：60→0, 20→1
        return self._linear_membership(batt, 60.0, 20.0, invert=True)

    def fuzzy_obstacle_near(self, danger: float) -> float:
        return max(0.0, min(1.0, danger))

    def fuzzy_far_from_goal(self, dist: float) -> float:
        # 原逻辑：0.1→0, 1.0→1
        return self._linear_membership(dist, 0.1, 1.0)
    
    # ===== 行为选择（低电优先 CHARGE） =====
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

    # ===== 底层控制命令 =====
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
                    v = -2.0
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

            # 更新动态障碍物
            self.update_dynamic_wall()

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
                print(f"[t={self.tick}] pos=({self.state.x:.2f},{self.state.y:.2f}) "
                      f"theta={self.state.theta:.2f} "
                      f"HL_target={self.state.current_high_level_target} "
                      f"dist={self.state.distance_to_goal:.2f} "
                      f"batt={self.state.battery_level:.1f} "
                      f"danger={self.state.obstacle_danger:.2f} "
                      f"Lobs={self.state.left_obstacle:.2f} "
                      f"Robs={self.state.right_obstacle:.2f} "
                      f"behavior={self.state.active_behavior}")


if __name__ == "__main__":
    controller = IntelligentRobotController()
    controller.run_controller()
