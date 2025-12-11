# IR project work

## Group27

# Intelligent Robot Controller (Webots)

## 1. What this project does

We built a smarter navigation controller for a mobile robot in Webots.
The main goals:

* Make **behaviour switching more stable** (no crazy flickering between GOAL / AVOID / CHARGE)
* Improve **obstacle avoidance** in cluttered maps
* Handle **low battery** with a proper CHARGE behaviour
* Collect logs so we can **analyse runs afterwards**

The robot combines:

* A **behaviour tree** (CHARGE > AVOID > GOAL)
* **Fuzzy logic** to compute how “relevant” each behaviour is
* A **cooldown + hysteresis** system to stop rapid switching
* **A*** global planning on a 2D occupancy grid
* A **VFH-style local planner** using left/right sensor sums
* A simple **battery model** + charging at a station
* **Stuck detection** and automatic re-planning

---

## 2. What we wrote ourselves

All the main logic is implemented by us in Python, including:

* Behaviour tree nodes (`Selector`, `ConditionNode`, `ActionNode`)
* Fuzzy membership functions (low battery, obstacle danger, far from goal)
* Behaviour selection with anti-chattering (cooldown + hysteresis margin)
* A* path planning on a hand-crafted occupancy grid
* Local obstacle avoidance + “reverse when stuck on wall” logic
* Battery drain & charging behaviour
* Dynamic obstacle motion (moving walls using sin waves)
* Logging of:

  * distance travelled
  * number of behaviour switches
  * “collision-like” events
  * stuck re-plans
  * energy used

No external navigation libraries are used – planning and decisions are coded by hand.

---

## 3. What is pre-built / external

We rely on:

* **Webots API**: `Supervisor`, motors, distance sensors, node translation/rotation
* **Standard Python libraries**:

  * `math` (angles, `hypot`, etc.)
  * `heapq` (priority queue for A*)
  * `os` (check log file)
* **Webots project structure**: standard `worlds/` and `controllers/` layout

We don’t use ROS or any external path planning / VFH packages.

---

## 4. Folder structure

* **`controllers/intelligent_robot_controller.py`**
  Our main controller with all behaviour logic, A*, fuzzy activation, logging, etc.

* **`analysis/`**

  * Stores CSV logs (e.g. `run_log.csv`) from experiments
  * Python scripts for plotting behaviour over time, comparing runs, etc.

* **`worlds/`**
  Webots world file(s): robot, static obstacles, moving walls, goal, charging station.

* **Other `.txt` files**
  Notes, parameters, and helper documentation.

---

## 5. Group members and Contributions

All four team members were involved in debugging, experiments, and writing the report.  
Below is a short summary of each person’s main focus in the code.

* `littleHandsomeboy`- **Wenjun He** – Battery & behaviour switching  
  - Implemented the battery model and charging logic in `RobotState` and `update_battery()`.  
  - Added the `fuzzy_low_battery()` function and the CHARGE priority rule in `select_behavior()`.  
  - Helped implement the CHARGE behaviour in `compute_command()` and tuned low-battery thresholds.

* `xc-LEUNG`- **Xiaocong Liang** – Sensors & local obstacle avoidance  
  - Set up the 8 IR distance sensors and wrote `update_sensors()` (obstacle danger, left/right obstacle).  
  - Implemented `fuzzy_obstacle_near()` and the AVOID behaviour rules in `compute_command()`.  
  - Tuned thresholds and gains so the robot can pass moving walls with fewer collisions.

* `Duochuan9`- **Hongbo Han** – Behaviour tree & stability / logging  
  - Implemented the behaviour tree framework (`BTNode`, `Selector`, `ConditionNode`, `ActionNode`).  
  - Added cooldown, hysteresis, and switch counting in `select_behavior()` to reduce jitter.  
  - Implemented stuck detection, logging (`init_log_file()`, `log_status()`), and final summary output.

* `cwjgiegie` - **Wenjian Chen** – Mapping & A* global planning  
  - Implemented the occupancy grid map (`build_static_occupancy()`, `mark_obstacle_rect()`) and  
    coordinate conversion (`world_to_grid()`, `grid_to_world()`).  
  - Wrote the A* planner `astar()` and integrated waypoints via  
    `plan_path_to_high_level_target()` and `get_current_path_target_point()`.  
  - Connected path following to `update_pose()` (distance, heading error, waypoint update, `mission_done`).

