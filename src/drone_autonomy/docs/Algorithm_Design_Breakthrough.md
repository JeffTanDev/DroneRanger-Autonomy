# Obstacle Persistence and Path Planning — PI Progress Report

**Purpose:** Report on design choices, methods tried vs. adopted vs. deprecated, and contributions for obstacle-aware 3D path planning on the drone stack.

---

## Executive Summary

We addressed two main issues: (1) **single-frame occupancy** causing the planner to treat unseen areas as free (leading to planning into the wrong side of obstacles and oscillation in front of large surfaces), and (2) **world-frame persistent grids** that were tried but caused the map to fill with obstacles and become unusable. We also implemented a **world buffer** variant: the world grid is used **only as soft cost** (buffer), not as hard obstacles, so paths can still pass through but prefer “never seen” regions. This improved behaviour somewhat over the original world grid, but **accuracy issues** (pose/depth alignment, rotation) still led to **many and messy obstacles** in the world grid, resulting in **unstable flight and very rugged paths**. We have therefore **paused** further work on the world buffer and are **focusing on improving the short-term (shorter) memory** approach. The solution currently adopted is **short-term local memory with strong decay** (no world grid), plus **replan gating by angular velocity**, **conservative avoidance** (inflation + buffer), and **waypoint advance logic**. This document records what was tried, what was kept, what was deprecated or paused, and the contributions made.

---

## 1. Problem Statement

The baseline pipeline replans periodically using **only the current depth image** to build a local 3D occupancy grid centered on the drone. That leads to:

| Symptom                                                                                                                     | Root cause                                                                                                                                                                       |
| --------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Planning to one side of an obstacle (e.g. left) then getting stuck or hitting the other side (e.g. right) on the next frame | Current view sees only one side; the other side is unobserved but treated as free, so the path goes there; next frame sees the obstacle and it is too late.                      |
| Oscillation or stuck in front of large surfaces (e.g. walls)                                                                | One frame sees lower part → plan goes up; next frame sees upper part → plan goes down. No fusion between frames, so the path flips between “see only top” and “see only bottom.” |

**Core issue:** Single-frame, no-memory occupancy → **unseen regions are treated as free** and **previously seen obstacles are not retained**, so the planner repeatedly “forgets” obstacles and makes inconsistent decisions.

---

## 2. Reference Ideas (Literature / Best Practice)

- **EGO-Planner (ZJU-FAST-Lab):** Obstacle information is stored when the trajectory encounters new obstacles (accumulation rather than per-frame reset).
- **Common practices:**
  - **Multi-frame / multi-view fusion:** Fuse multiple depth/occupancy frames in a common frame (world or local NED).
  - **Occupancy semantics:** **Occupied / free / unknown**; treat unknown as obstacle or high cost.
  - **Short-term persistence:** Once a voxel is occupied, keep it occupied for a short time unless repeatedly observed free (or use log-odds).

These motivated our attempts at persistence and our final choice of short-term local memory with strong decay.

---

## 3. Methods Tried, Adopted, and Deprecated

### 3.1 World-frame persistent 3D grid — **TRIED, DEPRECATED**

**Idea:** Maintain a world-NED 3D occupancy grid (e.g. centered at home or a fixed area). Each frame: project current depth to world NED, mark seen obstacles as occupied (with optional timestamp). Build the local planning grid by OR’ing the world grid with the current frame in the local window. Optionally clear cells after a TTL if not re-observed.

**What I did:** Designed and attempted a world-grid persistence (write occupied from current frame into world grid; read back into local grid with OR).

**Why deprecated:** In practice the map **only grew** (obstacles accumulated and were never cleared), so the environment tended to become fully occupied and the drone had no feasible path. In addition:

- **Asymmetric update:** Only “occupied” was written; “free” (ray-cast along depth rays) was not. So there was no mechanism to clear space.
- **Pose/depth misalignment:** Depth timestamp vs. vehicle pose (and rotation) errors caused the same obstacle to be written to different world cells across frames, creating ghost obstacles and drift.
- **Rotation amplifies error:** Under fast rotation, body→world transform errors are large; wrong cells were marked occupied and accumulated.

**Conclusion:** A world grid is hard to maintain correctly without symmetric updates (ray-cast free + occupied) and careful handling of rotation; we abandoned it in favor of a method that does not rely on a long-lived world grid.

---

### 3.1a World grid as soft cost only (world buffer) — **TRIED, PAUSED**

**Idea (Section 8.4 in earlier design):** Keep a world-NED 3D grid, but when merging it into the local planning grid, **do not** treat world-occupied cells as hard obstacles. Instead, mark them as **buffer (high cost)** only: A\* can still traverse these cells, but the path prefers regions that were never marked in the world grid. This way, even if the world grid has misalignments or accumulation, it does not block the path—it only makes the route slightly detour around “historically seen” areas.

**What I did:**

- Implemented in `astar_world_buffer.py`: a persistent 3D world grid (origin at NED 0,0,0), updated from current depth each frame (obstacle voxels written as occupied).
- In `build_3d_grid_ned`: after building the local grid from current depth (obstacles + inflation + buffer), **overlay** the world grid as **soft cost only**: for each local cell that is currently **free**, if the corresponding world-grid cell is occupied, set the local cell to **CELL_BUFFER** (not CELL_OBSTACLE). Current-frame obstacles remain hard; world grid only adds buffer cost.
- Replan gating by angular velocity was also added (skip replan when yaw rate magnitude exceeds a threshold) to reduce bad updates during fast rotation.

**Result:** The world buffer **did improve behaviour somewhat** compared to the original world grid (paths were no longer blocked by accumulated obstacles). However, **accuracy issues** (pose/depth alignment, rotation, timestamp mismatch) still caused the world grid to accumulate **many and messy obstacles** in wrong or duplicated positions. As a result, the **flight path became unstable** and **routes were very rugged** (the planner kept trying to avoid a large, noisy “soft cost” cloud). We have therefore **paused** further development on the world buffer and are **focusing on improving the short-term (shortern) memory** approach instead. World buffer remains available in `astar_world_buffer.py` for reference or future refinement (e.g. with ray-cast free and TTL).

---

### 3.2 Short-term local memory + strong decay — **ADOPTED** ✅

**Idea (Section 8.1 in earlier design):** Do **not** maintain a world grid. Instead keep a **short-lived local memory** of obstacle voxels: store them with a timestamp; use only those that (a) fall inside the **current** local grid and (b) have **last_seen** within a short TTL (e.g. 1–2 replan periods). OR these into the current frame’s grid. No world-frame alignment: memory is in world NED coordinates, but we only _read_ into the current local window and drop by TTL.

**What I did:**

- Implemented `_short_memory_voxels`: list of `(n, e, d, last_seen_s)` in world NED.
- In `build_3d_grid_ned`: (1) build grid from current depth; (2) prune memory by TTL; (3) for each stored voxel, if inside current local bounds, mark that cell occupied (OR); (4) append current frame’s obstacle voxels with current time.
- Parameter: `short_memory_ttl_s` (e.g. 4–12 s) so memory does not accumulate.

**Why adopted:** Avoids world-grid alignment and rotation issues; provides a small amount of “memory” (previous frame(s)) to reduce blind-side and oscillation problems without long-term buildup. Strong TTL keeps errors from persisting.

**Contribution:** Implemented and tuned this as the main persistence mechanism in `astar_shortern_memory.py`.

---

### 3.3 No replan when angular velocity is high — **ADOPTED** ✅

**Idea:** When the vehicle is rotating fast, pose/depth alignment is worse; replanning in that condition often produces bad or inconsistent paths. So: subscribe to body angular velocity; if its magnitude exceeds a threshold, **skip replan** for that cycle and keep following the existing path.

**What I did:**

- Subscribed to `/fmu/out/vehicle_angular_velocity` (PX4), store `angular_vel_xyz`.
- Parameter: `max_angular_vel_for_replan_rad_s` (e.g. 0.5 rad/s).
- In the planning loop: if `||angular_velocity|| > max_angular_vel_for_replan_rad_s`, do not call `plan_path_ned()` this cycle.

**Why adopted:** Reduces obviously wrong replans during fast turns and aligns with the design doc’s suggestion to avoid writing or trusting the world grid during rotation; here we avoid replanning altogether during high rotation.

**Contribution:** Added the subscription, state, parameter, and gating logic; documented in code and in this report.

---

### 3.4 Conservative avoidance (inflation + buffer + buffer cost) — **ADOPTED** ✅

**Idea:** Keep the path **farther** from obstacles instead of skimming them: (1) **Inflation:** expand obstacles by several cells (fully blocked); (2) **Buffer:** add extra layers of “flyable but high cost” cells; (3) **Buffer cost in A\*:** use a cost multiplier for stepping into buffer cells so the planner prefers paths that stay out of the buffer.

**What I did:**

- Kept and tuned parameters: `inflation_cells`, `buffer_cells`, `buffer_cost` (e.g. 1, 3, 30.0 in current config).
- A\* already supported buffer cells and `buffer_cost`; no change to the algorithm, only to default/tuned values and documentation.

**Why adopted:** User requirement for more conservative, “stay away from obstacles” behavior rather than flying close to them.

**Contribution:** Documented the role of each parameter and set conservative defaults suitable for PI/demo use.

---

### 3.5 Waypoint advance: “first waypoint ahead” + “skip behind” — **ADOPTED** ✅

**Idea:** At higher speed, the first waypoints of a new path are often **behind** the drone (planning latency + motion). If we keep targeting the first waypoint, the drone turns back. Fix: (1) When a new path is set, **start from the first waypoint that is ahead** of the drone (toward goal) instead of a fixed index (e.g. 1). (2) While following, **advance** the waypoint index not only when within `waypoint_reach_radius_m` but also when the **current waypoint is behind** the drone (dot product with goal direction), so we skip overflown waypoints.

**What I did:**

- Used existing `is_waypoint_ahead(wn, we, wd, px, py, pz, goal_n, goal_e, goal_d)` (dot product with goal).
- On new path: set `waypoint_idx` to the first `i` such that waypoint `i` is ahead; if none, use last waypoint.
- In the follow loop: advance waypoint if `dist <= reach_radius` **or** if the current waypoint is not ahead (already behind).

**Why adopted:** Eliminates the “waypoint behind → drone turns back” behavior at higher cruise speed.

**Contribution:** Designed and implemented this logic and wired it with goal NED and existing path reuse.

---

### 3.6 Velocity feedforward for faster flight — **ADOPTED** ✅

**Idea:** Publish trajectory setpoints with both **position** (next waypoint) and **velocity feedforward** (direction from current position to waypoint, magnitude = `cruise_speed_m_s`) so the controller can track the desired speed more quickly.

**What I did:**

- Extended `publish_sp()` with optional `velocity_feedforward=(vx, vy, vz)` in NED.
- In PLAN_FOLLOW, when sending the current waypoint, compute unit vector to waypoint and set velocity = `cruise_speed_m_s * direction`, then publish position + velocity.
- Documented that actual speed is still capped by PX4 (e.g. `MPC_XY_CRUISE`, `MPC_XY_VEL_MAX`).

**Why adopted:** User requirement to fly faster; feedforward improves tracking without changing the planning algorithm.

**Contribution:** Implemented and documented the interface and PX4 coupling.

---

### 3.7 Obstacle encounter report — **IMPLEMENTED, THEN REMOVED**

**Idea:** When obstacles are detected, write a diagnostic report (depth summary, processing steps, obstacle bearings/distances) to a configurable log directory for debugging.

**What I did:** Implemented `_write_obstacle_encounter_report`, parameters `obstacle_encounter_log_dir` and `obstacle_encounter_throttle_s`, and called it from `build_3d_grid_ned` when obstacles were present. Later **removed** this feature entirely (parameters, state, method, and call site) per user request to simplify the node.

**Status:** Not part of current codebase; retained in this report for completeness.

---

### 3.8 Single-frame + conservative (no persistent layer) — **TRIED, DEPRECATED**

**Summary:** We tried a pure single-frame approach with no persistent layer (grid init as unknown, ray-mark free, unknown as high cost in A*). Implemented in `astar_no_memory_conservative.py`. **Deprecated:** per-frame computation was too heavy (per–depth-point ray marking, large A* over unknown); we dropped it for now and may revisit after optimization. Main pipeline remains short-term local memory in `astar_shortern_memory.py`.

---

### 3.9 Other directions considered but not implemented (or paused)

- **Ray-casting “free” (for world grid):** If we had kept the world grid, we would need to mark ray-traversed cells as free to avoid endless accumulation. World grid was deprecated; world buffer was tried and paused, so this was not implemented.
- **Unknown = conservative:** Treat cells never hit by any ray as unknown and give them high cost or block in A\*. Tried in the single-frame conservative node; that approach is deprecated.
- **World grid as soft cost only (world buffer):** Implemented in `astar_world_buffer.py`; see §3.1a. Effect was somewhat better than the hard world grid, but accuracy issues led to many messy obstacles and rugged paths. **Paused;** focus is now on improving short-term memory. Optional future refinement if we revisit world representation (e.g. with ray-cast free and TTL).
- **Planning/perception decoupling (reuse same grid):** Replan without rebuilding the grid from depth every time; only OR new observations. Not implemented; current design uses short-term memory instead.

---

## 4. Current Implementation Summary

**Pipeline (e.g. in `astar_shortern_memory.py`):**

1. **Grid build:** `build_3d_grid_ned()`
   - Current depth → obstacle voxels in current local NED.
   - Prune short-term memory by TTL; OR memory voxels that fall in current local window into the grid.
   - Append current obstacles to `_short_memory_voxels` with timestamp.
   - Apply inflation (blocked) and buffer (high-cost) layers.

2. **Planning:** A\* on the grid with `buffer_cost`; path reuse (`reuse_waypoints`) to keep stable segments when still free.

3. **Replan gating:** If `||angular_velocity|| > max_angular_vel_for_replan_rad_s`, skip replan this cycle.

4. **Waypoint selection:** On new path, start at first waypoint ahead of drone; each cycle, advance waypoint if within reach or if current waypoint is behind.

5. **Control:** Publish trajectory setpoint (position = current waypoint, yaw toward waypoint, optional velocity feedforward at `cruise_speed_m_s`).

**Key parameters:**  
`short_memory_ttl_s`, `max_angular_vel_for_replan_rad_s`, `inflation_cells`, `buffer_cells`, `buffer_cost`, `cruise_speed_m_s`, `waypoint_reach_radius_m`, `replan_interval_s`, `reuse_waypoints`.

---

## 5. Summary Tables

### Methods status

| Method                                             | Status          | Notes                                                                                                        |
| -------------------------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------ |
| World-frame persistent grid                        | **Deprecated**  | Map filled with obstacles; rotation/pose error; asymmetric update.                                           |
| World grid as soft cost only (world buffer)        | **Paused**      | In `astar_world_buffer.py`; somewhat better than hard world grid, but accuracy → messy obstacles, rugged paths; focus now on shorter memory. |
| Single-frame + conservative (no persistent layer)  | **Deprecated**  | In `astar_no_memory_conservative.py`; heavy per-frame cost; dropped for now, may revisit after optimization. |
| Short-term local memory + strong decay             | **Adopted**     | Main persistence mechanism; no world grid.                                                                   |
| No replan when angular velocity high               | **Adopted**     | Avoids bad replans during turns.                                                                             |
| Conservative avoidance (inflation + buffer + cost) | **Adopted**     | Path stays farther from obstacles.                                                                           |
| Waypoint advance (first ahead + skip behind)       | **Adopted**     | Fixes turn-back at high speed.                                                                               |
| Velocity feedforward                               | **Adopted**     | Faster flight tracking.                                                                                      |
| Obstacle encounter report                          | **Removed**     | Was implemented; removed to simplify.                                                                        |
| Ray-cast free (world grid)                         | Not implemented | World grid deprecated.                                                                                       |
| Unknown = conservative (standalone)                | Not implemented | Tried in deprecated single-frame conservative node.                                                          |

### Problem → solution mapping

| Problem                                             | Approach used                                                             |
| --------------------------------------------------- | ------------------------------------------------------------------------- |
| Single-frame blind side / oscillation               | Short-term local memory + strong decay (OR previous frame(s) within TTL). |
| World grid “map full” / rotation error              | Deprecated hard world grid; tried world buffer (soft cost only), paused due to accuracy → rugged paths; use local memory only. |
| Bad replan during fast turn                         | No replan when angular velocity > threshold.                              |
| Path too close to obstacles                         | Larger inflation, buffer, and buffer_cost.                                |
| Drone turning back for past waypoints at high speed | First waypoint ahead + advance when waypoint behind.                      |
| Slow speed tracking                                 | Velocity feedforward + PX4 parameter guidance.                            |

---

## 6. Contributions (for PI)

1. **Root-cause analysis of world-grid failure:** Identified asymmetric update (occupied-only) and rotation/pose alignment as reasons the world grid became unusable; documented and switched to a design that does not rely on a long-lived world grid.

1a. **World buffer (world grid as soft cost only):** Implemented in `astar_world_buffer.py`; world-occupied cells overlay as buffer (high cost) only, not hard obstacles. Effect was somewhat better than the hard world grid, but accuracy issues still led to many messy obstacles and rugged, unstable paths. Documented the implementation and the decision to **pause** world buffer and **focus on improving the short-term (shortern) memory** approach.

2. **Design and implementation of short-term local memory:** Implemented TTL-based obstacle list, OR into current local grid only, and integration in `build_3d_grid_ned`; tuned TTL and documented tradeoffs.

3. **Replan gating by angular velocity:** Added subscription, state, parameter, and logic to skip replan when turning fast; documented rationale and coupling with design doc (e.g. “rotation → do not trust/write world grid”).

4. **Waypoint advance logic:** Designed and implemented “first waypoint ahead” and “advance when waypoint behind” so the drone does not turn back for overflown waypoints at higher speed; reused existing `is_waypoint_ahead` and goal NED.

5. **Conservative avoidance tuning:** Clarified and tuned inflation/buffer/buffer_cost for “stay away from obstacles” behavior and documented parameters for maintainability and PI reporting.

6. **Velocity feedforward and speed documentation:** Implemented optional velocity feedforward in setpoint publishing and documented dependency on PX4 parameters for actual speed limits.

7. **Code and documentation cleanup:** Removed obstacle encounter report feature and non-English comments as requested; kept this report as a single place for design rationale, methods tried/adopted/deprecated, and contributions for PI and future work.

8. **PI-facing report:** Turned the original design doc into this structured report: problem statement, methods tried vs. adopted vs. deprecated, current implementation summary, and explicit contributions list.

---

_Document version: World buffer (soft-cost-only world grid) implemented and paused; focus on improving short-term (shortern) memory. Main implementation: `astar_shortern_memory.py`. Paused/reference: `astar_world_buffer.py`. Deprecated/experimental: `astar_no_memory_conservative.py`._
