"""Shared collaborative planning and trajectory rendering for three WAAM robots.

Short swept motion
segments reserve space only during their own time windows, allowing jobs
with intersecting overall footprints to overlap safely. Successive jobs connect
from the previous deposition endpoint; safe-height travel and layer precedence
are retained. A final home return is reserved and removed when a safe direct
successor is committed, so stationary endpoint waits cannot be overlooked.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass, replace

from shapely import LineString, MultiPoint

from . import common as c
from .common import Record, RobotState, add_record
from .slicer import safe_robots
from .temporal import TIME_PAD_S, MotionSegment, TemporalConflicts, build_motion


@dataclass(frozen=True)
class RoutePlan:
    robot_id: int
    approach: tuple[c.Point, ...]
    retreat: tuple[c.Point, ...]
    approach_s: float
    deposition_s: float
    retreat_s: float
    duration_s: float
    arm_sweep: object
    tcp_sweep: object
    motion: tuple[MotionSegment, ...]
    key: tuple = ()
    spatial_key: tuple = ()


@dataclass(frozen=True)
class ScheduledJob:
    task_index: int
    robot_id: int
    start_s: float
    deposit_start_s: float
    deposit_finish_s: float
    finish_s: float
    plan: RoutePlan | None = None


def _path_geometry(points):
    xy = [point[:2] for point in points]
    return MultiPoint(xy) if len(set(xy)) == 1 else LineString(xy)


def make_route(task, i, rid, origin, source=-1, *, return_home=True):
    """Travel from the actual predecessor, using the configured safe height."""
    config, process = c.CONFIG, c.CONFIG.process
    home = tuple(c.ROBOTS[rid]["home"])
    safe_z = max(
        home[2], origin[2], process.safe_travel_z_mm or home[2], *(p[2] for p in task.points)
    )
    approach = (
        origin,
        (origin[0], origin[1], safe_z),
        (task.start[0], task.start[1], safe_z),
        task.start,
    )
    # When already at the next start, there is no clearance move to perform.
    if origin == task.start:
        approach = (origin,)
    retreat = (
        (task.end, (task.end[0], task.end[1], safe_z), (home[0], home[1], safe_z), home)
        if return_home
        else (task.end,)
    )
    approach_s = (
        math.fsum(math.dist(a, b) for a, b in zip(approach, approach[1:]))
        / process.travel_speed_mm_s
    )
    retreat_s = (
        math.fsum(math.dist(a, b) for a, b in zip(retreat, retreat[1:])) / process.travel_speed_mm_s
    )
    deposition_s = task.length / process.deposition_speed_mm_s
    duration = (
        approach_s
        + deposition_s
        + retreat_s
        + (process.arc_on_time_s or 0.0)
        + (process.arc_off_time_s or 0.0)
    )
    full_path = (*approach, *task.points, *retreat)
    base = c.ROBOTS[rid]["base"]
    motion = build_motion(
        approach, task.points, retreat, process, config.simulation.max_tcp_step_mm
    )
    return RoutePlan(
        rid,
        approach,
        retreat,
        approach_s,
        deposition_s,
        retreat_s,
        duration,
        MultiPoint([base[:2], *(p[:2] for p in full_path)]).convex_hull,
        _path_geometry(full_path),
        motion,
        (i, rid, source, int(return_home)),
        tuple((*s.start[:2], *s.end[:2]) for s in motion),
    )


def prepare(tasks):
    """Build feasible routes with timed motion segments for continuous XY checking."""
    config = c.CONFIG
    if config is None:
        raise ValueError("Load a case config before scheduling")
    ordered = sorted(tasks, key=lambda task: (task.layer, task.task_id))
    if len({task.task_id for task in ordered}) != len(ordered):
        raise ValueError("Task IDs must be unique")

    process = config.process
    if process.travel_speed_mm_s <= 0 or process.deposition_speed_mm_s <= 0:
        raise ValueError("Travel and deposition speeds must be positive")
    arc_on = process.arc_on_time_s or 0.0
    arc_off = process.arc_off_time_s or 0.0
    routes, costs = [], []
    for i, task in enumerate(ordered):
        if (
            not task.task_id
            or task.layer < 0
            or task.length <= 0
            or any(not math.isfinite(v) for point in task.points for v in point)
        ):
            raise ValueError(f"Task {task.task_id}: invalid identity, layer or path")
        task_routes, task_costs = {}, {}
        for rid in sorted(safe_robots(task.points, config)):
            home = tuple(c.ROBOTS[rid]["home"])
            task_routes[rid] = make_route(task, i, rid, home)
            # Static ranks/construction hints use processing only. Actual EFT
            # includes predecessor-dependent travel, waits and final return.
            task_costs[rid] = task_routes[rid].deposition_s + arc_on + arc_off
        if not task_costs:
            raise ValueError(f"Task {task.task_id}: no safe reachable robot")
        routes.append(task_routes)
        costs.append(task_costs)
    return ordered, routes, costs


def routes_conflict(left: RoutePlan, right: RoutePlan) -> bool:
    """Broad-phase test only: overlapping footprints require time-window refinement."""
    if left.robot_id == right.robot_id:
        return True
    config = c.CONFIG
    robots = {robot.id: robot for robot in config.robots}
    lcfg, rcfg = robots[left.robot_id], robots[right.robot_id]
    guard = max(0.01, config.collision.geometry_epsilon_mm)
    if config.collision.check_arm_envelope:
        clearance = (
            lcfg.arm_envelope_radius_mm
            + rcfg.arm_envelope_radius_mm
            + config.collision.arm_clearance_mm
            + guard
        )
        if left.arm_sweep.distance(right.arm_sweep) <= clearance:
            return True
    if config.collision.check_tcp_radius:
        clearance = lcfg.tcp_radius_mm + rcfg.tcp_radius_mm + guard
        if left.tcp_sweep.distance(right.tcp_sweep) <= clearance:
            return True
    return False


def conflict_table(routes, ordered=None):
    """Shared origin-dependent route and collision caches for every algorithm."""
    return PlanningContext(ordered, routes)


class PlanningContext(TemporalConflicts):
    def __init__(self, ordered, routes):
        super().__init__(routes, c.CONFIG, routes_conflict)
        self.ordered = ordered
        self.transitions = OrderedDict()
        self.previous_sequence = ()
        self.previous_checkpoints = []
        self.prefix_reused_tasks = 0

    def transition(self, i, rid, source=-1, *, return_home=True):
        key = (i, rid, source, int(return_home))
        if source == -1 and return_home:
            return self.routes[i][rid]
        if key in self.transitions:
            self.transitions.move_to_end(key)
            return self.transitions[key]
        origin = tuple(c.ROBOTS[rid]["home"]) if source == -1 else self.ordered[source].end
        task = self.ordered[i]
        # Every route must also be safe while other robots are parked at home.
        plan = (
            make_route(task, i, rid, origin, source, return_home=return_home)
            if rid in safe_robots((origin, *task.points), c.CONFIG)
            else None
        )
        self.transitions[key] = plan
        if len(self.transitions) > 512:
            self.transitions.popitem(last=False)
        return plan

    def stationary(self, rid, point, duration):
        base = c.ROBOTS[rid]["base"]
        return RoutePlan(
            rid,
            (point,),
            (point,),
            0.0,
            0.0,
            0.0,
            duration,
            MultiPoint([base[:2], point[:2]]).convex_hull,
            _path_geometry((point,)),
            (MotionSegment(0.0, duration, point, point),),
            (-1, rid, point, duration),
            ((*point[:2], *point[:2]),),
        )

    def between(self, left, right):
        return self.offsets(left.key, left, left.robot_id, right.key, right, right.robot_id)


class ScheduleBuilder:
    """Persistent candidate schedules with safe optional endpoint connections.

    Each robot's last job reserves a home return. Appending a safe direct
    successor cancels that return, checks the endpoint wait and replaces the
    next approach. Previously committed trajectories of other robots stay fixed.
    """

    def __init__(self, context, by_robot=None, priority=(), layer=-1, release=0.0):
        self.context = context
        self.by_robot = by_robot if by_robot is not None else {rid: () for rid in c.ROBOTS}
        self.priority, self.layer, self.release = priority, layer, release

    def snapshot(self):
        """Freeze lists before later in-place appends can change a saved prefix."""
        return ScheduleBuilder(
            self.context,
            {rid: tuple(jobs) for rid, jobs in self.by_robot.items()},
            tuple(self.priority),
            self.layer,
            self.release,
        )

    def fork(self):
        return ScheduleBuilder(
            self.context, dict(self.by_robot), self.priority, self.layer, self.release
        )

    @property
    def jobs(self):
        indexed = {job.task_index: job for jobs in self.by_robot.values() for job in jobs}
        return tuple(indexed[i] for i in self.priority)

    @property
    def makespan(self):
        return max((jobs[-1].finish_s for jobs in self.by_robot.values() if jobs), default=0.0)

    def work_available(self, rid):
        jobs = self.by_robot[rid]
        return jobs[-1].deposit_finish_s + (c.CONFIG.process.arc_off_time_s or 0.0) if jobs else 0.0

    def layer_jobs(self, layer):
        for jobs in self.by_robot.values():
            for job in reversed(jobs):
                job_layer = self.context.ordered[job.task_index].layer
                if job_layer < layer:
                    break
                if job_layer == layer:
                    yield job

    def release_for(self, layer):
        if layer < self.layer:
            raise ValueError("Cannot append a task before the current layer")
        return (
            max(self.work_available(rid) for rid in self.by_robot)
            if layer > self.layer
            else self.release
        )

    def obstacles(self, rid, earliest):
        for sid, jobs in self.by_robot.items():
            if sid == rid:
                continue
            # Robot jobs are chronologically ordered; skip completed history.
            low, high = 0, len(jobs)
            while low < high:
                middle = (low + high) // 2
                if jobs[middle].finish_s < earliest - TIME_PAD_S:
                    low = middle + 1
                else:
                    high = middle
            for index in range(low, len(jobs)):
                job = jobs[index]
                if index:
                    previous = jobs[index - 1]
                    if job.start_s > previous.finish_s and previous.plan.retreat[-1] != tuple(
                        c.ROBOTS[sid]["home"]
                    ):
                        yield (
                            previous.finish_s,
                            self.context.stationary(
                                sid, previous.plan.retreat[-1], job.start_s - previous.finish_s
                            ),
                        )
                yield job.start_s, job.plan
            # Initial/final home waits are covered by safe_robots() for every route.

    def earliest(self, plan, earliest):
        forbidden = []
        for time, other in self.obstacles(plan.robot_id, earliest):
            forbidden.extend(
                (time + low, time + high) for low, high in self.context.between(plan, other)
            )
        start = earliest
        for low, high in sorted(forbidden):
            if low >= start:
                break
            if start < high:
                start = high
        return start

    def safe_wait(self, rid, point, start, finish):
        if finish <= start:
            return True
        wait = self.context.stationary(rid, point, finish - start)
        for time, other in self.obstacles(rid, start):
            if any(low < start - time < high for low, high in self.context.between(wait, other)):
                return False
        return True

    def append(self, i, rid, *, in_place=False):
        task = self.context.ordered[i]
        release = self.release_for(task.layer)
        old_jobs = self.by_robot[rid]
        previous = old_jobs[-1] if old_jobs else None
        candidates = []
        if previous is not None:
            plan = self.context.transition(i, rid, previous.task_index)
            ready = self.work_available(rid)
            if plan is not None:
                start = self.earliest(plan, max(release, ready))
                if self.safe_wait(rid, self.context.ordered[previous.task_index].end, ready, start):
                    trimmed = self.context.transition(
                        previous.task_index, rid, previous.plan.key[2], return_home=False
                    )
                    if trimmed is not None:
                        candidates.append((start + plan.duration_s, 0, start, plan, trimmed))
        # Guaranteed feasible fallback: keep the already reserved home return.
        home_plan = self.context.transition(i, rid)
        start = self.earliest(home_plan, max(release, previous.finish_s if previous else 0.0))
        candidates.append((start + home_plan.duration_s, 1, start, home_plan, None))
        finish, _, start, plan, trimmed = min(candidates, key=lambda x: (x[0], x[1]))
        deposit = start + plan.approach_s + (c.CONFIG.process.arc_on_time_s or 0.0)
        job = ScheduledJob(i, rid, start, deposit, deposit + plan.deposition_s, finish, plan)
        if in_place:
            # A fixed chromosome has no branch to retain. Avoid copying the
            # entire prefix for each task in large multi-layer evaluations.
            own_jobs = old_jobs if isinstance(old_jobs, list) else list(old_jobs)
            if trimmed is not None:
                own_jobs[-1] = replace(previous, plan=trimmed, finish_s=self.work_available(rid))
            own_jobs.append(job)
            self.by_robot[rid] = own_jobs
            if not isinstance(self.priority, list):
                self.priority = list(self.priority)
            self.priority.append(i)
            self.layer, self.release = task.layer, release
            return self
        own_jobs = old_jobs
        if trimmed is not None:
            own_jobs = (
                *old_jobs[:-1],
                replace(previous, plan=trimmed, finish_s=self.work_available(rid)),
            )
        by_robot = dict(self.by_robot)
        by_robot[rid] = (*own_jobs, job)
        return ScheduleBuilder(self.context, by_robot, (*self.priority, i), task.layer, release)


def best_eft(builder, i, robot_ids):
    candidates = []
    for rid in sorted(robot_ids):
        candidate = builder.append(i, rid)
        job = candidate.by_robot[rid][-1]
        candidates.append((job.finish_s, job.start_s, rid, candidate))
    _, _, rid, candidate = min(candidates, key=lambda row: row[:3])
    return rid, candidate


def render(ordered, routes, genes, *, jobs, priority=None):
    """Render a collaborative schedule, including full robot occupancy intervals."""
    process = c.CONFIG.process
    arc_on = process.arc_on_time_s or 0.0
    arc_off = process.arc_off_time_s or 0.0
    states = {
        rid: RobotState(
            rid,
            tuple(c.ROBOTS[rid]["home"]),
            records=[Record(rid, 0.0, tuple(c.ROBOTS[rid]["home"]), "W")],
        )
        for rid in sorted(c.ROBOTS)
    }
    assignments = []

    def move(robot, points, time):
        for point in points[1:]:
            distance = c.distance_3d(robot.point, point)
            if distance < 1e-9:
                continue
            add_record(robot.records, Record(robot.robot_id, time, robot.point, "T"))
            time += distance / process.travel_speed_mm_s
            add_record(robot.records, Record(robot.robot_id, time, point, "W"))
            robot.point = point
        return time

    ordered_jobs = sorted(jobs, key=lambda item: (item.robot_id, item.start_s, item.task_index))
    for job in ordered_jobs:
        task = ordered[job.task_index]
        plan = job.plan or routes[job.task_index][job.robot_id]
        robot = states[job.robot_id]
        if job.start_s < robot.available_s - 1e-6:
            raise ValueError("Overlapping jobs were assigned to one robot")
        if math.dist(robot.point, plan.approach[0]) > 1e-6:
            raise ValueError("Route does not start at the robot's actual position")
        add_record(robot.records, Record(robot.robot_id, job.start_s, robot.point, "W"))
        time = move(robot, plan.approach, job.start_s) + arc_on
        deposit_start = time
        for point in task.points[1:]:
            distance = c.distance_3d(robot.point, point)
            if distance < 1e-9:
                continue
            add_record(robot.records, Record(robot.robot_id, time, robot.point, "D"))
            time += distance / process.deposition_speed_mm_s
            add_record(robot.records, Record(robot.robot_id, time, point, "W"))
            robot.point = point
        deposit_finish = time
        time = move(robot, plan.retreat, time + arc_off)
        if not math.isclose(time, job.finish_s, rel_tol=0.0, abs_tol=1e-5):
            raise ValueError("Rendered trajectory duration does not match its schedule")
        robot.available_s = time
        assignments.append(
            dict(
                task_id=task.task_id,
                layer=task.layer,
                robot_id=job.robot_id,
                job_start_s=job.start_s,
                job_finish_s=job.finish_s,
                deposit_start_s=deposit_start,
                finish_s=deposit_finish,
                approach_time_s=plan.approach_s,
                return_home_time_s=plan.retreat_s,
                transition_from_task=(
                    ordered[plan.key[2]].task_id if plan.key and plan.key[2] >= 0 else "home"
                ),
            )
        )

    makespan = max((state.available_s for state in states.values()), default=0.0)
    # A nonempty trajectory needs a positive interval; empty schedules keep zero.
    for robot in states.values():
        add_record(robot.records, Record(robot.robot_id, makespan, robot.point, "W"))
        robot.available_s = makespan
    assignments.sort(key=lambda row: (row["layer"], row["deposit_start_s"], row["task_id"]))
    return states, assignments
