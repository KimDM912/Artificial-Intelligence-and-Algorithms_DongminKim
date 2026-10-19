"""Shared collaborative planning and trajectory rendering for three WAAM robots.

Short swept motion
segments reserve space only during their own time windows, allowing jobs
with intersecting overall footprints to overlap safely. Successive jobs connect
from the previous deposition endpoint using straight or safe-height travel.
Task-specific support release times are enforced. Initial task positions and
terminal endpoints compete with optional home/clearance moves. Every idle robot
remains an obstacle at its actual position before, between and after its jobs.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass, replace
from itertools import combinations, permutations

from shapely import LineString, MultiPoint

from . import common as c
from .common import Record, RobotState, add_record
from .feasibility import safe_robots
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
    retreat_wait_s: float = 0.0


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


def make_route(task, i, rid, origin, source=-1, *, return_home=False, approach_mode="straight"):
    """Construct one travel candidate under the supplied XY robot collision model."""
    if approach_mode not in ("safe_height", "straight", "initial", "via_home"):
        raise ValueError(f"Unknown approach mode: {approach_mode}")
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
    if approach_mode in ("straight", "initial"):
        approach = (origin, task.start)
    if approach_mode == "via_home":
        approach = (origin, (origin[0], origin[1], safe_z),
                    (home[0], home[1], safe_z), home,
                    (home[0], home[1], safe_z),
                    (task.start[0], task.start[1], safe_z), task.start)
    # When already at the next start, there is no clearance move to perform.
    if origin == task.start and approach_mode != "via_home":
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
        (i, rid, source, int(return_home), approach_mode),
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
            task_routes[rid] = make_route(task, i, rid, task.start, -2, approach_mode="initial")
            # Static ranks/construction hints use processing only. Actual EFT
            # includes predecessor-dependent travel, waits and optional egress.
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
    guard = (config.collision.geometry_epsilon_mm if config.collision.touching_is_collision
             else -config.collision.geometry_epsilon_mm)
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
        clearance = lcfg.tcp_radius_mm + rcfg.tcp_radius_mm
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
        self.terminal_conflicts = OrderedDict()
        self.previous_sequence = ()
        self.previous_checkpoints = []
        self.prefix_reused_tasks = 0

    def transition(self, i, rid, source=-1, *, return_home=False, approach_mode="straight"):
        key = (i, rid, source, int(return_home), approach_mode)
        if source == -2 and not return_home and approach_mode == "initial":
            return self.routes[i][rid]
        if key in self.transitions:
            self.transitions.move_to_end(key)
            return self.transitions[key]
        task = self.ordered[i]
        origin = (task.start if source == -2 else tuple(c.ROBOTS[rid]["home"])
                  if source == -1 else self.ordered[source].end)
        points = (origin, *task.points)
        if return_home or approach_mode == "via_home" or source == -1:
            points = (*points, tuple(c.ROBOTS[rid]["home"]))
        plan = (
            make_route(task, i, rid, origin, source, return_home=return_home,
                       approach_mode=approach_mode)
            if rid in safe_robots(points, c.CONFIG)
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

    def endpoints_conflict(self, rid, point, sid, other_point):
        """Two infinite endpoint waits cannot be separated by delaying either job.

        Use the existing collision engine on single stationary segments so all
        clearance, touching and numerical-boundary rules remain unchanged.
        """
        key = (rid, point, sid, other_point)
        if key in self.terminal_conflicts:
            self.terminal_conflicts.move_to_end(key)
            return self.terminal_conflicts[key]
        intervals = self.between(self.stationary(rid, point, math.inf),
                                 self.stationary(sid, other_point, math.inf))
        result = any(low == -math.inf and high == math.inf for low, high in intervals)
        self.terminal_conflicts[key] = result
        if len(self.terminal_conflicts) > 4096:
            self.terminal_conflicts.popitem(last=False)
        return result

    def with_terminal_wait(self, plan):
        """Last endpoints stay occupied until a later job replaces the wait."""
        end = plan.retreat[-1]
        motion = (*plan.motion, MotionSegment(plan.duration_s, math.inf, end, end))
        return replace(plan, motion=motion, key=(*plan.key, "terminal"),
                       spatial_key=tuple((*s.start[:2], *s.end[:2]) for s in motion))

    def egress_route(self, plan):
        """Isolate a home retreat so its departure can be delayed independently."""
        path = plan.retreat
        process = c.CONFIG.process.model_copy(update={"arc_on_time_s": 0.0, "arc_off_time_s": 0.0})
        motion = build_motion(path, (path[-1],), (path[-1],), process,
                              c.CONFIG.simulation.max_tcp_step_mm)
        base = c.ROBOTS[plan.robot_id]["base"]
        return RoutePlan(plan.robot_id, path, (path[-1],), plan.retreat_s, 0.0, 0.0,
                         plan.retreat_s, MultiPoint([base[:2], *(p[:2] for p in path)]).convex_hull,
                         _path_geometry(path), motion, (*plan.key, "egress"),
                         tuple((*s.start[:2], *s.end[:2]) for s in motion))

    def delayed_retreat(self, plan, wait_s):
        """Insert an explicit stationary wait after arc-off, before the retreat."""
        if wait_s == 0.0:
            return plan
        if not math.isfinite(wait_s) or wait_s < 0:
            raise ValueError("Retreat wait must be finite and nonnegative")
        work = self.transition(plan.key[0], plan.robot_id, plan.key[2],
                               approach_mode=plan.key[4])
        if work is None:
            raise ValueError("Cannot reconstruct the deposition route before retreat")
        egress = self.egress_route(plan)
        departure = work.duration_s + wait_s
        motion = (*work.motion,
                  MotionSegment(work.duration_s, departure, plan.retreat[0], plan.retreat[0]),
                  *(replace(s, start_s=departure + s.start_s, finish_s=departure + s.finish_s)
                    for s in egress.motion))
        return replace(plan, duration_s=departure + egress.duration_s,
                       motion=motion, retreat_wait_s=wait_s,
                       key=(*plan.key[:5], "delayed_egress", wait_s),
                       spatial_key=tuple((*s.start[:2], *s.end[:2]) for s in motion))


class ScheduleBuilder:
    """Persistent candidate schedules with safe optional endpoint connections.

    Endpoints are occupied until departure, even after the last scheduled job.
    A return home is an optional collision-avoidance route, not a requirement.
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

    def obstacles(self, rid, earliest):
        for sid, jobs in self.by_robot.items():
            if sid == rid:
                continue
            if not jobs:
                yield 0.0, self.context.stationary(sid, tuple(c.ROBOTS[sid]["home"]), math.inf)
                continue
            first = jobs[0]
            if first.start_s >= earliest:
                yield 0.0, self.context.stationary(sid, first.plan.approach[0], first.start_s)
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
                    if job.start_s > previous.finish_s:
                        yield (
                            previous.finish_s,
                            self.context.stationary(
                                sid, previous.plan.retreat[-1], job.start_s - previous.finish_s
                            ),
                        )
                yield job.start_s, job.plan
            last = jobs[-1]
            yield last.finish_s, self.context.stationary(sid, last.plan.retreat[-1], math.inf)

    def endpoint_blocked(self, rid, endpoint):
        """Check the endpoint independently of the candidate approach path."""
        for sid, jobs in self.by_robot.items():
            if sid == rid:
                continue
            other_endpoint = jobs[-1].plan.retreat[-1] if jobs else tuple(c.ROBOTS[sid]["home"])
            if self.context.endpoints_conflict(rid, endpoint, sid, other_endpoint):
                return True
        return False

    def terminal_blocked(self, plan):
        """Reject only unavoidable collisions between infinite stationary tails."""
        if not plan.motion:
            return False
        tail = plan.motion[-1]
        if tail.finish_s != math.inf or tail.start != tail.end:
            return False
        return self.endpoint_blocked(plan.robot_id, tail.end)

    def earliest(self, plan, earliest):
        if self.terminal_blocked(plan):
            return math.inf
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
                start = math.nextafter(high, math.inf)
        return start

    def safe_wait(self, rid, point, start, finish):
        if finish <= start:
            return True
        wait = self.context.stationary(rid, point, finish - start)
        for time, other in self.obstacles(rid, start):
            if any(low < start - time < high for low, high in self.context.between(wait, other)):
                return False
        return True

    def append(self, i, rid, *, release_s=0.0, in_place=False):
        task = self.context.ordered[i]
        release = release_s
        if not math.isfinite(release) or release < 0:
            raise ValueError("Task release must be finite and nonnegative")
        blocked_returns = {
            False: self.endpoint_blocked(rid, task.end),
            True: self.endpoint_blocked(rid, tuple(c.ROBOTS[rid]["home"])),
        }
        if all(blocked_returns.values()):
            return None
        old_jobs = self.by_robot[rid]
        previous = old_jobs[-1] if old_jobs else None
        candidates = []
        origins = []
        if previous is not None:
            ready = self.work_available(rid)
            trimmed = self.context.transition(previous.task_index, rid, previous.plan.key[2],
                                             approach_mode=previous.plan.key[4])
            if trimmed is not None:
                for mode in ("straight", "safe_height", "via_home"):
                    origins.append((previous.task_index, ready, mode, trimmed))
            if previous.plan.retreat[-1] == tuple(c.ROBOTS[rid]["home"]):
                for mode in ("straight", "safe_height"):
                    origins.append((-1, previous.finish_s, mode, None))
        else:
            origins = [(-2, 0.0, "initial", None), (-1, 0.0, "straight", None),
                       (-1, 0.0, "safe_height", None)]
        for rank, (source, ready, mode, trimmed) in enumerate(origins):
            for return_home in (False, True):
                if blocked_returns[return_home]:
                    continue
                plan = self.context.transition(i, rid, source, return_home=return_home,
                                               approach_mode=mode)
                if plan is None:
                    continue
                if candidates and max(release, ready) + plan.duration_s >= min(x[0] for x in candidates):
                    # A collision delay can only increase this lower bound.
                    continue
                start = self.earliest(self.context.with_terminal_wait(plan), max(release, ready))
                if not math.isfinite(start):
                    continue
                if not self.safe_wait(rid, plan.approach[0], ready, start):
                    continue
                candidates.append((start + plan.duration_s, 2 * rank + int(return_home),
                                   start, plan, trimmed))
        if not candidates:
            return None
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

    def parking_alternatives(self):
        """Try optional egress of blockers when no ready task can be appended.

        Only collision-free egress appended to an existing last job is allowed;
        deposition times stay fixed. All egress is rendered and timed.
        """
        ids = [rid for rid, jobs in self.by_robot.items()
               if jobs and not jobs[-1].plan.key[3]]
        for count in range(1, len(ids) + 1):
            for subset in combinations(ids, count):
                by_robot = dict(self.by_robot)
                for rid in subset:
                    old = by_robot[rid][-1]
                    plan = self.context.transition(old.task_index, rid, old.plan.key[2],
                                                   return_home=True, approach_mode=old.plan.key[4])
                    if plan is None:
                        break
                    by_robot[rid] = (*by_robot[rid][:-1],
                                    replace(old, plan=plan, finish_s=old.start_s + plan.duration_s))
                else:
                    candidate = ScheduleBuilder(self.context, by_robot, self.priority)
                    if all(candidate.earliest(self.context.with_terminal_wait(by_robot[rid][-1].plan),
                                              by_robot[rid][-1].start_s)
                           == by_robot[rid][-1].start_s for rid in subset):
                        yield candidate

    def wait_and_park(self, rid):
        """Wait safely at the last endpoint, then leave once the route is clear."""
        old = self.by_robot[rid][-1]
        plan = self.context.transition(old.task_index, rid, old.plan.key[2],
                                       return_home=True, approach_mode=old.plan.key[4])
        if plan is None:
            return None
        work = self.context.transition(old.task_index, rid, old.plan.key[2],
                                       approach_mode=old.plan.key[4])
        ready = old.start_s + work.duration_s
        egress = self.context.egress_route(plan)
        departure = self.earliest(self.context.with_terminal_wait(egress), ready)
        if not math.isfinite(departure) or not self.safe_wait(rid, plan.retreat[0], ready, departure):
            return None
        delayed = self.context.delayed_retreat(plan, departure - ready)
        by_robot = dict(self.by_robot)
        by_robot[rid] = (*by_robot[rid][:-1],
                        replace(old, plan=delayed, finish_s=old.start_s + delayed.duration_s))
        candidate = ScheduleBuilder(self.context, by_robot, self.priority, self.layer, self.release)
        # Recheck the complete modified job, including past deposition and the wait.
        if candidate.earliest(self.context.with_terminal_wait(delayed), old.start_s) != old.start_s:
            return None
        return candidate

    def waiting_parking_alternatives(self):
        """Fallback only: try safe delayed departures in each robot order."""
        ids = [rid for rid, jobs in self.by_robot.items() if jobs and not jobs[-1].plan.key[3]]
        for count in range(1, len(ids) + 1):
            for order in permutations(ids, count):
                candidate = self
                for rid in order:
                    candidate = candidate.wait_and_park(rid)
                    if candidate is None:
                        break
                if candidate is not None:
                    yield candidate


def best_eft(builder, i, robot_ids, *, release_s=0.0):
    candidates = []
    for rid in sorted(robot_ids):
        candidate = builder.append(i, rid, release_s=release_s)
        if candidate is None:
            continue
        job = candidate.by_robot[rid][-1]
        candidates.append((job.finish_s, job.start_s, rid, candidate))
    if not candidates:
        return None, None
    _, _, rid, candidate = min(candidates, key=lambda row: row[:3])
    return rid, candidate


def render(ordered, routes, genes, *, jobs, priority=None):
    """Render a collaborative schedule, including full robot occupancy intervals."""
    process = c.CONFIG.process
    arc_on = process.arc_on_time_s or 0.0
    arc_off = process.arc_off_time_s or 0.0
    initial = {rid: tuple(c.ROBOTS[rid]["home"]) for rid in c.ROBOTS}
    for job in sorted(jobs, key=lambda item: item.start_s, reverse=True):
        initial[job.robot_id] = job.plan.approach[0]
    states = {
        rid: RobotState(
            rid,
            initial[rid],
            records=[Record(rid, 0.0, initial[rid], "W")],
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
        time = move(robot, plan.retreat, time + arc_off + plan.retreat_wait_s)
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
                egress_wait_time_s=plan.retreat_wait_s,
                approach_mode=plan.key[4],
                transition_from_task=(
                    ordered[plan.key[2]].task_id if plan.key and plan.key[2] >= 0
                    else "initial" if plan.key[2] == -2 else "home"
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
