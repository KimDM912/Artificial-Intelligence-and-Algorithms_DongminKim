"""Conservative LPT-EFT baseline with an exclusive cell and home-return trips."""

from __future__ import annotations

from common import common as c
from common.common import Record, RobotState, Task, add_record
from common.slicer import safe_robots

ALGORITHM_NAME = "lpt_eft"


def schedule(tasks: list[Task]) -> tuple[dict[int, RobotState], list[dict[str, object]]]:
    config = c.CONFIG
    if config is None:
        raise ValueError("Load a case config before scheduling")
    arc_on = config.process.arc_on_time_s or 0.0
    arc_off = config.process.arc_off_time_s or 0.0
    states = {
        rid: RobotState(
            rid,
            tuple(c.ROBOTS[rid]["home"]),
            records=[Record(rid, 0.0, tuple(c.ROBOTS[rid]["home"]), "W")],
        )
        for rid in sorted(c.ROBOTS)
    }
    assignments = []
    release = 0.0
    by_layer = {}
    for task in tasks:
        by_layer.setdefault(task.layer, []).append(task)

    def route(robot_id, task):
        home = tuple(c.ROBOTS[robot_id]["home"])
        safe_z = max(home[2], config.process.safe_travel_z_mm or home[2], task.start[2])
        approach = [
            home,
            (home[0], home[1], safe_z),
            (task.start[0], task.start[1], safe_z),
            task.start,
        ]
        retreat = [task.end, (task.end[0], task.end[1], safe_z), (home[0], home[1], safe_z), home]
        return approach, retreat

    def move(robot, points, time):
        for point in points[1:]:
            distance = c.distance_3d(robot.point, point)
            if distance < 1e-9:
                continue
            add_record(robot.records, Record(robot.robot_id, time, robot.point, "T"))
            time += distance / config.process.travel_speed_mm_s
            add_record(robot.records, Record(robot.robot_id, time, point, "W"))
            robot.point = point
        return time

    for layer in sorted(by_layer):
        for task in sorted(by_layer[layer], key=lambda t: (-t.length, t.task_id)):
            candidates = []
            for rid in safe_robots(task.points, config):
                approach, retreat = route(rid, task)
                travel = sum(
                    c.distance_3d(a, b)
                    for path in (approach, retreat)
                    for a, b in zip(path, path[1:], strict=False)
                )
                finish = (
                    release
                    + travel / config.process.travel_speed_mm_s
                    + task.length / config.process.deposition_speed_mm_s
                    + arc_on
                    + arc_off
                )
                candidates.append((finish, rid, approach, retreat))
            if not candidates:
                raise ValueError(f"Task {task.task_id}: no safe reachable robot")
            _, rid, approach, retreat = min(candidates)
            robot = states[rid]
            time = move(robot, approach, release)
            time += arc_on
            deposit_start = time
            for point in task.points[1:]:
                add_record(robot.records, Record(rid, time, robot.point, "D"))
                time += c.distance_3d(robot.point, point) / config.process.deposition_speed_mm_s
                add_record(robot.records, Record(rid, time, point, "W"))
                robot.point = point
            deposit_finish = time
            time += arc_off
            release = move(robot, retreat, time)
            robot.available_s = release
            assignments.append(
                dict(
                    task_id=task.task_id,
                    layer=layer,
                    robot_id=rid,
                    deposit_start_s=deposit_start,
                    finish_s=deposit_finish,
                )
            )

    for robot in states.values():
        add_record(robot.records, Record(robot.robot_id, max(release, 1e-6), robot.point, "W"))
    return states, assignments


if __name__ == "__main__":
    from common.runner import run

    raise SystemExit(run(schedule, ALGORITHM_NAME))
