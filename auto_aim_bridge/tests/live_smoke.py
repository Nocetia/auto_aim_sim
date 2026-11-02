#!/usr/bin/env python3
"""Verify exact-corner and rendered-frame bridge modes against a live simulator."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import tempfile
import time
import uuid


MAX_PNP_POSITION_ERROR_METERS = 0.01
MAX_PNP_YAW_ERROR_RADIANS = 0.03
MAX_ACTIVE_COMMAND_PLANNING_AGE_SECONDS = 0.03
MAX_ROTATING_YAW_TRACKING_ERROR_RADIANS = 0.10
MAX_ROTATING_PITCH_TRACKING_ERROR_RADIANS = 0.04
MAX_RAW_CORNER_ERROR_PIXELS = 50.0

# A larger prediction horizon would already exceed the simulator's default
# wall-clock command watchdog and would most likely indicate mixed time bases.
MAX_PLANNING_LEAD_SECONDS = 0.25


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def load_state(state_path):
    try:
        state = json.loads(state_path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None

    if state.get("protocol") != "rm_aim_sim/2" or state.get("type") != "state":
        return None

    publication_age = time.time() - state.get("wall_unix_sec", 0.0)

    if not math.isfinite(publication_age) or not -0.1 <= publication_age <= 0.5:
        return None

    return state


def wait_for_state(state_path, simulator, timeout=8.0, after_publication=None):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        require(simulator.poll() is None, "The simulator exited before publishing state")
        state = load_state(state_path)

        if state is not None:
            is_new_publication = (
                after_publication is None
                or state["publication"] > after_publication
            )

            if is_new_publication:
                return state

        time.sleep(0.01)

    raise AssertionError("Timed out waiting for a fresh simulator state")


def read_json_lines(path):
    if not path.exists():
        return []

    records = []

    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        if not line.strip():
            continue

        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            raise AssertionError(f"Invalid JSON at {path}:{line_number}") from error

    return records


def require_finite(value, description):
    require(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value),
        f"{description} must be a finite number, received {value!r}",
    )


def validate_observations(observations, allow_fire):
    require(len(observations) >= 10, "Expected at least 10 processed observations")

    position_errors = []
    yaw_errors = []
    tracking_count = 0
    active_plan_count = 0

    for observation in observations:
        state = observation["state"]
        result = observation["result"]
        plan = result["plan"]

        source_sim_time = result["source_sim_time"]
        planning_sim_time = result["planning_sim_time"]

        require_finite(state["sim_time"], "State simulation time")
        require_finite(source_sim_time, "Source simulation time")
        require_finite(planning_sim_time, "Planning simulation time")

        require(
            math.isclose(
                source_sim_time,
                state["sim_time"],
                rel_tol=0.0,
                abs_tol=1e-9,
            ),
            "Result source_sim_time does not identify its nested source state",
        )

        planning_lead = planning_sim_time - source_sim_time

        require(
            planning_lead >= -1e-9,
            "Planning simulation time precedes its source observation: "
            f"lead={planning_lead:.6f} s",
        )
        require(
            planning_lead <= MAX_PLANNING_LEAD_SECONDS,
            "Planning simulation time is more than the 250 ms simulator control "
            f"watchdog ahead of its source observation: lead={planning_lead:.6f} s",
        )

        if not allow_fire:
            require(
                state["stats"]["fired"] == 0,
                "A bridge without --allow-fire must never fire",
            )

        for field in ("yaw", "pitch", "yaw_vel", "pitch_vel", "yaw_acc", "pitch_acc"):
            require_finite(plan[field], f"Planner {field}")

        if result["tracker_state"] == "tracking":
            tracking_count += 1

        if plan["control"]:
            active_plan_count += 1

        for armor in result["armors"]:
            position_error = armor["position_error_m"]
            yaw_error = armor["yaw_error_rad"]

            require_finite(position_error, "PnP position error")
            require_finite(yaw_error, "PnP yaw error")

            for coordinate in armor["position_world"]:
                require_finite(coordinate, "PnP position coordinate")

            position_errors.append(position_error)
            yaw_errors.append(abs(yaw_error))

    require(len(position_errors) >= 10, "Expected repeated solved armor diagnostics")
    require(tracking_count >= 5, "The tracker did not repeatedly confirm the target")
    require(active_plan_count >= 5, "The planner did not produce active aiming plans")

    maximum_position_error = max(position_errors)
    maximum_yaw_error = max(yaw_errors)

    require(
        maximum_position_error < MAX_PNP_POSITION_ERROR_METERS,
        f"PnP position error exceeds 1 cm: {maximum_position_error:.6f} m",
    )
    require(
        maximum_yaw_error < MAX_PNP_YAW_ERROR_RADIANS,
        f"PnP yaw error exceeds 0.03 rad: {maximum_yaw_error:.6f} rad",
    )

    return maximum_position_error, maximum_yaw_error


def validate_raw_observations(observations):
    require(len(observations) >= 10, "Expected at least 10 rendered observations")

    detected_observation_count = 0
    target_detection_count = 0
    solved_armor_count = 0
    tracking_count = 0
    active_plan_count = 0
    corner_errors = []

    for observation in observations:
        frame = observation["state"]
        result = observation["result"]

        require(frame["type"] == "frame", "Raw observations must retain frame metadata")
        require(frame.get("request_id"), "Raw observations must identify their frame request")
        require(result["input"] == "raw-frame", "Raw result recorded the wrong input mode")
        require_finite(result["detector_ms"], "Detector latency")

        detections = result["detections"]

        if detections:
            detected_observation_count += 1

        solved_armor_count += result["solved_armors"]

        if result["tracker_state"] == "tracking":
            tracking_count += 1

        if result["plan"]["control"]:
            active_plan_count += 1

        ideal_armors = frame["camera"].get("armors", [])

        for detection in detections:
            if detection["color"] != frame["appearance"]["team"]:
                continue

            target_detection_count += 1

            detector_corners = detection["traditional_corners"]

            if len(detector_corners) != 4:
                detector_corners = detection["corners"]

            require(len(detector_corners) == 4, "Detector must produce four PnP corners")

            for point in detector_corners:
                require_finite(point[0], "Detector corner x")
                require_finite(point[1], "Detector corner y")

            if not ideal_armors:
                continue

            detector_center = (
                sum(point[0] for point in detector_corners) / 4.0,
                sum(point[1] for point in detector_corners) / 4.0,
            )

            nearest_ideal = min(
                ideal_armors,
                key=lambda armor: math.hypot(
                    detector_center[0]
                    - sum(point[0] for point in armor["corners"]) / 4.0,
                    detector_center[1]
                    - sum(point[1] for point in armor["corners"]) / 4.0,
                ),
            )

            corner_errors.extend(
                math.hypot(
                    detected_point[0] - ideal_point[0],
                    detected_point[1] - ideal_point[1],
                )
                for detected_point, ideal_point in zip(
                    detector_corners,
                    nearest_ideal["corners"],
                )
            )

        for armor in result["armors"]:
            for coordinate in armor["position_world"]:
                require_finite(coordinate, "Rendered-frame PnP position")

            require_finite(armor["yaw_world"], "Rendered-frame PnP yaw")

    require(
        detected_observation_count >= 10,
        "YOLO did not repeatedly detect the rendered armor",
    )
    require(
        target_detection_count >= 10,
        "YOLO did not repeatedly classify the target color",
    )
    require(solved_armor_count >= 10, "Rendered detections did not reach PnP")
    require(tracking_count >= 5, "Rendered detections did not confirm tracking")
    require(active_plan_count >= 5, "Rendered detections did not produce aiming plans")
    require(corner_errors, "No rendered detections could be matched to diagnostic ideal corners")

    maximum_corner_error = max(corner_errors)
    require(
        maximum_corner_error < MAX_RAW_CORNER_ERROR_PIXELS,
        "Detector corner error exceeds the rendered-frame limit: "
        f"{maximum_corner_error:.3f} px",
    )

    return maximum_corner_error


def validate_dry_run(initial_state, final_state, ipc_directory, commands):
    require(not commands, "Dry-run mode must not log or publish commands")
    require(
        not (ipc_directory / "command.json").exists(),
        "Dry-run mode created command.json, possibly during client destruction",
    )
    require(
        final_state["ipc"]["received"] == initial_state["ipc"]["received"],
        "Dry-run mode consumed control commands",
    )
    require(
        final_state["ipc"]["last_seq"] == initial_state["ipc"]["last_seq"],
        "Dry-run mode changed the accepted command sequence",
    )

    for angle in ("yaw", "pitch"):
        require(
            abs(final_state["gimbal"][angle] - initial_state["gimbal"][angle]) < 1e-8,
            f"Dry-run mode moved the gimbal {angle}",
        )


def validate_control(
    initial_state,
    final_state,
    observations,
    commands,
    allow_fire,
    allow_tracking_interruptions=False,
):
    require(
        final_state["ipc"]["received"] > initial_state["ipc"]["received"],
        "The simulator did not consume bridge commands",
    )
    require(
        final_state["ipc"]["rejected"] == initial_state["ipc"]["rejected"],
        "The simulator rejected a bridge command",
    )

    first_active_index = next(
        (
            command_index
            for command_index, command in enumerate(commands)
            if command["control"]
        ),
        None,
    )

    require(first_active_index is not None, "The bridge never enabled active control")

    continuous_active_commands = commands[first_active_index:]

    require(
        len(continuous_active_commands) >= 10,
        "Expected at least 10 active commands after target acquisition",
    )

    inactive_command_indices = [
        command_index
        for command_index, command in enumerate(
            continuous_active_commands,
            start=first_active_index,
        )
        if not command["control"]
    ]

    if not allow_tracking_interruptions:
        require(
            not inactive_command_indices,
            "Active control was interrupted after target acquisition at command indices "
            + ", ".join(str(index) for index in inactive_command_indices[:10]),
        )

    active_commands = [command for command in commands if command["control"]]
    require(len(active_commands) >= 10, "Expected at least 10 active aiming commands")

    command_sequences = []
    fired_command_count = 0

    for command in commands:
        published_fire = command.get("fire")
        planner_fire_intent = command.get("planner_fire_intent")

        require(
            isinstance(published_fire, bool),
            f"Published fire must be boolean, received {published_fire!r}",
        )
        require(
            isinstance(planner_fire_intent, bool),
            "Planner fire intent must be logged as a boolean",
        )

        expected_fire = (
            allow_fire
            and command["control"]
            and command["tracker_state"] == "tracking"
            and planner_fire_intent
        )

        require(
            published_fire == expected_fire,
            "Published fire does not match the explicit permission and safety gates",
        )

        if published_fire:
            fired_command_count += 1

        sequence = command["seq"]
        require(
            isinstance(sequence, int) and not isinstance(sequence, bool),
            f"Command sequence must be an integer, received {sequence!r}",
        )
        command_sequences.append(sequence)

        for field in ("yaw", "pitch", "yaw_vel", "pitch_vel", "yaw_acc", "pitch_acc"):
            require_finite(command[field], f"Command {field}")

        if command["control"]:
            require_finite(command["source_sim_time"], "Command source simulation time")
            require_finite(command["planning_sim_time"], "Command planning simulation time")
            require_finite(command["state_sim_time"], "Command state simulation time")

            planning_lead = command["planning_sim_time"] - command["source_sim_time"]
            planning_age = command["state_sim_time"] - command["planning_sim_time"]

            require(
                -1e-9 <= planning_lead <= MAX_PLANNING_LEAD_SECONDS,
                "Active command planning time is inconsistent with its source: "
                f"lead={planning_lead:.6f} s",
            )
            require(
                -1e-9 <= planning_age <= MAX_ACTIVE_COMMAND_PLANNING_AGE_SECONDS,
                "Active command used a stale or future plan: "
                f"age={planning_age:.6f} s",
            )

    require(
        all(
            current_sequence > previous_sequence
            for previous_sequence, current_sequence in zip(
                command_sequences,
                command_sequences[1:],
            )
        ),
        "Published command sequences must be strictly increasing",
    )

    if allow_fire:
        require(fired_command_count >= 3, "Expected repeated permitted firing commands")
    else:
        require(fired_command_count == 0, "Control-only mode unexpectedly permitted firing")

    final_result = observations[-1]["result"]

    if not allow_tracking_interruptions:
        require(
            final_result["tracker_state"] == "tracking",
            "The target was not in tracking state at the final observation",
        )
        require(
            final_result["plan"]["control"],
            "The final target plan did not request active control",
        )

    tracking_error_limits = {
        "yaw": MAX_ROTATING_YAW_TRACKING_ERROR_RADIANS,
        "pitch": MAX_ROTATING_PITCH_TRACKING_ERROR_RADIANS,
    }

    for angle, maximum_tracking_error in tracking_error_limits.items():
        recent_command_goal = statistics.median(
            command[angle]
            for command in active_commands[-10:]
        )
        actual_angle = final_state["gimbal"][angle]
        tracking_error = abs(
            math.remainder(
                actual_angle - recent_command_goal,
                2.0 * math.pi,
            )
        )

        require(
            tracking_error < maximum_tracking_error,
            f"Actual {angle} did not follow the rotating command: "
            f"error={tracking_error:.6f} rad, limit={maximum_tracking_error:.6f} rad",
        )

    yaw_displacement = final_state["gimbal"]["yaw"] - initial_state["gimbal"]["yaw"]
    require(yaw_displacement > 0.03, "The offset target must cause positive yaw movement")


def make_simulator_config(repository, channel, rotate_target, allow_fire):
    configuration = (repository / "configs" / "default.cfg").read_text()
    configuration, channel_replacements = re.subn(
        r'^channel="default"$',
        f'channel="{channel}"',
        configuration,
        count=1,
        flags=re.MULTILINE,
    )
    configuration, center_replacements = re.subn(
        r"^center=Vector3\([^\n]*\)$",
        "center=Vector3(4, 0.6, 0.30)",
        configuration,
        count=1,
        flags=re.MULTILINE,
    )

    require(channel_replacements == 1, "Default config is missing its IPC channel")
    require(center_replacements == 1, "Default config is missing its target center")

    if allow_fire:
        configuration, health_replacements = re.subn(
            r"^max_health=[^\n]*$",
            "max_health=10000",
            configuration,
            count=1,
            flags=re.MULTILINE,
        )

        require(
            health_replacements == 1,
            "Default config is missing its target health setting",
        )

    rotation_value = "true" if rotate_target else "false"
    configuration, rotation_replacements = re.subn(
        r"^rotate=[^\n]*$",
        f"rotate={rotation_value}",
        configuration,
        count=1,
        flags=re.MULTILINE,
    )

    require(
        rotation_replacements == 1,
        "Default config is missing its target rotation setting",
    )

    if rotate_target:
        configuration, angular_speed_replacements = re.subn(
            r"^omega=[^\n]*$",
            "omega=1.0",
            configuration,
            count=1,
            flags=re.MULTILINE,
        )

        require(
            angular_speed_replacements == 1,
            "Default config is missing its target angular speed",
        )

    return configuration


def stop_process(process):
    if process is None or process.poll() is not None:
        return

    process.terminate()

    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def log_tail(path):
    if not path.exists():
        return "(no log was created)"

    return "\n".join(path.read_text(errors="replace").splitlines()[-60:])


def run_scenario(
    repository,
    godot,
    bridge,
    workspace,
    enable_control,
    allow_fire=False,
    input_mode="exact-corners",
):
    require(
        input_mode in ("exact-corners", "raw-frame"),
        f"Unsupported smoke-test input mode: {input_mode}",
    )

    rotate_target = (
        input_mode == "exact-corners"
        and enable_control
        and not allow_fire
    )

    if allow_fire:
        scenario_name = "static_fire"
    elif rotate_target:
        scenario_name = "rotating_control"
    elif enable_control:
        scenario_name = "static_control"
    else:
        scenario_name = "static_dry_run"

    scenario_name = input_mode.replace("-", "_") + "_" + scenario_name
    scenario_directory = workspace / scenario_name
    scenario_directory.mkdir()

    channel = "bridge_smoke_" + uuid.uuid4().hex
    ipc_directory = Path("/dev/shm") / ("rm_aim_sim_" + channel)
    state_path = ipc_directory / "state.json"

    config_path = scenario_directory / "simulator.cfg"
    config_path.write_text(
        make_simulator_config(
            repository,
            channel,
            rotate_target=rotate_target,
            allow_fire=allow_fire,
        )
    )

    output_directory = scenario_directory / "bridge_output"
    simulator_log_path = scenario_directory / "godot.log"
    bridge_log_path = scenario_directory / "bridge.log"

    simulator = None
    bridge_process = None

    try:
        with (
            simulator_log_path.open("w") as simulator_log,
            bridge_log_path.open("w") as bridge_log,
        ):
            simulator_arguments = [str(godot)]

            if input_mode == "exact-corners":
                simulator_arguments.append("--headless")

            simulator_arguments.extend(
                [
                    "--path",
                    str(repository),
                    "--",
                    "--remote",
                    "--config=" + str(config_path),
                ]
            )

            simulator = subprocess.Popen(
                simulator_arguments,
                cwd=repository,
                stdout=simulator_log,
                stderr=subprocess.STDOUT,
                shell=False,
            )

            initial_state = wait_for_state(state_path, simulator)
            require(initial_state["ipc"]["enabled"], "External control must be enabled")
            require(initial_state["camera"]["armors"], "The target must initially be visible")

            bridge_arguments = [
                str(bridge),
                "--channel=" + channel,
                "--duration=" + ("7" if input_mode == "raw-frame" else "4"),
                "--input=" + input_mode,
                "--output=" + str(output_directory),
            ]

            if enable_control:
                bridge_arguments.append("--control")

            if allow_fire:
                bridge_arguments.append("--allow-fire")

            bridge_process = subprocess.Popen(
                bridge_arguments,
                cwd=scenario_directory,
                stdout=bridge_log,
                stderr=subprocess.STDOUT,
                shell=False,
            )
            bridge_process.wait(timeout=20)

            require(bridge_process.returncode == 0, "The bridge exited unsuccessfully")

            final_state = wait_for_state(state_path, simulator)
            final_state = wait_for_state(
                state_path,
                simulator,
                after_publication=final_state["publication"],
            )

            observations = read_json_lines(output_directory / "observations.jsonl")
            commands = read_json_lines(output_directory / "commands.jsonl")
            if input_mode == "raw-frame":
                maximum_corner_error = validate_raw_observations(observations)
                quality_summary = (
                    f"max detector corner error={maximum_corner_error:.3f} px"
                )
            else:
                position_error, yaw_error = validate_observations(
                    observations,
                    allow_fire=allow_fire,
                )
                quality_summary = (
                    f"max PnP position error={position_error:.6f} m, "
                    f"max yaw error={yaw_error:.6f} rad"
                )

            if allow_fire:
                require(final_state["stats"]["fired"] > 0, "No projectile was fired")
                require(final_state["stats"]["hit"] > 0, "No projectile hit the target")
                require(
                    final_state["health"]["current"] < final_state["health"]["maximum"],
                    "Successful hits did not reduce target health",
                )
            else:
                require(final_state["stats"]["fired"] == 0, "The bridge unexpectedly fired")

            require(final_state["stats"]["pending"] == 0, "A pending shot remained after stop")

            if enable_control:
                validate_control(
                    initial_state,
                    final_state,
                    observations,
                    commands,
                    allow_fire=allow_fire,
                    allow_tracking_interruptions=input_mode == "raw-frame",
                )
            else:
                validate_dry_run(initial_state, final_state, ipc_directory, commands)

            print(
                f"PASS: {scenario_name}, {len(observations)} observations, "
                f"{quality_summary}, "
                f"fired={final_state['stats']['fired']}, "
                f"hit={final_state['stats']['hit']}"
            )
    except Exception as error:
        raise AssertionError(
            f"{scenario_name} failed: {error}\n"
            f"Bridge log:\n{log_tail(bridge_log_path)}\n"
            f"Simulator log:\n{log_tail(simulator_log_path)}"
        ) from error
    finally:
        stop_process(bridge_process)
        stop_process(simulator)

        # Both subprocesses have exited; this unique test-owned channel is disposable.
        if ipc_directory.exists():
            shutil.rmtree(ipc_directory)

    simulator_output = simulator_log_path.read_text(errors="replace")
    require("SCRIPT ERROR" not in simulator_output, simulator_output)
    require("ERROR:" not in simulator_output, simulator_output)


def main():
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("--godot", required=True, type=Path)
    argument_parser.add_argument("--bridge", required=True, type=Path)
    argument_parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="Simulator repository root; inferred from this script by default",
    )
    argument_parser.add_argument(
        "--suite",
        choices=("exact", "raw", "all"),
        default="exact",
        help="exact uses headless mode; raw requires a graphical display",
    )
    arguments = argument_parser.parse_args()

    repository = arguments.root.resolve()
    godot = arguments.godot.resolve()
    bridge = arguments.bridge.resolve()

    require(godot.is_file(), f"Godot executable does not exist: {godot}")
    require(bridge.is_file(), f"Bridge executable does not exist: {bridge}")

    if arguments.suite in ("raw", "all"):
        require(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"),
            "The raw smoke suite needs DISPLAY or WAYLAND_DISPLAY; use a graphical session",
        )

    with tempfile.TemporaryDirectory(prefix="rm_aim_bridge_smoke_") as temporary_directory:
        workspace = Path(temporary_directory)

        if arguments.suite in ("exact", "all"):
            run_scenario(repository, godot, bridge, workspace, enable_control=False)
            run_scenario(repository, godot, bridge, workspace, enable_control=True)
            run_scenario(
                repository,
                godot,
                bridge,
                workspace,
                enable_control=True,
                allow_fire=True,
            )

        if arguments.suite in ("raw", "all"):
            run_scenario(
                repository,
                godot,
                bridge,
                workspace,
                enable_control=False,
                input_mode="raw-frame",
            )
            run_scenario(
                repository,
                godot,
                bridge,
                workspace,
                enable_control=True,
                input_mode="raw-frame",
            )


if __name__ == "__main__":
    main()
