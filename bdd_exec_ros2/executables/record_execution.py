import argparse
import signal
import subprocess
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

import yaml


def load_recording(
    config_path: Path, execution: str, run_id: str
) -> tuple[Path, list[str]]:
    with config_path.open() as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("recording config must use schema_version: 1")
    try:
        execution_config = config["executions"][execution]
        output_root = Path(config["output_root"]).expanduser()
        bag_path = Path(execution_config["bag_path"].format(run_id=run_id))
        topics = execution_config["topics"]
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"invalid recording config: {error}") from error

    if bag_path.is_absolute() or ".." in bag_path.parts:
        raise ValueError("bag_path must be relative to output_root")
    if (
        not isinstance(topics, list)
        or not topics
        or not all(isinstance(topic, str) and topic.startswith("/") for topic in topics)
    ):
        raise ValueError("topics must be a non-empty list of absolute ROS topic names")
    return output_root / bag_path, topics


def _stop(recorder: subprocess.Popen) -> None:
    if recorder.poll() is not None:
        return
    recorder.send_signal(signal.SIGINT)
    try:
        recorder.wait(timeout=10)
    except subprocess.TimeoutExpired:
        recorder.terminate()
        recorder.wait(timeout=5)


def main(args=None) -> int:
    parser = argparse.ArgumentParser(
        description="Record a configured execution and trigger /bdd/start."
    )
    parser.add_argument("config", type=Path)
    parser.add_argument("execution", help="Execution URI from the recording config")
    parser.add_argument(
        "--run-id",
        default=datetime.now(UTC).strftime("run-%Y%m%dT%H%M%S%fZ"),
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.0,
        help="Seconds to let rosbag discovery settle before triggering the test",
    )
    options = parser.parse_args(args)

    try:
        bag_path, topics = load_recording(
            options.config, options.execution, options.run_id
        )
        if bag_path.exists():
            raise ValueError(f"bag path already exists: {bag_path}")
        bag_path.parent.mkdir(parents=True, exist_ok=True)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    recorder = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(bag_path), *topics],
        start_new_session=True,
    )
    try:
        time.sleep(max(0.0, options.startup_delay))
        if recorder.poll() is not None:
            raise RuntimeError(
                f"ros2 bag record exited with code {recorder.returncode}"
            )
        subprocess.run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/bdd/start",
                "std_msgs/msg/Empty",
                "{}",
            ],
            check=True,
        )
        print(f"Recording to {bag_path}; press Ctrl-C to stop.")
        return recorder.wait()
    except KeyboardInterrupt:
        return 0
    finally:
        _stop(recorder)


if __name__ == "__main__":
    sys.exit(main())
