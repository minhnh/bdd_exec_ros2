import argparse
import signal
import subprocess
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

import yaml


def load_recording(config_path: Path, set_name: str) -> tuple[str, list[str]]:
    with config_path.open() as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("recording config must use schema_version: 1")
    try:
        trigger_topic = config["trigger_topic"]
        topics = config["topic_sets"][set_name]
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"invalid recording config: {error}") from error

    if not isinstance(trigger_topic, str) or not trigger_topic.startswith("/"):
        raise ValueError("trigger_topic must be an absolute ROS topic name")
    if (
        not isinstance(topics, list)
        or not topics
        or not all(isinstance(topic, str) and topic.startswith("/") for topic in topics)
    ):
        raise ValueError("topics must be a non-empty list of absolute ROS topic names")
    return trigger_topic, topics


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
        description="Record a configured execution and trigger the BDD test coordinator to start.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("config", type=Path)
    parser.add_argument("set_name", help="Name of the set of topics to record")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path.cwd(),
        help="Parent directory for the bag",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.0,
        help="Seconds to let rosbag discovery settle before triggering the test",
    )
    parser.add_argument(
        "--listener-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for subscription from the test coordinator on the start topic",
    )
    options = parser.parse_args(args)
    if not options.set_name or Path(options.set_name).name != options.set_name:
        parser.error("set-name must be a non-empty directory name")
    run_id = f"{options.set_name}-{datetime.now(UTC):%Y%m%dT%H%M%S%fZ}"

    try:
        trigger_topic, topics = load_recording(options.config, options.set_name)
        bag_path = options.output_dir.expanduser() / run_id
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
                "--max-wait-time-secs",
                str(max(0.0, options.listener_timeout)),
                trigger_topic,
                "std_msgs/msg/Empty",
                "{}",
            ],
            check=True,
        )
        print(f"Recording to {bag_path}; press Ctrl-C to stop.")
        return recorder.wait()
    except subprocess.CalledProcessError as error:
        print(
            f"Could not trigger {trigger_topic}; is the coordinator running?",
            file=sys.stderr,
        )
        return error.returncode or 1
    except KeyboardInterrupt:
        return 0
    finally:
        _stop(recorder)


if __name__ == "__main__":
    sys.exit(main())
