#!/usr/bin/env python3

# Copyright 2026 Minh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import signal
import argparse
import uuid

from PySide6.QtCore import QThread, Signal, Slot
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QApplication,
    QHeaderView,
    QLabel,
    QMainWindow,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

import rclpy

from bdd_ros2_interfaces.msg import ScenarioStatusList, Trinary as TrinaryMsg

from bdd_exec_ros2.conversions import format_time_msg, from_uuid_msg


class RosWorker(QThread):
    message_received = Signal(object)

    def __init__(self, topic_name, context_args=None):
        super().__init__()
        self.topic_name = topic_name
        self.context_args = context_args  # Args passed to rclpy.init
        self._node = None

    def run(self):
        # Initialize ROS with any extra args (e.g. --ros-args)
        if not rclpy.ok():
            rclpy.init(args=self.context_args)

        # Create a unique node name to allow running multiple viz instances
        node_name = f"bdd_viz_{uuid.uuid4().hex[:8]}"
        self._node = rclpy.create_node(node_name)
        self._node.get_logger().info(f"Subscribing to: {self.topic_name}")

        self._sub = self._node.create_subscription(
            ScenarioStatusList, self.topic_name, self._callback, 10
        )

        try:
            rclpy.spin(self._node)
        except Exception:
            pass
        finally:
            self.stop()

    def _callback(self, msg):
        self.message_received.emit(msg)

    def stop(self):
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class BddVisualizer(QMainWindow):
    def __init__(self, status_topic: str, width: int, height: int, ros_args):
        super().__init__()
        self.setWindowTitle("BDD Dashboard")
        self.resize(width, height)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.lbl_status = QLabel(f"Listening on {status_topic}...")
        layout.addWidget(self.lbl_status)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(
            ["Context / Fluent", "State", "Time", "Oracle Details"]
        )
        header = self.tree.header()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        layout.addWidget(self.tree)

        self._scenario_items = {}

        self.ros_thread = RosWorker(status_topic, ros_args)
        self.ros_thread.message_received.connect(self.update_ui)
        self.ros_thread.start()

    @Slot(object)
    def update_ui(self, msg: ScenarioStatusList):
        # Update timestamp label
        if hasattr(msg, "header"):
            self.lbl_status.setText(f"Last Update: {format_time_msg(msg.stamp)}")

        active_uuids = set()

        for scenario in msg.scenarios:
            ctx_id = from_uuid_msg(scenario.context_id)
            active_uuids.add(ctx_id)

            if ctx_id not in self._scenario_items:
                scr_item = QTreeWidgetItem(self.tree)
                scr_item.setText(0, f"Scenario: {str(ctx_id.hex[:8])}...")
                scr_item.setExpanded(True)
                f = scr_item.font(0)
                f.setBold(True)
                scr_item.setFont(0, f)
                self._scenario_items[ctx_id] = {"item": scr_item, "children": {}}

            scr_data = self._scenario_items[ctx_id]
            scr_item = scr_data["item"]

            # Scenario Result
            txt, color = self._get_trinary_style(scenario.result.trinary.value)
            scr_item.setText(1, txt)
            scr_item.setForeground(1, QBrush(color))
            scr_item.setText(2, format_time_msg(scenario.result.stamp))

            for fluent in scenario.fluents:
                f_name = fluent.representation
                if f_name not in scr_data["children"]:
                    fl_item = QTreeWidgetItem(scr_item)
                    fl_item.setText(0, f_name)
                    scr_data["children"][f_name] = fl_item

                fl_item = scr_data["children"][f_name]
                trin_val = fluent.result.trinary.value
                txt, color = self._get_trinary_style(trin_val)

                fl_item.setText(1, txt)
                fl_item.setForeground(1, QBrush(color))
                fl_item.setText(2, format_time_msg(fluent.result.stamp))

                history_list = fluent.trinaries
                current_count = len(history_list)
                displayed_count = fl_item.childCount()

                fl_item.setText(3, f"Updates: {current_count}")

                # Only append NEW items to avoid rebuilding tree every frame
                if current_count > displayed_count:
                    for i in range(displayed_count, current_count):
                        hist_msg = history_list[i]

                        # Create the history row
                        h_item = QTreeWidgetItem(fl_item)
                        h_item.setText(0, f"Change #{i + 1}")

                        # Apply Color based on THIS specific history item's value
                        h_val = hist_msg.trinary.value
                        h_txt, h_color = self._get_trinary_style(h_val)
                        h_item.setText(1, h_txt)
                        h_item.setForeground(1, QBrush(h_color))
                        h_item.setText(2, format_time_msg(hist_msg.stamp))

                        # Make the index label gray/subtle
                        h_item.setForeground(0, QBrush(QColor("gray")))

        # Cleanup
        for old_id in list(self._scenario_items.keys()):
            if old_id not in active_uuids:
                item = self._scenario_items[old_id]["item"]
                item.setForeground(0, QBrush(QColor("gray")))
                item.setText(1, "FINISHED")

    def _get_trinary_style(self, value):
        if value == TrinaryMsg.TRUE:
            return "TRUE", QColor("darkgreen")
        elif value == TrinaryMsg.FALSE:
            return "FALSE", QColor("red")
        elif value == TrinaryMsg.UNKNOWN:
            return "UNKNOWN", QColor("darkorange")
        else:
            raise ValueError(f"Invalid trinary value: {value}")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="BDD Visualization Tool",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "-t", "--topic", type=str, default="/bdd/status", help="BDD test status topic"
    )
    parser.add_argument("--width", type=int, default=1000, help="Window width")
    parser.add_argument("--height", type=int, default=600, help="Window height")

    # Parse Known Args
    # We use parse_known_args() so that if the user passes ROS flags (like --ros-args),
    # argparse won't throw an error. It puts unknown flags into 'ros_args'.
    args, ros_args = parser.parse_known_args()

    # Handle Signals
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Launch App
    app = QApplication(sys.argv)
    window = BddVisualizer(
        status_topic=args.topic, width=args.width, height=args.height, ros_args=ros_args
    )
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
