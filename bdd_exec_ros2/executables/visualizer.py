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
from enum import Enum, auto

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
from rclpy.time import Time

from bdd_ros2_interfaces.msg import (
    FluentStatus,
    ScenarioStatus,
    ScenarioStatusList,
    Trinary as TrinaryMsg,
    TrinaryStamped,
)
from bdd_exec_ros2.conversions import format_time_msg, from_uuid_msg


class ColumnIdx(Enum):
    SCENARIO_FLUENT = 0
    RESULT = auto()
    RESULT_TIME = auto()
    START_TIME = auto()
    END_TIME = auto()
    DETAILS = auto()


COLUMN_NAMES = {
    ColumnIdx.SCENARIO_FLUENT: "Scenario/Fluent",
    ColumnIdx.RESULT: "Result",
    ColumnIdx.RESULT_TIME: "Result Time",
    ColumnIdx.START_TIME: "Start Time",
    ColumnIdx.END_TIME: "End Time",
    ColumnIdx.DETAILS: "Oracle Details",
}


def get_trinary_style(value) -> tuple[str, QColor]:
    if value == TrinaryMsg.TRUE:
        return "TRUE", QColor("darkgreen")
    elif value == TrinaryMsg.FALSE:
        return "FALSE", QColor("red")
    elif value == TrinaryMsg.UNKNOWN:
        return "UNKNOWN", QColor("darkorange")
    else:
        raise ValueError(f"Invalid trinary value: {value}")


def create_new_scr_item(parent: QTreeWidget, ctx_id: uuid.UUID) -> QTreeWidgetItem:
    scr_item = QTreeWidgetItem(parent)

    scr_item.setText(
        ColumnIdx.SCENARIO_FLUENT.value, f"Scenario: {str(ctx_id.hex[:8])}..."
    )
    scr_item.setExpanded(True)
    f = scr_item.font(ColumnIdx.SCENARIO_FLUENT.value)
    f.setBold(True)
    scr_item.setFont(ColumnIdx.SCENARIO_FLUENT.value, f)

    return scr_item


def update_scr_item_view(scr_item: QTreeWidgetItem, scr_status: ScenarioStatus) -> bool:
    # result
    txt, color = get_trinary_style(scr_status.result.trinary.value)
    scr_item.setText(ColumnIdx.RESULT.value, txt)
    scr_item.setForeground(ColumnIdx.RESULT.value, QBrush(color))
    scr_item.setText(
        ColumnIdx.RESULT_TIME.value, format_time_msg(scr_status.result.stamp)
    )

    # start/end time
    if scr_status.start_time.sec > 0:
        scr_item.setText(
            ColumnIdx.START_TIME.value, format_time_msg(scr_status.start_time)
        )

    finished = False
    if scr_status.end_time.sec > 0:
        scr_item.setText(ColumnIdx.END_TIME.value, format_time_msg(scr_status.end_time))
        finished = True

    return finished


def create_new_fluent_item(scr_item: QTreeWidgetItem, rep: str):
    fl_item = QTreeWidgetItem(scr_item)
    fl_item.setText(ColumnIdx.SCENARIO_FLUENT.value, rep)
    return fl_item


def update_fluent_item_view(fl_item: QTreeWidgetItem, fl_status: FluentStatus):
    fl_item.setText(
        ColumnIdx.DETAILS.value, f"Trinary count: {len(fl_status.trinaries)}"
    )

    # Result
    trin_val = fl_status.result.trinary.value
    txt, color = get_trinary_style(trin_val)

    fl_item.setText(ColumnIdx.RESULT.value, txt)
    fl_item.setForeground(ColumnIdx.RESULT.value, QBrush(color))
    fl_item.setText(
        ColumnIdx.RESULT_TIME.value, format_time_msg(fl_status.result.stamp)
    )

    # start/end time
    if fl_status.start_time.sec > 0:
        fl_item.setText(
            ColumnIdx.START_TIME.value, format_time_msg(fl_status.start_time)
        )

    if fl_status.end_time.sec > 0:
        fl_item.setText(ColumnIdx.END_TIME.value, format_time_msg(fl_status.end_time))


def create_new_trin_item(
    fl_item: QTreeWidgetItem, trin_id: int, trin_msg: TrinaryStamped
) -> QTreeWidgetItem:
    trin_item = QTreeWidgetItem(fl_item)
    trin_item.setText(ColumnIdx.SCENARIO_FLUENT.value, f"Change #{trin_id}")

    # Trinary value
    h_val = trin_msg.trinary.value
    h_txt, h_color = get_trinary_style(h_val)
    trin_item.setText(ColumnIdx.RESULT.value, h_txt)
    trin_item.setForeground(ColumnIdx.RESULT.value, QBrush(h_color))
    trin_item.setText(ColumnIdx.RESULT_TIME.value, format_time_msg(trin_msg.stamp))

    return trin_item


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
        self.tree.setHeaderLabels([COLUMN_NAMES[cl_idx] for cl_idx in ColumnIdx])
        header = self.tree.header()
        header.setSectionResizeMode(
            ColumnIdx.SCENARIO_FLUENT.value, QHeaderView.Stretch
        )
        header.setSectionResizeMode(ColumnIdx.DETAILS.value, QHeaderView.Stretch)
        header.setSectionResizeMode(
            ColumnIdx.RESULT.value, QHeaderView.ResizeToContents
        )
        header.setSectionResizeMode(
            ColumnIdx.RESULT_TIME.value, QHeaderView.ResizeToContents
        )
        header.setSectionResizeMode(
            ColumnIdx.START_TIME.value, QHeaderView.ResizeToContents
        )
        header.setSectionResizeMode(
            ColumnIdx.END_TIME.value, QHeaderView.ResizeToContents
        )
        layout.addWidget(self.tree)

        self._scenario_items = {}

        self.ros_thread = RosWorker(status_topic, ros_args)
        self.ros_thread.message_received.connect(self.update_ui)
        self.ros_thread.start()

    @Slot(object)
    def update_ui(self, msg: ScenarioStatusList):
        # Update timestamp label
        self.lbl_status.setText(f"Last Update: {format_time_msg(msg.stamp)}")

        cleanup_set = set()

        for scr_status in msg.scenarios:
            ctx_id = from_uuid_msg(scr_status.context_id)

            if ctx_id not in self._scenario_items:
                scr_item = create_new_scr_item(parent=self.tree, ctx_id=ctx_id)
                self._scenario_items[ctx_id] = {
                    "item": scr_item,
                    "children": {},
                    "finished": False,
                }

            scr_data = self._scenario_items[ctx_id]
            scr_item = scr_data["item"]
            scr_finished = scr_data["finished"]

            if scr_finished:
                continue

            # update scenario item
            scr_finished = update_scr_item_view(
                scr_item=scr_item, scr_status=scr_status
            )
            if scr_finished:
                self._scenario_items[ctx_id]["finished"] = True
                cleanup_set.add(ctx_id)

            for fl_status in scr_status.fluents:
                f_rep = fl_status.representation
                if f_rep not in scr_data["children"]:
                    fl_item = create_new_fluent_item(scr_item=scr_item, rep=f_rep)
                    scr_data["children"][f_rep] = {"item": fl_item, "children": {}}

                fl_data = scr_data["children"][f_rep]
                fl_item = fl_data["item"]

                update_fluent_item_view(fl_item=fl_item, fl_status=fl_status)

                active_trins = set()
                for trin_msg in fl_status.trinaries:
                    trin_t = Time.from_msg(trin_msg.stamp).to_datetime().timestamp()
                    active_trins.add(trin_t)
                    if trin_t in fl_data["children"]:
                        # assuming pass trin won't change
                        continue

                    trin_id = len(fl_data["children"]) + 1
                    # Create new trinary entry
                    trin_item = create_new_trin_item(
                        fl_item=fl_item, trin_id=trin_id, trin_msg=trin_msg
                    )
                    fl_data["children"][trin_t] = {
                        "item": trin_item,
                        "discarded": False,
                    }

                # clean up discarded trinaries
                for trin_t, trin_data in fl_data["children"].items():
                    if trin_t in active_trins:
                        # still active
                        continue

                    if trin_data["discarded"]:
                        # already handled
                        continue

                    # Make the index label gray/subtle
                    trin_item = trin_data["item"]
                    trin_item_text = trin_item.text(ColumnIdx.SCENARIO_FLUENT.value)
                    trin_item.setText(
                        ColumnIdx.SCENARIO_FLUENT.value, f"{trin_item_text} (discarded)"
                    )
                    trin_item.setForeground(
                        ColumnIdx.SCENARIO_FLUENT.value, QBrush(QColor("gray"))
                    )
                    trin_item.setForeground(
                        ColumnIdx.RESULT.value, QBrush(QColor("gray"))
                    )

                    trin_data["discarded"] = True

        # Clean up completed scenarios
        for ctx_id in cleanup_set:
            item = self._scenario_items[ctx_id]["item"]
            item.setForeground(0, QBrush(QColor("gray")))


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
