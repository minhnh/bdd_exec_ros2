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
from typing import Optional
import uuid
from enum import Enum, auto

from PySide6.QtCore import QRect, QSize, Qt, QThread, Signal, Slot
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QApplication,
    QHeaderView,
    QLabel,
    QMainWindow,
    QStyledItemDelegate,
    QStyle,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.time import Time

from bdd_ros2_interfaces.msg import (
    BehaviourStatus,
    FluentStatus,
    ScenarioStatus,
    ScenarioStatusList,
    Trinary as TrinaryMsg,
    TrinaryStamped,
)
from bdd_exec_ros2.conversions import format_time_msg, from_uuid_msg, time_to_stamp


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

TRIN_COLORS = {
    TrinaryMsg.TRUE: ("TRUE", QColor("darkgreen")),
    TrinaryMsg.FALSE: ("FALSE", QColor("red")),
    TrinaryMsg.UNKNOWN: ("UNKNOWN", QColor("darkorange")),
}


def get_trinary_style(value) -> tuple[str, QColor]:
    if value not in TRIN_COLORS:
        raise ValueError(f"Invalid trinary value: {value}")

    return TRIN_COLORS[value]


def create_new_scr_item(
    parent: QTreeWidget, ctx_id: uuid.UUID, scr_rep: str
) -> QTreeWidgetItem:
    scr_item = QTreeWidgetItem(parent)

    scr_item.setText(
        ColumnIdx.SCENARIO_FLUENT.value,
        f"Scenario: {scr_rep} - {str(ctx_id.hex[:8])}...",
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


def create_new_clause_item(scr_item: QTreeWidgetItem, rep: str):
    fl_item = QTreeWidgetItem(scr_item)
    fl_item.setText(ColumnIdx.SCENARIO_FLUENT.value, rep)
    fl_item.setToolTip(ColumnIdx.SCENARIO_FLUENT.value, rep)
    return fl_item


def update_fluent_item_view(fl_item: QTreeWidgetItem, fl_status: FluentStatus):
    trinary_values = [t.trinary.value for t in fl_status.trinaries]
    fl_item.setData(ColumnIdx.DETAILS.value, Qt.UserRole, trinary_values)

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


def update_bhv_item_view(bhv_item: QTreeWidgetItem, bhv_status: BehaviourStatus):
    trin_val = bhv_status.result.trinary.value
    txt, color = get_trinary_style(trin_val)
    bhv_item.setText(ColumnIdx.RESULT.value, txt)
    bhv_item.setForeground(ColumnIdx.RESULT.value, QBrush(color))
    bhv_item.setText(
        ColumnIdx.RESULT_TIME.value, format_time_msg(bhv_status.result.stamp)
    )


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


class TrinaryHistoryDelegate(QStyledItemDelegate):
    _square_size: int
    _spacing: int

    def __init__(self, parent=None, font_size: Optional[int] = None):
        super().__init__(parent)
        if font_size is None:
            self._square_size = 12
        else:
            self._square_size = round(font_size * 1.2)
        self._spacing = round(self._square_size / 3)

    def paint(self, painter, option, index):
        # Retrieve the list of trinary values from the UserRole
        data = index.data(Qt.UserRole)

        if not isinstance(data, list):
            return super().paint(painter, option, index)

        painter.save()

        # Draw background selection highlight if item is selected
        if option.state & QStyle.State_Selected:
            painter.fillRect(option.rect, option.palette.highlight())

        # Square settings
        x = option.rect.x() + self._spacing * 2
        y = option.rect.y() + (option.rect.height() - self._square_size) // 2

        for val in data:
            _, color = get_trinary_style(val)
            painter.setBrush(color)
            painter.setPen(Qt.NoPen)

            # Draw the square
            painter.drawRect(QRect(x, y, self._square_size, self._square_size))
            x += self._square_size + self._spacing

            # Stop drawing if we go outside the column width
            if x > option.rect.right() - self._square_size:
                break

        painter.restore()

    def sizeHint(self, option, index):
        base_size = super().sizeHint(option, index)
        # Ensure the row is at least 20 pixels high for our squares
        return base_size.expandedTo(QSize(0, 20))


class BddVisualizer(QMainWindow):
    def __init__(
        self,
        status_topic: str,
        width: int,
        height: int,
        font_size: Optional[int],
        ros_args,
    ):
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
            ColumnIdx.SCENARIO_FLUENT.value, QHeaderView.Interactive
        )
        self.tree.setColumnWidth(ColumnIdx.SCENARIO_FLUENT.value, 600)
        header.setMinimumSectionSize(200)

        header.setSectionResizeMode(
            ColumnIdx.DETAILS.value, QHeaderView.ResizeToContents
        )
        self.tree.setItemDelegateForColumn(
            ColumnIdx.DETAILS.value,
            TrinaryHistoryDelegate(self.tree, font_size=font_size),
        )
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
                scr_item = create_new_scr_item(
                    parent=self.tree, ctx_id=ctx_id, scr_rep=scr_status.representation
                )
                bhv_item = create_new_clause_item(
                    scr_item=scr_item, rep=scr_status.behaviour.representation
                )
                self._scenario_items[ctx_id] = {
                    "item": scr_item,
                    "bhv_item": bhv_item,
                    "children": {},
                    "finished": False,
                }

            scr_data = self._scenario_items[ctx_id]
            scr_item = scr_data["item"]
            scr_finished = scr_data["finished"]

            if scr_finished:
                continue

            # update behaviour
            bhv_item = scr_data["bhv_item"]
            update_bhv_item_view(bhv_item=bhv_item, bhv_status=scr_status.behaviour)

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
                    fl_item = create_new_clause_item(scr_item=scr_item, rep=f_rep)
                    scr_data["children"][f_rep] = {"item": fl_item, "children": {}}

                fl_data = scr_data["children"][f_rep]
                fl_item = fl_data["item"]

                update_fluent_item_view(fl_item=fl_item, fl_status=fl_status)

                active_trins = set()
                for trin_msg in fl_status.trinaries:
                    trin_t = time_to_stamp(Time.from_msg(trin_msg.stamp))
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
    parser.add_argument(
        "--font-size", type=int, default=None, help="Font size for the UI"
    )

    # Parse Known Args
    # We use parse_known_args() so that if the user passes ROS flags (like --ros-args),
    # argparse won't throw an error. It puts unknown flags into 'ros_args'.
    args, ros_args = parser.parse_known_args()

    # Handle Signals
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Launch App
    app = QApplication(sys.argv)

    # Set font size if specified
    if args.font_size is not None:
        font = app.font()
        font.setPointSize(args.font_size)
        app.setFont(font)

    # Create dashboard
    window = BddVisualizer(
        status_topic=args.topic,
        width=args.width,
        height=args.height,
        font_size=args.font_size,
        ros_args=ros_args,
    )
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
