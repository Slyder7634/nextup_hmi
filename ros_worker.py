import rclpy
from rclpy.node import Node
from PyQt6.QtCore import QThread, pyqtSignal

class RosWorker(QThread):
    node_initialized = pyqtSignal(Node)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.node = None

    def run(self):
        rclpy.init()
        self.node = Node('nextup_hmi')
        self.node_initialized.emit(self.node)
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            # Sleep to control CPU usage - 10ms sleep = ~100Hz max spin rate
            self.msleep(10)  # Added for CPU optimization

    def stop(self):
        self.running = False
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()