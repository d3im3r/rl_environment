import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CmdToJointStates(Node):
    def __init__(self):
        super().__init__('cmd_to_joint_states_stage4')

        # Publica el "estado" que consume robot_state_publisher
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Recibe comandos desde ESP32 o terminal
        self.sub = self.create_subscription(JointState, '/arm_cmd', self.cb, 10)

        # IMPORTANTE:
        # - Aquí se define QUÉ joints controla este stage.
        # - Los nombres deben coincidir con el URDF.
        self.joint_names = ['joint1', 'joint2', 'joint3', 'gripper_joint', 'gripper_joint_mirror']
        self.last_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.idx_grip_left  = self.joint_names.index('gripper_joint')
        self.idx_grip_right = self.joint_names.index('gripper_joint_mirror')

        # Publicación periódica (30 Hz) -> robot "vivo" en RViz
        self.timer = self.create_timer(1.0/30.0, self.tick)

    def cb(self, msg: JointState):

        if not msg.name or not msg.position:
            return

        m = dict(zip(msg.name, msg.position))

        for i, jn in enumerate(self.joint_names):
            if jn in m:
                self.last_pos[i] = float(m[jn])

        if 'gripper_joint' in m:
            val = float(m['gripper_joint'])
            self.last_pos[self.idx_grip_left]  = val
            self.last_pos[self.idx_grip_right] = val

    def tick(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = self.last_pos
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()