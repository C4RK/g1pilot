#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#从 Unitree 官方的底层数据格式（DDS/LowState）中提取信息，
#并将其转化为标准的 ROS 2 消息，好让 RViz、控制器和你的算法能看懂。
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from astroviz_interfaces.msg import MotorState, MotorStateList

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class G1JointIndex:
    #unitree发过来的数据是一个数组，代码通过这个字典将关节和电机对应起来。
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


_joint_index_to_ros_name = {
    #将数字变成字符串。RViz加载的URDF模型就是通过字符串来移动对应零件的
    G1JointIndex.LeftHipPitch: "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll: "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw: "left_hip_yaw_joint",
    G1JointIndex.LeftKnee: "left_knee_joint",
    G1JointIndex.LeftAnklePitch: "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll: "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch: "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll: "right_hip_roll_joint",
    G1JointIndex.RightHipYaw: "right_hip_yaw_joint",
    G1JointIndex.RightKnee: "right_knee_joint",
    G1JointIndex.RightAnklePitch: "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll: "right_ankle_roll_joint",
    G1JointIndex.WaistYaw: "waist_yaw_joint",
    G1JointIndex.WaistRoll: "waist_roll_joint",
    G1JointIndex.WaistPitch: "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch: "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll: "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw: "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow: "left_elbow_joint",
    G1JointIndex.LeftWristRoll: "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch: "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw: "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch: "right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll: "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw: "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow: "right_elbow_joint",
    G1JointIndex.RightWristRoll: "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch: "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw: "right_wrist_yaw_joint",
}


class RobotState(Node):
    def __init__(self):
        super().__init__('robot_state')
        #参数声明与读取
        self.declare_parameter('use_robot', True)#True连网线，False本地模拟
        self.declare_parameter('interface', '')#告诉底层SDK该从哪个网卡发数据包
        self.declare_parameter('publish_joint_states', True)#控制是否发布/joint_states话题

        self.use_robot = bool(self.get_parameter('use_robot').value)
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.ns = '/g1pilot'#命名空间

        #定义发布者Quality of Service
        qos_profile = QoSProfile(depth=10)#深度为10,如果数据发布得太快，系统会缓存最后十条
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos_profile)#关节角度
        self.imu_pub = self.create_publisher(Imu, f"{self.ns}/imu", qos_profile)#身体平衡姿态
        self.motor_state_pub = self.create_publisher(MotorStateList, f"{self.ns}/motor_state", qos_profile)#自定义消息
        self.tf_broadcaster = TransformBroadcaster(self)#负责广播坐标变换。实时广播

        #数据结构的准备
        self.joint_indices = sorted(_joint_index_to_ros_name.keys())#数字排序
        self.joint_names = [_joint_index_to_ros_name[i] for i in self.joint_indices]#字符串列表
        self.joint_state_msg = JointState()#预填消息体
        self.joint_state_msg.name = self.joint_names

        if self.use_robot:
            #真机模式
            ChannelFactoryInitialize(0, interface)#初始化Unitree底层通信
            self.subscriber_low_state = ChannelSubscriber("rt/lowstate", LowState_)# 订阅机器人发出的原始信号
            self.subscriber_low_state.Init(self.callback_lowstate)# 一旦有信号，就去执行 callback_lowstate 逻辑
        else:
            #模拟模式
            self.create_timer(0.05, self._sim_tick)# 创建一个 20Hz (1/0.05) 的定时器，周期性调用模拟函数

    def callback_lowstate(self, msg: LowState_):#网线发回一次数据包LowState，这个函数就跳动一次
        now = self.get_clock().now().to_msg()

        imu_msg = Imu()#标准消息格式
        imu_msg.header = Header()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "pelvis"
        imu_msg.orientation.w = float(msg.imu_state.quaternion[0])#四元数
        imu_msg.orientation.x = float(msg.imu_state.quaternion[1])
        imu_msg.orientation.y = float(msg.imu_state.quaternion[2])
        imu_msg.orientation.z = float(msg.imu_state.quaternion[3])
        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])#角速度
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])#加速度
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu_msg)

        # TF pelvis -> imu_link
        t = TransformStamped()#声明一个空的表
        t.header.stamp = now
        t.header.frame_id = "pelvis"#父节点，参考基准，盆骨
        t.child_frame_id = "imu_link"#子节点，安装在盆骨上的IMU传感器
        t.transform.translation.x = 0.0#位移偏置
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = imu_msg.orientation#直接把从原始数据里提取出来的四元数赋给了这个变换。
        self.tf_broadcaster.sendTransform(t)#节点向系统广播

        # Motor states
        positions = []#第一个空箱。装每个电机的姿态
        motor_list_msg = MotorStateList()#第二个空箱。装每个电机的自定义数据，包括温度和电压等
        for idx in self.joint_indices:#按照顺序挨个来
            if idx < len(msg.motor_state):#检查硬件发来的数据结构长度是否正常
                m = msg.motor_state[idx]#拿出第idx电机的原始数据
                motor_state = MotorState()#为这个电机创建一张新的表单
                motor_state.name = _joint_index_to_ros_name[idx]
                motor_state.temperature = float(m.temperature[0] if hasattr(m.temperature, "__len__") else m.temperature)
                motor_state.voltage = float(m.vol)
                motor_state.position = float(m.q)
                motor_state.velocity = float(m.dq)
                motor_list_msg.motor_list.append(motor_state)#将数据填进
                positions.append(float(m.q))#将数据填进

        self.motor_state_pub.publish(motor_list_msg)#发布电机状态

        if self.publish_joint_states:#如果发布
            self.joint_state_msg.header.stamp = now
            self.joint_state_msg.position = positions
            self.joint_pub.publish(self.joint_state_msg)#发布关节而不是电机

    def _sim_tick(self):
        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "pelvis"
        imu_msg.orientation.w = 1.0
        self.imu_pub.publish(imu_msg)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = now
            js.name = self.joint_names
            js.position = [0.0] * len(js.name)#生成29个0的列表
            self.joint_pub.publish(js)#发布，告诉RViz，所有29个电机处于0度位置


def main(args=None):
    rclpy.init(args=args)#启动底层引擎，初始化ROS2的通信环境
    node = RobotState()#实例化RobotState类
    try:
        rclpy.spin(node)#进入死循环，让节点保持活跃
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
