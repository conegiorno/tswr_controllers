import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseStamped
from .model import AWsimModel

import math
import numpy as np
import pandas as pd

# Parameters
k = 0.08  # look forward gain
Lfc = 0.75  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.04  # [s] time tick
WB = 0.28  # [m] wheel base of vehicle
iter_icem = 200 # icem loop iterations
traj_dim = 80 # length of simulated trajectory
mu = 0.0 # mean of standard distribution
sigma = 0.2 # spread in the distribution
elite_size = 20 # size of elites set
cost_err = 0.15 # minimal cost that will break iCEM loop

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
    
    def calc_distance(self, x, y):
        return math.sqrt((self.x-x)**2 + (self.y-y)**2)

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

class TargetPath:

    def __init__(self, cx, cy):
        self.states = [State(el, cy[idx]) for idx, el in enumerate(cx)]
        self.count = len(cx)
        
    def next_idx(self, current_idx):
        if current_idx+1 >= self.count:
            return 0
        
        return current_idx+1

def yaw_from_quaternion(q):
    return np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2)

class iCEMController(Node):

    def __init__(self):
        super().__init__('icem_controller')
        
        options = QoSProfile(depth=1)
        options.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        options.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher_ = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile=options)
        
        self.declare_parameter('waypoints_file', 'config/waypoints.csv')
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        df_waypoints = pd.read_csv(waypoints_file)
        self.thresh_err = 0.1
        initial_state = [2.28139033424668e-05,
                         -6.034809985067113e-07,
                         -6.034809985067113e-07,
                         3.259629011154175e-09,
                         4.117931530345231e-05,
                         -9.948154911398888e-06,
                         1.0]
        self.time_nanosecs = self.get_clock().now().nanoseconds
        self.state = State(x=initial_state[0], y=initial_state[1], yaw=yaw_from_quaternion(initial_state[3:]), v=0.0)
        x_r = df_waypoints['pose.x'].to_numpy()
        y_r = df_waypoints['pose.y'].to_numpy()

        self.model = AWsimModel(dt=dt)
                
        self.target_speed = 3.5 # [units/s]
        
        self.target_path = TargetPath(x_r, y_r)
        self.target_idx = 28
        self.search_target_samples = 10

        self.pose_subscription_ = self.create_subscription(PoseStamped, '/ground_truth/pose', 
                                                           self.icem_controll, 10)
        self.velocity_subscription_ = self.create_subscription(VelocityReport, '/vehicle/status/velocity_status',
                                                            self.update_vel, 10)


    def update_vel(self, vel_status: VelocityReport):
        self.state.v = vel_status.longitudinal_velocity

    def icem_controll(self, pose: PoseStamped):
        pose_l = [pose.pose.position.x, 
                 pose.pose.position.y,
                 pose.pose.position.z, 
                 pose.pose.orientation.x,
                 pose.pose.orientation.y, 
                 pose.pose.orientation.z, 
                 pose.pose.orientation.w]
        
        
        dst = self.target_path.states[self.target_idx].calc_distance(pose_l[0], pose_l[1])
        Lf = Lfc + k * self.state.v
        while dst < Lf:
            self.target_idx = self.target_path.next_idx(self.target_idx)
            dst = self.target_path.states[self.target_idx].calc_distance(pose_l[0], pose_l[1])

        current_state = State(x=pose_l[0], y=pose_l[1], yaw=yaw_from_quaternion(pose_l[3:]), v=self.state.v)
        self.state = current_state
        ai = self.proportional_control()
        di = self.icem_steer_control(current_state, dst)
        ai = np.clip(ai, -2.0, 2.0)
        self.state.a = ai
        
        self.state.update(ai, di)
        
        self.publish_control(di, ai)
        
    def proportional_control(self):
        a = Kp * (self.target_speed - self.state.v)

        return a


    def simulate_cost(self, model, state: State, target_state, dist):
        cost_tuple_list = []        
        # initial sim state
        x0 = np.array([state.x, state.y, state.yaw, state.v])

        steer_angles = self.split_steering_range(mu, sigma, iter_icem)

        for steer_angle in steer_angles:
            u_trj = np.zeros((traj_dim, 2), dtype=float)
            u_trj[:, 0] = state.a
            u_trj[:, 1] = steer_angle
            x_trj = np.zeros((u_trj.shape[0] + 1, x0.shape[0]))
            x_trj[0] = x0
            cost = 999999999.9 # big initial number
            for n in range((u_trj.shape[0])):
                x_trj[n+1] = model.discrete_dynamics(x_trj[n], u_trj[n])
                sim_state = x_trj[n+1]
                # cost as cartesian distance
                cost_new = math.sqrt((sim_state[0] - target_state.x) ** 2 + (sim_state[1] - target_state.y) ** 2)
                # updating the smallest cost
                if cost_new < cost:
                    cost = cost_new
                if cost <= cost_err:
                    break

            state_tuple = (cost, steer_angle, sim_state)
            cost_tuple_list.append(state_tuple)

        return cost_tuple_list

    def split_steering_range(self, mu, sigma, iter):
        steer_angles = []
        increment = float(sigma / iter_icem)
        for angle in range(iter_icem):
            item = (angle * increment) - (sigma / 2)
            steer_angles.append(item)
        
        return steer_angles


    def get_action(self, cost_tuple_list, elite_size):
        elites = []
        global mu

        elites = list(zip(*cost_tuple_list))[1]
        elites = elites[0 : elite_size]

        average = sum(elites) / len(elites)
        mu = average
        mu = np.clip(mu, -0.62 + (sigma / 2), 0.62 - (sigma / 2))

        return elites[0] # best action


    def icem_steer_control(self, state: State, Lf):
        
        if Lf == 0:
            return 0.0
        
        target_state = self.target_path.states[self.target_idx]

        cost_tuple_list = self.simulate_cost(self.model, state, target_state, Lf)
        cost_tuple_list.sort()        
        delta = self.get_action(cost_tuple_list, elite_size)

        # drive straight at start
        if state.v < 0.1:
            delta = 0.0

        return delta


    def publish_control(self, steer, accel):
        acc_msg = AckermannControlCommand()
        acc_msg.lateral.steering_tire_angle = steer
        acc_msg.longitudinal.acceleration = accel
        acc_msg.longitudinal.speed = self.target_speed
        self.publisher_.publish(acc_msg)
        self.get_logger().info(f'Published acc: {accel} and steer: {steer}')

def main(args=None):
    rclpy.init(args=args)

    icem_controller = iCEMController()

    rclpy.spin(icem_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    icem_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
