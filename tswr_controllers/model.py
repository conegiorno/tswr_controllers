import numpy as np
# import jax.numpy as np


class AWsimModel:
    def __init__(self, dt = 0.04):
        # add number of states and number of inputs
        # avoid wrong shapes
        print("AWSIM model")

    def discrete_dynamics(self, x, u, dt=0.04):
        """Discrete model
        Args:
            x: states variables [state_size]
                [x position, y position, yaw, velocity]
            u: control input [action_size]
                [acceleration, steering]
            dt: sampling time
        Returns:
            Next state [state_size].
        """
        yaw = np.asarray(x)[2]
        v = np.asarray(x)[3]

        acceleration = np.asarray(u)[0]
        steering = np.asarray(u)[1]
        steering = np.clip(steering, -0.62, 0.62)

        x_next = np.array(
            [v * np.cos(yaw), v * np.sin(yaw),
             v * np.tan(steering) / 0.28, acceleration]
        )
        return x + dt * x_next

    def rollout(self, x0, u_trj):
        """Rollout trajectory
        Args:
            x0: initial states [state_size]
                [x position, y position, yaw, velocity]
            u_trj: control trajectory: shape [N, number of states]
                [acceleration, steering]
            dt: sampling time
        Returns:
            x_trj: state trajectory: shape[N+1, number of states].
        """
        x_trj = np.zeros((u_trj.shape[0] + 1, x0.shape[0]))
        x_trj[0] = x0
        for n in range((u_trj.shape[0])):
            x_trj[n+1] = self.discrete_dynamics(x_trj[n], u_trj[n])
        return x_trj


class CarModel:
    def __init__(self, dt=0.04):
        # add number of states and number of inputs
        # avoid wrong shapes
        self.dt = dt
        print("Car model")

    def continuous_dynamics(self, x, u):
        """Continuous model
        Args:
            x: states variables [state_size]
                [x position, y position, yaw, velocity, steering angle]
            u: control input [action_size]
                [acceleration, steering velocity]
        Returns:
            Next state [state_size].
        """
        yaw = x[2]
        v = x[3]
        steer = x[4]
        x_next = np.array(
            [v * np.cos(yaw), v * np.sin(yaw),
             v * np.tan(steer), u[0], u[1]]
        )
        return x_next

    def discrete_dynamics(self, x, u):
        """Discrete model
        Args:
            x: states variables [state_size]
                [x position, y position, yaw, velocity, steering angle]
            u: control input [action_size]
                [acceleration, steering velocity]
            dt: sampling time
        Returns:
            Next state [state_size].
        """
        yaw = np.asarray(x)[2]
        v = np.asarray(x)[3]
        steer = np.asarray(x)[4]
        acceleration = np.asarray(u)[0]
        sterring = np.asarray(u)[1]
        sterring = np.clip(sterring, -0.62, 0.62)
        x_next = np.array(
            [v * np.cos(yaw) , v * np.sin(yaw),
             v * np.tan(steer) / 0.28, acceleration, sterring]
        )
        return x + self.dt * x_next

    def rollout(self, x0, u_trj):
        """Rollout trajectory
        Args:
            x0: initial states [state_size]
                [x position, y position, yaw, velocity, steering angle]
            u_trj: control trajectory: shape [N, number of states]
                [acceleration, steering velocity]
            dt: sampling time
        Returns:
            x_trj: state trajectory: shape[N+1, number of states].
        """
        x_trj = np.zeros((u_trj.shape[0] + 1, x0.shape[0]))
        x_trj[0] = x0
        for n in range((u_trj.shape[0])):
            x_trj[n+1] = self.discrete_dynamics(x_trj[n], u_trj[n])
        return x_trj
