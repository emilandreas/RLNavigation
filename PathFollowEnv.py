import vrepper
from vrepper import vrepper
from Simulator import Simulator
import os,time
import numpy as np
import pandas
import helpers
import gym
from gym import spaces
# from gym.utils import colorize, seeding
VEHICLE_NAME = "Pioneer_p3dx"
R_MOTOR_NAME = "Pioneer_p3dx_rightMotor"
L_MOTOR_NAME = "Pioneer_p3dx_leftMotor"
R_WHEEL_NAME = "Pioneer_p3dx_rightWheel"
L_WHEEL_NAME = "Pioneer_p3dx_leftWheel"
PATH_PATH = "./pathStraight.csv"

class PathFollowEnv(gym.Env):
    def __init__(self,headless=False):
        global VEHICLE_NAME, R_MOTOR_NAME, L_MOTOR_NAME, R_WHEEL_NAME, L_WHEEL_NAME
        self.venv = venv = vrepper(headless=headless)
        venv.start()
        venv.load_scene(
            os.getcwd() + '/vrep_scenes/path_follow_straight.ttt')

        self.car = venv.get_object_by_name(VEHICLE_NAME)
        self.r_motor = venv.get_object_by_name(R_MOTOR_NAME)
        self.l_motor = venv.get_object_by_name(L_MOTOR_NAME)
        self.r_wheel = venv.get_object_by_name(R_WHEEL_NAME)
        self.l_wheel = venv.get_object_by_name(L_WHEEL_NAME)
        self.path = self._load_path()
        self.n_path_elements = self.path.shape[0]
        self.path_progress = 0

        print('(PathFollowEnv) initialized')

        obs = np.array([np.inf]*4)
        act = np.array([1.]*2)

        self.action_space = spaces.Box(-act, act)
        self.observation_space = spaces.Box(-obs, obs)

    def _load_path(self):
        global PATH_PATH
        path = pandas.read_csv(PATH_PATH,',')
        path_array = path.as_matrix()
        path_array = path_array[:, :6]
        return path_array

    def _self_observe(self):
        # observe then assign
        carpos = self.car.get_position()
        carangle = self.car.get_orientation()[2]
        _, r_wheel_vel = self.r_wheel.get_velocity()
        _, l_wheel_vel = self.l_wheel.get_velocity()

        # cross-track
        cross_track_err, closest_point_index = helpers.get_dist_to_path(carpos, self.path[:, :3])
        self.path_progress = float(closest_point_index)/self.n_path_elements

        # angle difference between path direction and car
        diff_angle = helpers.get_angle_diff(np.deg2rad(self.path[closest_point_index, 5]), carangle)

        self.observation = np.array([
            cross_track_err,
            diff_angle,
            r_wheel_vel,
            l_wheel_vel
        ]).astype('float32')


    def _control_robo(self, velocity, steering): # -1 < val < 1, -1 is backwards or left, and 1 is full forwards or right

        MOTOR_MULTIPLIER = 40

        if steering > 0:
            right_motor_val = (1-abs(steering))*(velocity-abs(steering))
            left_motor_val = 1*(velocity + abs(steering))
        else:
            right_motor_val = 1*(velocity + abs(steering))
            left_motor_val = (1-abs(steering))*(velocity - abs(steering))

        self.l_motor.set_velocity(left_motor_val*MOTOR_MULTIPLIER)
        self.r_motor.set_velocity(right_motor_val*MOTOR_MULTIPLIER)

    def _step(self,actions):
        actions = np.clip(actions, -1, 1)
        l = actions[0]
        r = actions[1]

        # Directly map input to each wheel
        MOTOR_MULTIPLIER = 40
        self.l_motor.set_velocity(l*MOTOR_MULTIPLIER)
        self.r_motor.set_velocity(r*MOTOR_MULTIPLIER)

        self.venv.step_blocking_simulation()

        # observe again
        self._self_observe()

        # cost
        k1 = 10
        k2 = 10
        cross_track_err = self.observation[0]
        reward = (k1*self.path_progress)**2 - (k2*cross_track_err)**2

        return self.observation, reward, False, {}

    def _reset(self):
        self.venv.stop_blocking_simulation()
        self.venv.start_blocking_simulation()
        self._self_observe()
        return self.observation

    def _destroy(self):
        self.venv.stop_blocking_simulation()
        self.venv.end()

if __name__ == '__main__':
    env = PathFollowEnv(headless=False)
    for k in range(5):
        observation = env.reset()
        for _ in range(20):
            # env.render()
            action = env.action_space.sample() # your agent here (this takes random actions)
            observation, reward, done, info = env.step(action)
            print(reward)

    print('simulation ended. leaving in 5 seconds...')
    time.sleep(5)
