import vrepper
from vrepper import vrepper
import os,time
import numpy as np
import pandas
import helpers
import gym
from gym import spaces
from approxeng.input.selectbinder import ControllerResource

# from gym.utils import colorize, seeding
VEHICLE_NAME = "Pioneer_p3dx"
R_MOTOR_NAME = "Pioneer_p3dx_rightMotor"
L_MOTOR_NAME = "Pioneer_p3dx_leftMotor"
R_WHEEL_NAME = "Pioneer_p3dx_rightWheel"
L_WHEEL_NAME = "Pioneer_p3dx_leftWheel"
PATH_PATH = "./pathStraight.csv"

class PathFollowEnv(gym.Env):
    def __init__(self,headless=False, use_xbox=False, max_step=200):
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
        self.end_point = venv.get_object_by_name('End')
        self.path = self._load_path()
        self.n_path_elements = self.path.shape[0]
        self.path_progress = 0

        print('(PathFollowEnv) initialized')

        obs = np.array([np.inf]*4)
        act = np.array([1.]*2)

        self.action_space = spaces.Box(-act, act)
        self.observation_space = spaces.Box(-obs, obs)
        self.step_count = 0
        self._max_episode_steps = max_step
        self.use_xbox = use_xbox
        if use_xbox:
            self.joystick = ControllerResource()

    def _load_path(self):
        global PATH_PATH
        path = pandas.read_csv(PATH_PATH,',')
        path_array = path.as_matrix()
        path_array = path_array[:, :6]
        return path_array

    def _self_observe(self):
        # observe then assign
        car_pos = np.array(self.car.get_position())
        car_angle = self.car.get_orientation()[2]

        end_pos = np.array(self.end_point.get_position())
        self.dist_to_target = np.linalg.norm(end_pos - car_pos)

        # get wheel speed around z-axis
        _, r_wheel_vel = self.r_wheel.get_velocity()
        r_wheel_vel = r_wheel_vel[2]
        _, l_wheel_vel = self.l_wheel.get_velocity()
        l_wheel_vel = l_wheel_vel[2]

        # cross-track
        cross_track_err, closest_point_index = helpers.get_dist_to_path(car_pos, self.path[:, :3])
        self.path_progress = float(closest_point_index)/self.n_path_elements

        # angle difference between path direction and car
        diff_angle = helpers.get_angle_diff(np.deg2rad(self.path[closest_point_index, 5]), car_angle)

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
        self.step_count += 1
        actions = np.clip(actions, -1, 1)
        l = actions[0]
        r = actions[1]

        MOTOR_MULTIPLIER = 40

        # override if xbox controller is used
        if self.use_xbox:
            with self.joystick as joystick:
                if joystick.connected:
                    vel = joystick['ly']
                    steering = joystick['lx']
                    print("vel: {}, streering: {}".format(vel, steering))
                    self._control_robo(vel, steering)
        else:

            # Directly map input to each wheel
            self.l_motor.set_velocity(l*MOTOR_MULTIPLIER)
            self.r_motor.set_velocity(r*MOTOR_MULTIPLIER)

        self.venv.step_blocking_simulation()

        # observe again
        self._self_observe()
        done = False
        # cost
        k1 = 10
        k2 = 10
        cross_track_err = self.observation[0]
        reward = (k1*self.path_progress)**2 - (k2*cross_track_err)**2

        success_dist = 0.5
        # print("\rDist to target: {}, Reward: {}".format(self.dist_to_target, reward))
        if self.dist_to_target < success_dist:
            done = True
            reward += 100
        if self.step_count >= self._max_episode_steps:
            done = True
        return self.observation, reward, done, {}

    def _reset(self):
        self.venv.stop_simulation()
        self.venv.start_blocking_simulation()
        self._self_observe()
        self.step_count = 0
        return self.observation

    def _destroy(self):
        self.venv.stop_blocking_simulation()
        self.venv.end()


if __name__ == '__main__':
    use_xbox = True
    env = PathFollowEnv(headless=False, use_xbox=use_xbox, max_step=np.infty)
    for k in range(5):
        observation = env.reset()
        done = False
        while not done:
            action = env.action_space.sample() # your agent here (this takes random actions)
            observation, reward, done, info = env.step(action)
        print("end of episode")
    print('simulation ended. leaving in 5 seconds...')
    time.sleep(5)
