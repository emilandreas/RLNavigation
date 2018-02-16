import vrepper
from vrepper import vrepper
import os,time
import numpy as np
import pandas
import helpers
import gym
from gym import spaces
from approxeng.input.selectbinder import ControllerResource
from math import pi

# from gym.utils import colorize, seeding
VEHICLE_NAME = "Pioneer_p3dx"
VEHICLE_NAME = "Manta#0"

if VEHICLE_NAME == "Pioneer_p3dx":
    R_MOTOR_NAME = "Pioneer_p3dx_rightMotor"
    L_MOTOR_NAME = "Pioneer_p3dx_leftMotor"
    R_WHEEL_NAME = "Pioneer_p3dx_rightWheel"
    L_WHEEL_NAME = "Pioneer_p3dx_leftWheel"
elif VEHICLE_NAME == "Manta#0":
    MOTOR_NAME = "motor_joint#0"
    STEERING_NAME = "steer_joint#0"

PATH_PATH = "./pathStraight.csv"

class PathFollowEnv(gym.Env):
    def __init__(self,headless=False, use_xbox=False, max_step=200):
        if VEHICLE_NAME == "Pioneer_p3dx":
            global VEHICLE_NAME, R_MOTOR_NAME, L_MOTOR_NAME, R_WHEEL_NAME, L_WHEEL_NAME
        elif VEHICLE_NAME == "Manta#0":
            MOTOR_NAME, STEERING_NAME

        self.venv = venv = vrepper(headless=headless)
        venv.start()
        venv.load_scene(
            os.getcwd() + '/vrep_scenes/path_follow_manta.ttt')

        if VEHICLE_NAME == "Pioneer_p3dx":
            self.car = venv.get_object_by_name(VEHICLE_NAME)
            self.r_motor = venv.get_object_by_name(R_MOTOR_NAME)
            self.l_motor = venv.get_object_by_name(L_MOTOR_NAME)
            self.r_wheel = venv.get_object_by_name(R_WHEEL_NAME)
            self.l_wheel = venv.get_object_by_name(L_WHEEL_NAME)
        elif VEHICLE_NAME == "Manta#0":
            self.car = venv.get_object_by_name("body_dummy#0")
            self.motor = venv.get_object_by_name(MOTOR_NAME)
            self.steering = venv.get_object_by_name(STEERING_NAME)

        self.end_point = venv.get_object_by_name('End')
        self.path = self._load_path()
        self.n_path_elements = self.path.shape[0]
        self.path_progress = 0

        print('(PathFollowEnv) initialized')

        obs = np.array([np.inf]*4)
        act = np.array([1.]*1)

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
        global VEHICLE_NAME

        # observe
        car_pos = np.array(self.car.get_position())
        car_angle = self.car.get_orientation()[2]
        # print('Car_angle: {}'.format(car_angle))

        end_pos = np.array(self.end_point.get_position())

        self.dist_to_target = np.linalg.norm(end_pos - car_pos)

        self.cross_track_err, closest_point_index = helpers.get_dist_to_path(car_pos, self.path[:, :3])

        self.path_progress = float(closest_point_index)/self.n_path_elements


        # Give crosstrack error the right sign, pos if right, neg if left
        vector_path_to_car = np.array(car_pos) - np.array(self.path[closest_point_index, :3])

        if closest_point_index == 0:
            vector_path_direction = np.array(self.path[closest_point_index + 1, :3]) - \
                                    np.array(self.path[closest_point_index, :3])
        else:
            vector_path_direction = np.array(self.path[closest_point_index, :3]) - \
                                    np.array(self.path[closest_point_index - 1, :3])
        cross_prod = np.cross(vector_path_direction, vector_path_to_car)
        if cross_prod[2] > 0:
            self.cross_track_err *= -1

        # print('path_angle: {}'.format(self.path[closest_point_index, 5]))

        # angle difference between path direction and car
        self.diff_angle = helpers.get_angle_diff(np.deg2rad(self.path[closest_point_index, 5]), car_angle)

        if VEHICLE_NAME == "Pioneer_p3dx":
            _, r_wheel_vel = self.r_wheel.get_velocity()
            r_wheel_vel = r_wheel_vel[2]
            _, l_wheel_vel = self.l_wheel.get_velocity()
            l_wheel_vel = l_wheel_vel[2]

            self.observation = np.array([
                self.cross_track_err,
                self.diff_angle,
                r_wheel_vel,
                l_wheel_vel
            ]).astype('float32')

        elif VEHICLE_NAME == "Manta#0":
            car_vel, car_ang_vel = self.car.get_velocity()
            car_vel = car_vel[0]
            car_ang_vel = car_ang_vel[2]
            cross_track_vel = np.sin(self.diff_angle)*abs(car_vel)

            # print("VELOCITIES: car_vel: {}, vector_path_to_car: {}, cross_track_vel: {}".format(car_vel,
            #                                                                                  vector_path_to_car,
            #                                                                                  cross_track_vel))

            steering_angle = self.steering.get_joint_angle()
            steering_angle = steering_angle # around the z-axis


            self.observation = np.array([
                self.cross_track_err,
                cross_track_vel,
                self.diff_angle,
                car_ang_vel
            ]).astype('float32')


    def _control_robo(self, velocity, steering): # -1 < val < 1, -1 is backwards or left, and 1 is full forwards or right
        max_steering_angle = 0.5235987
        max_motor_torque = 60
        self.motor.set_velocity(velocity*max_motor_torque)
        self.steering.set_position_target(np.rad2deg(steering * max_steering_angle))

    def _step(self,actions):
        global VEHICLE_NAME
        self.step_count += 1
        actions = np.clip(actions, -1, 1)



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
            if VEHICLE_NAME == "Pioneer_p3dx":
                MOTOR_MULTIPLIER = 40
                l = actions[0]
                r = actions[1]
                self.l_motor.set_velocity(l*MOTOR_MULTIPLIER)
                self.r_motor.set_velocity(r*MOTOR_MULTIPLIER)
            elif VEHICLE_NAME == "Manta#0":
                # action_vel = actions[0]
                action_steering = actions[0]
                self._control_robo(0.5, action_steering)
                # self._control_robo(action_vel, action_steering)

        self.venv.step_blocking_simulation()

        # observe again
        self._self_observe()

        # reward function
        reward, done = self._reward_function()

        if reward == None:
            reward = 0

        # print("\rDist to target: {}, Reward: {}".format(self.dist_to_target, reward))

        # done = self._exit_environment()

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
    def _reward_function(self):
        temp_reward = 0
        temp_done = False
        if self.step_count >= self._max_episode_steps:
            temp_done = True
        REWARD_SLACK_DIST = 0.5
        if -REWARD_SLACK_DIST <= self.cross_track_err <= REWARD_SLACK_DIST:
            if -pi/2 <= self.diff_angle <= pi/2:
                if self.path_progress >= 0.96:
                    temp_reward = 10
                    temp_done = True
                else:
                    temp_reward = self.path_progress

        return temp_reward, temp_done

    def _exit_environment(self):
        if self.step_count >= self._max_episode_steps or self.path_progress > 0.96:
            return True
        else:
            return False

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
