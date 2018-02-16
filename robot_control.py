from vrep_lib import vrep

class Pioneer_robo_control:
    def __init__(self, clientID, robo_handle, right_motor_handle, left_motor_handle):
        self.robo_handle = robo_handle
        self.right_motor_handle = right_motor_handle
        self.left_motor_handle = left_motor_handle
        self.clientID = clientID

    def control_robo(self, velocity, steering): # -1 < val < 1, -1 is backwards or left, and 1 is full forwards or right

        MOTOR_MULTIPLIER = 40

        if steering > 0:
            right_motor_val = (1-abs(steering))*(velocity-abs(steering))
            left_motor_val = 1*(velocity + abs(steering))
        else:
            right_motor_val = 1*(velocity + abs(steering))
            left_motor_val = (1-abs(steering))*(velocity - abs(steering))

        self.set_left_motor_speed(left_motor_val*MOTOR_MULTIPLIER)
        self.set_right_motor_speed(right_motor_val*MOTOR_MULTIPLIER)

    def set_right_motor_speed(self, val):
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor_handle, val, vrep.simx_opmode_streaming)

    def set_left_motor_speed(self, val):
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor_handle, val, vrep.simx_opmode_streaming)



class Manta_robo_control:
    def __init__(self, clientID, robo_handle, steering_handle, motor_handle):
        self.robo_handle = robo_handle
        self.steering_handle = steering_handle
        self.motor_handle = motor_handle
        self.clientID = clientID
        self.max_steering_angle = 0.5235987
        self.max_motor_torque = 60

    def control_robo(self, velocity, steering):
        self.set_steering(-steering*self.max_steering_angle)
        self.set_motor_speed(velocity*self.max_motor_torque)

    def set_steering(self, val):
        errorCode = vrep.simxSetJointTargetPosition(self.clientID, self.steering_handle, val, vrep.simx_opmode_streaming)

    def set_motor_speed(self, val):
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.motor_handle, val, vrep.simx_opmode_streaming)
