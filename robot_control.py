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



