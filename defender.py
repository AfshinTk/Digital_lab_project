import math
import time
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class PID:
    def __init__(self, kp=2, ki=0.0, kd=0.0, current_time=None):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.sample_time = 0.01
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

    def update(self, feedback_value, target ,current_time=None):
        error = target - feedback_value
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            
            self.DTerm = 0.0
            self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

class One_Direction(PID):
    def __init__(self):
        PID.__init__(self,-200, 0, 0)
        self.x=0
        self.y=0
    def set_desire_pos(self,des_x,des_y):
        self.des_x=des_x
        self.des_y=des_y
    def set_direction(self,direction):
        self.direction_vector=(math.sin(direction),-math.cos(direction))
    def update_robot_pos(self,robot_pos):
        self.x=robot_pos[0]
        self.y=robot_pos[1]
        self.update(self.direction_vector[0]*(self.des_x-self.x)+self.direction_vector[1]*(self.des_y-self.y),0)
    def One_Direction_motor_speed(self):
        return max(min(self.output,10),-10),max(min(self.output,10),-10)

class Angle_control(PID):
    def __init__(self):
        PID.__init__(self,-5, 0, 0)
        self.pi=0
    def sef_desire_pi(self,des_pi):
        self.des_pi=des_pi
    def update_robot_pi(self,robot_pi):
        self.pi=robot_pi
        self.update(self.pi,self.des_pi)
    def Angular_motor_speed(self):
        return max(min(-self.output,10),-10),max(min(self.output,10),-10)

class GotoXY(Angle_control,One_Direction):
    def __init__(self):
        Angle_control.__init__(self)
        One_Direction.__init__(self)
        self.pi=0
        self.x=0
        self.y=0
        self.finished_GotoXY=1
    def set_des_XY(self,des_x,des_y):
        self.des_x=des_x
        self.des_y=des_y
    def update_robot_pos_pi(self,robot_pi,robot_pos):
        self.pi=robot_pi
        self.x=robot_pos[0]
        self.y=robot_pos[1]
        self.des_pi=math.atan2(self.des_y-self.y,self.des_x-self.x)+math.pi/2
        if((self.des_x-self.x)**2+(self.des_y-self.y)**2<0.0001):
            self.vl,self.vr=0,0
            self.finished_GotoXY=1
            return
        self.finished_GotoXY=0
        if(abs(self.des_pi-self.pi)>0.07):
            self.sef_desire_pi(self.des_pi)
            self.update_robot_pi(self.pi)
            self.vl,self.vr=self.Angular_motor_speed()
        else:
            self.set_desire_pos(self.des_x,self.des_y)
            self.set_direction(robot_pi)
            self.update_robot_pos(robot_pos)
            self.vl,self.vr=self.One_Direction_motor_speed()
    def XY_motor_speed(self):
        return self.vl,self.vr
        
class ChasingBall(Angle_control,One_Direction):
    def __init__(self):
        Angle_control.__init__(self)
        One_Direction.__init__(self)
        self.end_ChasingBall=1
        self.near=0.0005
    def update_ball__data(self,ball_data):
        self.des_pi=math.atan2(ball_data["direction"][1],ball_data["direction"][0])
        if(1/ball_data["strength"]<self.near):
            self.vl,self.vr=0,0
            self.end_ChasingBall=1
            return
        self.end_ChasingBall=0
        if(abs(self.des_pi)>0.1):
            self.sef_desire_pi(self.des_pi)
            self.update_robot_pi(0)
            self.vl,self.vr=self.Angular_motor_speed()
        else:
            self.vl,self.vr=10,10
    def ChasingBall_MotorSpeed(self):
        return self.vl,self.vr
        
class MyRobot1(RCJSoccerRobot):
    def run(self):
        control_xy = GotoXY()
        control_track=ChasingBall()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue
                robot_pos=self.get_gps_coordinates()
                control_xy.set_des_XY(0,0.7)
                control_xy.update_robot_pos_pi(self.get_compass_heading(),robot_pos)
                control_track.update_ball__data(ball_data)
                if(robot_pos[1]<=0.68):
                    vl,vr=control_xy.XY_motor_speed()
                elif(1/ball_data["strength"]>0.05):
                    vl,vr=control_track.ChasingBall_MotorSpeed()
                else:
                    vl,vr=9,-9
                self.left_motor.setVelocity(vl)
                self.right_motor.setVelocity(vr)
