#! /usr/bin/python
"""
Created on Mon Apr 21 00:43:25 2014

@author: mbocamazo, Eric Miller
Last updated: 2016/03
Current shortcomings:
sail torque, drag model equations
"""

#from random import *
import math
from math import atan2, degrees, pi, sin, cos, radians, sqrt
import time

class WorldModel:
    """encodes simulator world state"""
    enable_wrapping=False

    def __init__(self,windspeed,windheading):
        self.boat1 = Boat(40,0,0) #later include boat list for support of multiple boats
        self.wind = Wind(windspeed,windheading)
        self.lastclock = time.time()
        self.relwind = abs(self.wind.windheading-self.boat1.heading)%(2.0*pi)
        if self.relwind > pi:
            self.relwind = 2.0*pi-self.relwind
        self.relwindcomp = pi-self.relwind #the complement
            
    def update_model(self):
        dt = time.time()-self.lastclock
        self.lastclock += dt
        self.relwind = abs(self.wind.windheading-self.boat1.heading)%(2.0*pi)
        if self.relwind > pi:
            self.relwind = 2.0*pi-self.relwind
        self.relwindcomp = pi-self.relwind
        self.boat1.update(dt, self)
        self.wind.update(dt)
        
        
class Wind:
    """encodes information about the wind"""
    def __init__(self,windspeed,windheading):
        self.windspeed = windspeed
        self.windheading = windheading        
        
    def update(self,dt):
        """updates the wind behavior based on some pattern, placeholder for later"""
        self.windheading = self.windheading%(2.0*pi) #sanitization
        if self.windspeed <= 0:
            self.windspeed = 0
    
class Boat:
    """encodes information about the boat"""
    def __init__(self,length,xpos,ypos,color=(100,100,100)):
        self.length = length
        self.xpos = xpos
        self.ypos = ypos
        self.MainPos = 0
        #self.JibPos = 0
        self.MainSuggestion = 0
        #self.JibSuggestion = 0
        self.RudderPos = 0
        self.RudderSuggestion = 0
        self.vx = 0
        self.vy = 0
        self.heading = 0 #note: not redundant with vx, vy; in simple model could be
        self.angularVelocity = 0
        self.forward_speed = 0
        self.color = color #should be a three-tuple
        self.wind_over_port = True 
        self.log_coefficient = 0.01 #was 0.1
        self.lambda_1 = 0.1 #can't be named lambda, reserved
        self.lambda_2 = 4 #decay rate for angular velocity, to zero, way of encoding drag
        #self.strength_Main = 0.65
        #self.strength_Jib = 1-self.strength_Main
        self.debug_list = (0,0)
        self.main_angle = 0
        #self.jib_angle = 0
        self.motorboat = 3 # Predefined speed of boat, or False to disable
        
        self.k = 2 #velocity scaling for the wind
        self.kw = 0.3 #angular velocity scaling for the torque from the rudder
        self.q = .05 #ang vel scaling for torque from the sails
        
        self.disp_k =0.25 #scaling for the output
        
    def update(self,dt,model):
        self.heading = self.heading % (2.0*pi) #sanitization
        self.trim(model)
        self.kinematics(dt,model)
        if model.enable_wrapping:
            if self.xpos > 820:
                self.xpos = 0
            if self.xpos < 0:
                self.xpos = 820
            if self.ypos > 800:
                self.ypos = 0
            if self.ypos < 0:
                self.ypos = 800
    
    def trim(self,model):
        """readjust sails and rudder to suggestions if possible"""
        if self.MainSuggestion <= 0:
            self.MainSuggestion = 0
        if self.MainSuggestion >= 1:
            self.MainSuggestion = 1
        # if self.JibSuggestion <= 0:
        #    self.JibSuggestion = 0
        # if self.JibSuggestion >= 1:
        #    self.JibSuggestion = 1
        if self.RudderSuggestion >=1:
            self.RudderSuggestion = 1
        elif self.RudderSuggestion <= -1:
            self.RudderSuggestion = -1
        self.RudderPos = self.RudderSuggestion
        self.MainPos = min([self.MainSuggestion,model.relwindcomp*2.0/pi,1]) #in foolish ratio units, [0 1], 1 being full out, 90 degrees
        #self.JibPos = min([self.JibSuggestion,model.relwindcomp*2.0/pi,1])
        self.wind_over_port = ((model.wind.windheading-self.heading)%(2.0*pi))<pi #a boolean
    
    def kinematics(self,dt,model):
        """updates kinematics"""
        #calc the forward speed
        direction = atan2(self.vy,self.vx)
        speed = sqrt(self.vx**2+self.vy**2)
        self.forward_speed = cos(direction-self.heading)*speed #as it accelerates, log aspect diminishes
        
        #rudder torque aspect
        Tr = -self.kw*math.log(abs(self.forward_speed)+1)*self.RudderPos #all of these ratios are made up 
        #sail torque aspect
        Ts = -self.q*sin(self.heading-model.wind.windheading)*sqrt(abs(model.wind.windspeed))
        Ts = 0 #!!! This line substantially decreases the realism.
        #Ts = self.q*self.strength_Jib*sin(self.jib_angle-model.wind.windheading)*sqrt(abs(model.wind.windspeed))
        #this overwrite is an error and needs to be addressed, however, it works decently with it.
        print "Tr={},Ts={}".format(Tr,Ts)
        #log torque aspect?
        
        angular_drag = -cmp(self.angularVelocity,0)*self.angularVelocity**2*self.lambda_2 #effectively drag
        self.angularVelocity += angular_drag*dt
        self.angularVelocity += Tr*dt
        self.angularVelocity += Ts*dt
        
        
        #forward sail aspect
        #Sr = shadow_ratio(model.relwindcomp)
        PrMain = power_ratio(self.MainPos, model.relwindcomp)
        # PrJib = power_ratio(self.JibPos,model.relwindcomp)*self.strength_Jib
        Dr = drag_ratio(self.RudderPos)
        Vmax = Vtmax(model.relwindcomp, self.k) * PrMain*Dr
        self.forward_speed += self.lambda_1*(Vmax-self.forward_speed)*dt  #decay to the max, acceleration term
        if self.motorboat is not False:
            self.forward_speed = self.motorboat
        
        self.debug_list = (Ts, angular_drag, Vtmax(model.relwindcomp,self.k), Vmax)

        #conversion to cartesian
        self.vx = self.forward_speed*cos(self.heading)
        self.vy = self.forward_speed*sin(self.heading)
        
#        floating log aspect (in cartesian, rather than doing vector addition, which is also possible)    
        self.vx += model.wind.windspeed*cos(model.wind.windheading)*self.log_coefficient
        self.vy += model.wind.windspeed*sin(model.wind.windheading)*self.log_coefficient
        
        #finally, updates
        self.heading += self.angularVelocity*dt
        self.xpos += self.vx*dt*self.disp_k
        self.ypos += self.vy*dt*self.disp_k

    def posStr(self):
        return "(x={:.3f},\ty={:.3f},\theading={:.1f} (deg),\tspeed={:.3f})".format(self.xpos, self.ypos, self.heading*180/pi, self.forward_speed)
        
def Vtmax(theta,k):
    """theoretical max for a relative wind angle.  Doesn't belong to the boat class, but could!"""
    a = -0.2
    b = 0.8
    c = -0.2
    Vtmax = a*theta+b*theta**2+c*theta**3
    return Vtmax*k
    
def power_ratio(SailP,relwindcomp):
    """power given sail position, assuming relwindcomp/pi is best"""
    return sin(pi**2.0/(4*relwindcomp)*SailP)  
    
# def shadow_ratio(relwindcomp):
#     """shadow that falls on the jib given the wind"""
#     if relwindcomp >= pi/2.0:
#         return 0
#     else:
#         return (2.0/pi*relwindcomp)-1
        
def drag_ratio(RudderPos):
    """coefficient on velocity from the drag given the rudder pos"""
#    return 1 - 0.9*sin(abs(RudderPos)) #given current state, this means that at pi/4, effectively 1/4 as fast (1-0.9*sin(1))
    return 1 - 0.3*sin(abs(RudderPos*pi/2.0))

if __name__ == '__main__':
    model = WorldModel(2,0) #initial windspeed, windheading
    running = True
    while running:
        model.update_model()
        time.sleep(.1)
        print model.boat1.posStr()
