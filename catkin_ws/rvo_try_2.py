#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from sympy import *
from datetime import datetime

with open('system3_2.txt', 'r') as f:
    lines = f.readlines()
    comfort_x = [float(line.split()[5]) for line in lines]
    comfort_y = [float(line.split()[6]) for line in lines]

with open('system2_2.txt', 'r') as f1:
    lines1 = f1.readlines()
    comfort_x1 = [float(line.split()[5]) for line in lines1]
    comfort_y1 = [float(line.split()[6]) for line in lines1]

fo = open("foo_2.txt", "rw+")    




past_route_x =[]
past_route_y =[]
settle_time=0.1
walker_prefer_speed=0.4
walker_angular_velocity_constraint=1    
total_time=0.0

class Agents:
    def __init__(self, position, r,v):
        self.position = position  
        self.r = r  
        self.vx = v[0]
        self.vy = v[1]
        self.distance = 0
        self.point_theta = 0
        self.distance_theta = 0
        self.velocity_theta = 0
        self.collision_time_walker = 0
        self.comfort=0
    def distance_calculate(self,igo_position):
        self.distance = math.sqrt(pow((self.position[0]-igo_position[0]),2)+pow((self.position[1]-igo_position[1]),2))
        self.distance_theta = np.arctan2(self.position[1]-igo_position[1] , self.position[0]-igo_position[0])
    def intersect_theta(self,igo_r):
        new_r = self.r + igo_r
        self.point_theta = np.arcsin(new_r / self.distance) 
    def cal_velocity_theta(self,vx,vy):
        self.velocity_theta=np.arctan2(-self.vy+vy,-self.vx+vx)
    def collision_time(self,vx,vy):
        new_r = self.r + 0.3
        velocity=math.sqrt(pow((-self.vy+vy),2)+pow((-self.vx+vx),2))
        #print 'velocity', velocity
        self.velocity_theta=np.arctan2(-self.vy+vy,-self.vx+vx)
        psi=abs(self.distance_theta-self.velocity_theta)
        
        self.collision_time_walker = abs(self.distance*np.cos(psi)-math.sqrt(pow((new_r),2)-pow((self.distance*np.sin(psi)),2)))/velocity
        # print 'collision_time',self.collision_time_walker
    def cal_comfort(self):
        aa=cos(self.velocity_theta-self.distance_theta)
        if aa<0:
            aa=0.01
        elif aa<0.5:
            aa=0.5
        if(self.distance<0.6):
            self.distance=0.6
        elif(self.distance>8):
            self.distance=8
        self.comfort=2/(1+math.exp(0.8*(self.distance-0.6)/aa))
        
    

class Walker:
    def _init_(self):
        self.position = []
        self.r = 0
        self.vx = 0
        self.vy = 0
        self.velocity_theta=0
        self.goal = []
        self.goal_time = 0
    def self_cb(self,position,r):
        self.position = position
        self.r = r
        self.vx = 0.0
        self.vy = 0.4
        self.velocity_theta=90*3.14/180
    def set_goal(self, position):
        self.goal = position
    def prefer_angle (self):
        return  np.arctan2(self.goal[1]-self.position[1] , self.goal[0]-self.position[0])
    def goal_time_count(self):
        d = math.sqrt(pow((self.position[0]-self.goal[0]),2)+pow((self.position[1]-self.goal[1]),2))
        self.goal_time = d/0.4
        # print 'goal_time',self.goal_time
    def next_speed(self,theta):
        if((theta-np.arctan2(self.vy,self.vx))>walker_angular_velocity_constraint*settle_time):
            self.velocity_theta+=walker_angular_velocity_constraint*settle_time
            print walker_angular_velocity_constraint*settle_time
        elif((np.arctan2(self.vy,self.vx)-theta)>walker_angular_velocity_constraint*settle_time):
            self.velocity_theta-=walker_angular_velocity_constraint*settle_time
            print walker_angular_velocity_constraint*settle_time
        else:
            self.velocity_theta = theta
            print"3"
        self.vx = walker_prefer_speed*np.cos(self.velocity_theta)
        self.vy = walker_prefer_speed*np.sin(self.velocity_theta)

fig, ax = plt.subplots()
fig.set_size_inches(8,8)

list = []

human_r=0.5
list.append(Agents([ 5.6 ,8.5], human_r,[0,-1]))
list.append(Agents([ 1 ,8], human_r,[0.707,-0.707]))
# list.append(Agents([ 4 ,8], human_r,[0,-1]))
# list.append(Agents([ 5 ,7], human_r,[0,-1]))


# for i in range (17):
#     list.append(Agents([ 3 ,0.3+0.6*i], 0.3,[0,0]))
#     list.append(Agents([ 6 ,0.3+0.6*i], 0.3,[0,0]))

# for i in range (8):
#     list.append(Agents([ 6+0.6*i ,5.1], 0.3,[0,0]))
#     list.append(Agents([ 6 ,0.3+0.6*i], 0.3,[0,0]))
#list.append(Agents([ 5 ,2], 0.3,[0,-1]))

walker = Walker()
walker.self_cb([5,1], 0.3)
walker.set_goal([5,6])


while walker.position[1]<10:

    
    theta_plus_origin=0
    pedestrian_number=0
    moving_list = []
    collision_list = []
    collision = false
    for obj in list:
        obj.distance_calculate(walker.position)
        if(obj.distance<0.6):
            print 'detect collision:',pedestrian_number
        
        obj.intersect_theta(walker.r)
        obj.cal_velocity_theta(walker_prefer_speed*np.cos(walker.prefer_angle()),walker_prefer_speed*np.sin(walker.prefer_angle()))
        if ((obj.velocity_theta)<(obj.distance_theta+obj.point_theta))&(obj.velocity_theta>(obj.distance_theta-obj.point_theta)):
            collision_list.append(pedestrian_number)
        pedestrian_number+=1
    # print collision_list
    walker.goal_time_count()
    min_collision_time=100
    for obj in collision_list:
        list[obj].collision_time(walker_prefer_speed*np.cos(walker.prefer_angle()),walker_prefer_speed*np.sin(walker.prefer_angle()))
        if(list[obj].collision_time_walker<walker.goal_time):
            collision = true
        if(min_collision_time>list[obj].collision_time_walker):
            min_collision_time=list[obj].collision_time_walker
    # print 'min_collision_time',min_collision_time
    if(collision==false):
        prefer_angle1=walker.prefer_angle()
        print prefer_angle1*180/3.14
        print"safe"
        walker.next_speed(prefer_angle1)
        if walker.position[1]<5.9:
            walker.position[0]+=walker.vx*settle_time
            walker.position[1]+=walker.vy*settle_time
    else :

        theta_plus=0
        theta_minus=0

        while(collision==true):
            theta_plus+=1*3.14/180
            collision_list = []
            pedestrian_number=0
            
            last_min_collision_time=min_collision_time
            for obj in list:
                obj.distance_calculate(walker.position)
                
                
                obj.intersect_theta(walker.r)
                obj.cal_velocity_theta(walker_prefer_speed*np.cos(walker.prefer_angle()+theta_plus),walker_prefer_speed*np.sin(walker.prefer_angle()+theta_plus))
                if ((obj.velocity_theta)<(obj.distance_theta+obj.point_theta))&(obj.velocity_theta>(obj.distance_theta-obj.point_theta)):
                    collision_list.append(pedestrian_number)
                pedestrian_number+=1
            # print collision_list
            if(len(collision_list)==0):
                collision =false
            else:
                min_collision_time=100
                for obj in collision_list:
                    list[obj].collision_time(walker_prefer_speed*np.cos(walker.prefer_angle()+theta_plus),walker_prefer_speed*np.sin(walker.prefer_angle()+theta_plus))
                    if(list[obj].collision_time_walker<walker.goal_time):
                        collision = true
                    if(min_collision_time>list[obj].collision_time_walker):
                        min_collision_time=list[obj].collision_time_walker
                        min_obj=obj
            # print 'min_collision_time',min_obj,min_collision_time    
            if(min_collision_time-last_min_collision_time>=settle_time*5):
                collision =false

            theta_plus_origin=theta_plus

            if(theta_plus>=1.57):
                collision =false
                theta_plus=list[min_obj].point_theta+list[min_obj].distance_theta-walker.prefer_angle()
                if(theta_plus>(-list[min_obj].point_theta+list[min_obj].distance_theta-walker.prefer_angle())):
                    theta_plus=-list[min_obj].point_theta+list[min_obj].distance_theta-walker.prefer_angle()
        collision =true

        while(collision==true):
            theta_minus+=1*3.14/180
            collision_list = []
            pedestrian_number=0
            
            last_min_collision_time=min_collision_time
            for obj in list:
                obj.distance_calculate(walker.position)
                
                
                obj.intersect_theta(walker.r)
                obj.cal_velocity_theta(walker_prefer_speed*np.cos(walker.prefer_angle()-theta_minus),walker_prefer_speed*np.sin(walker.prefer_angle()-theta_minus))
                if ((obj.velocity_theta)<(obj.distance_theta+obj.point_theta))&(obj.velocity_theta>(obj.distance_theta-obj.point_theta)):
                    collision_list.append(pedestrian_number)
                pedestrian_number+=1
            # print collision_list
            if(len(collision_list)==0):
                collision =false
            else:
                min_collision_time=100
                for obj in collision_list:
                    list[obj].collision_time(walker_prefer_speed*np.cos(walker.prefer_angle()-theta_minus),walker_prefer_speed*np.sin(walker.prefer_angle()-theta_minus))
                    if(list[obj].collision_time_walker<walker.goal_time):
                        collision = true
                    if(min_collision_time>list[obj].collision_time_walker):
                        min_collision_time=list[obj].collision_time_walker
                        min_obj=obj

            # print 'min_collision_time',min_obj,min_collision_time    
            if(min_collision_time-last_min_collision_time>=settle_time*5):
                collision =false    

            if(theta_minus>=1.57):
                collision =false
                if(theta_plus<1.57):
                    break
                else:
                    theta_minus=list[min_obj].point_theta-list[min_obj].distance_theta+walker.prefer_angle()
                    if((-list[min_obj].point_theta-list[min_obj].distance_theta+walker.prefer_angle())<theta_minus):
                        theta_minus=-list[min_obj].point_theta-list[min_obj].distance_theta+walker.prefer_angle()
                    
            else:
                theta_plus=theta_plus_origin

        if(theta_plus<theta_minus):
            prefer_angle1=walker.prefer_angle()+theta_plus
            walker.next_speed(prefer_angle1)
            if walker.position[1]<5.9:
                walker.position[0]+=walker.vx*settle_time
                walker.position[1]+=walker.vy*settle_time
            
        else:
            prefer_angle1=walker.prefer_angle()-theta_minus
            walker.next_speed(prefer_angle1)
            if walker.position[1]<5.9:
                walker.position[0]+=walker.vx*settle_time
                walker.position[1]+=walker.vy*settle_time
           
    

    total_time+=settle_time
    list[0].cal_comfort()
    list[1].cal_comfort()
    fo.write("%f  "%(total_time))
    fo.write("%f  "%(list[1].comfort))
    fo.write("%f  "%(list[1].distance-0.6))
    fo.write("%f  "%(list[0].comfort))
    fo.write("%f\n"%(list[0].distance-0.6))


    print walker.velocity_theta*180/3.14
        
    for obj in list:
        obj.position[0]+=settle_time*obj.vx
        obj.position[1]+=settle_time*obj.vy
    
    
    past_route_x.append(walker.position[0]  )
    past_route_y.append(walker.position[1]  )
    print(walker.position)
    ax.clear()
    ax.grid('on')
    if walker.position[1]<5.9:
        ax.add_patch(plt.Circle((walker.position[0]-5, walker.position[1]-1), walker.r, color='green', alpha=0.5))
    ax.add_patch(plt.Circle((walker.goal[0]-5, walker.goal[1]-1), 0.1, color='red', alpha=0.5))
    # plt.quiver(walker.position[0]-5, walker.position[1]-1, np.cos(walker.velocity_theta), np.sin(walker.velocity_theta), pivot='middle')
    number=0
    for obj in list:
        ax.add_patch(plt.Circle((obj.position[0]-5,obj.position[1]-1), obj.r, color='black', alpha=0.5))
        plt.text(obj.position[0]-0.1-5, obj.position[1]-0.1-1, number, fontsize=14)

        number+=1

    
    for i in range(len(past_route_x)):
        circ1=plt.Circle((past_route_x[i]-5,past_route_y[i]-1), 0.02, color='blue', alpha=0.5)
        ax.add_patch(circ1)
        
        
    for i in range(len(comfort_x)):
        circ3=plt.Circle((-comfort_y[i],comfort_x[i]+4), 0.02, color='red', alpha=0.5)
        ax.add_patch(circ3)

    for i in range(len(comfort_x1)):
        circ2=plt.Circle((-comfort_y1[i],comfort_x1[i]+4), 0.02, color='black', alpha=0.5)
        ax.add_patch(circ2) 
        

    

    ax.legend([circ1,circ2, circ3], ['System 1','System 2','Our System'],loc='lower right')    

        

    ax.plot()
    plt.xlim(-5,5)
    plt.ylim(-1,9)
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")

    ax.set_xticks(np.linspace(-5, 5, 11))
    ax.set_yticks(np.linspace(-1, 9, 11))
    # plt.show(block=True)
    plt.pause(0.0000001)
    time.sleep(settle_time) 
    
    

# for i in range(200):
    
   
    

    # def cross_point1(self,igo_position):
    #     crosspoint1=[self.distance*np.cos(self.point_theta)*np.cos(self.distance_theta-self.point_theta)+igo_position[0],self.distance*np.cos(self.point_theta)*np.sin(self.distance_theta-self.point_theta)+igo_position[1]]
    #     return crosspoint1
    # def cross_point2(self,igo_position):
    #     crosspoint1=[self.distance*np.cos(self.point_theta)*np.cos(self.distance_theta+self.point_theta)+igo_position[0],self.distance*np.cos(self.point_theta)*np.sin(self.distance_theta+self.point_theta)+igo_position[1]]
    #     return crosspoint1