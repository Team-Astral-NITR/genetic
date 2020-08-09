#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from enum import Enum

import random
import numpy as np
import math
from collections import OrderedDict
from itertools import islice

from picker_robot.msg import Errors
from pid_controller.msg import PID

no_of_weights=3
initital_pop=int(math.pow(2,9))

class DOF(Enum):
     ROLL=0
     PITCH=1
     YAW=2
     SURGE=3
     SWAY=4
     HEAVE=5



class GeneticNode:
    def __init__(self,val):
        #publishers and subscribers

        rospy.Subscriber('/genetic',Errors,self.genetic_callback)
        #i and d errors
        self.int_err=0
        self.prev_err=0
        #ends

        #some more parameters for new changes
        self.degree_of_freedom=val
        self.status=0
        self.pid_vals=[0,0,0]
        #ends
    def genetic_callback(self,msg):
        err=list(msg.error)
        #print(err,self.degree_of_freedom)
        #return
        self.int_err+=err[self.degree_of_freedom]
        initial_pop=np.random.uniform(low=0,high=0.5,size=(initital_pop,no_of_weights))
        if(self.degree_of_freedom==3 or self.degree_of_freedom==4 or self.degree_of_freedom==5 or  self.degree_of_freedom==2):
            self.status=1
            return
        #genetic stuff
        if(self.status==0):
            while(len(initial_pop) > 1):
                #print(self.degree_of_freedom)
                fitness=self.fitness_finder(initial_pop,err)
                parents=self.selection(fitness)
                children=self.crossover(parents)
                mutated_children=self.mutation(children)
                initial_pop=mutated_children
            self.prev_err=err[self.degree_of_freedom]
            self.pid_vals[0]=initial_pop[0][0]
            self.pid_vals[1]=initial_pop[0][1]
            self.pid_vals[2]=initial_pop[0][2]
            self.status=1


    def fitness_finder(self,initial_pop,err):
        terms=[err[self.degree_of_freedom],self.int_err,err[self.degree_of_freedom]-self.prev_err]
        fit=1/np.sum(initial_pop*terms,axis=1)
        fitness={}
        for i in range(0,len(fit)):
            fitness[fit[i]]=initial_pop[i]
        fitness=OrderedDict(sorted(fitness.items(),reverse=True))
        return fitness

    def selection(self,fitness):
        parents=np.empty(((len(fitness)//2),no_of_weights))
        count=0
        for k, v in islice(fitness.items(), len(fitness)//2):
            parents[count]=v
            count=count+1
        return parents

    def crossover(self,parents):
        number=len(parents)
        if(number == 1):
            return parents
        children=np.empty((len(parents),no_of_weights))
        parent1=parents[0:number//2]
        parent2=parents[number//2:]
        crossover_point=parents.shape[1]//2
        for i in range(0,1):
            children[i]=parents[i]
        for i in range(1,number):
            if i < number//2:
                children[i,0:crossover_point]=parent1[i%(number//2)][0:crossover_point]
                children[i,crossover_point:]=parent2[i%(number//2)][crossover_point:]
            else:
                children[i,0:crossover_point]=parent2[i%(number//2)][0:crossover_point]
                children[i,crossover_point:]=parent1[i%(number//2)][crossover_point:]
        #print(children)
        #print(parents)
        return children

    def mutation(self,children):
        mutated_children=children.copy()
        number=len(children)
        for i in range(1,number):
            index=int(np.random.uniform(0,no_of_weights,1))
            val=np.random.uniform(0,0.1,1)
            '''
            index=random.randrange(0,genes-1)
            val=random.uniform(-4,4)
            '''
            mutated_children[i][index]=val
        return mutated_children



# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('genetic_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        genetic_pub=rospy.Publisher('/pid_genetic_params',PID,queue_size=10)
        #ends

        #custom pid message
        gen_params=PID()
        gen_params.kp=[0,0,0,0,0,0]
        gen_params.ki=[0,0,0,0,0,0]
        gen_params.kd=[0,0,0,0,0,0]
        #ends

        genetic_obj_list=[GeneticNode(0),GeneticNode(1),GeneticNode(2),GeneticNode(3),GeneticNode(4),GeneticNode(5)]
        while not rospy.is_shutdown():
            #for i in range(0,6):
            #    print(genetic_obj_list[i].status)
            #print()
            if(genetic_obj_list[0].status==1 and genetic_obj_list[1].status==1 and genetic_obj_list[2].status==1 and genetic_obj_list[3].status==1 and genetic_obj_list[4].status==1 and genetic_obj_list[5].status==1):
                #print("HI")
                for i in range (0,6):
                    gen_params.kp[i]=genetic_obj_list[i].pid_vals[0]
                    gen_params.ki[i]=genetic_obj_list[i].pid_vals[1]
                    gen_params.kd[i]=genetic_obj_list[i].pid_vals[2]
                genetic_pub.publish(gen_params)
                for i in range(0,6):
                    genetic_obj_list[i].status=0

            #rospy.spin()
    except rospy.ROSInterruptException: pass
