from keras.models import model_from_json
import numpy
import euclid

import pygame.mixer
from pygame.locals import *
from pygame.key import *


import csv

from Car import Car
from Network import Network


# control_type_list # a list of integers. the length of the list is the number of cars. The number in the list is an enumerated type representig the control reigeme 
# 0: manual control
# 1: neural training #data from the car is written to file. 

# reinforcement flag
 
#log_data 0 no 1 yes
# If the reinforcement flag is 1, all data logged on all cars in the circut is recorded and written to file. 
# : reinforcementi



class Environment():
    def __init__(self, control_type_list, log_data_list, model_file, weights_file, obstacle_list, display_option):

        if len(control_type_list) != len(log_data_list):
            raise AttributeError( "control type and log data don't match")

        self.car_list = []
        self.network_list = []
        # Construct car list and network list
        for i in range(0,len(control_type_list)):

            if control_type_list[i] == 0:
                control_type = 'manual'
            elif control_type_list[i] ==1:
                control_type = 'neural'

            # the positions and orientations can be costomized later. 
            self.car_list.append(Car([1000,80], 1.0, 0.0, control_type, log_data_list[i]))
            self.network_list.append(Network.load(model_file, weights_file))
 
        self.obstacle_list = obstacle_list

        self.display_option = display_option

        #Limits for manual control
        self.min_steering = 0.0045
        self.max_steering = 0.009
        
        self.max_fitness_index = 0
        self.max_fitness = 0.0
        self.max_fitness_network = self.network_list[0]
        self.max_fitness_car = self.car_list[0]

        self.all_finished = False #indicates if all cars have either crashed or finished
        self.display_index = 0 #The index of the car to follow on screen
        self.total_time = 0.0

        self.quit = False

    def control(self):
        # the first car in the list is controlled manually, others by nn

        #needs to be an index because we are also indexing the network list. 
        for i in range(0,len(self.car_list)): 

            if self.car_list[i].control_type == 'manual':
            #print 'manual'
                if self.display_option==True:
                    control_list = [0,0,0]
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_a] ==True:
                        control_list[0] = 1
                    if keys[pygame.K_s] ==True:
                        control_list[0] = 1
                    if keys[pygame.K_d] ==True:
                        control_list[2] = 1
                    if keys[pygame.K_f] ==True:
                        control_list[2] = 1
                    #Test for no inputs, set straight steering
                    if all(value == 0 for value in control_list):
                        control_list[1] = 1
                    self.car_list[i].control_list(control_list)

            elif self.car_list[i].control_type == 'neural':
                if self.car_list[i].crashed != True and self.car_list[i].finishes <1:
                    prediction = self.network_list[i].predict(self.car_list[i].get_inputs())
                    print prediction
                    self.car_list[i].control_list(prediction)

        #Quits the game
        if self.display_option==True:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_x] == True:
                self.quit = True
            elif keys[pygame.K_c] == True:
                self.all_finished = True

    def update(self, time_delta):
        for i in range(0,len(self.car_list)):
            if self.car_list[i].crashed != True and self.car_list[i] != True:
                self.car_list[i].update(time_delta, self.obstacle_list)
        
        self.total_time += time_delta

    def display(self, screen_handle, screen_size):
        screen_vec = euclid.Vector3(screen_size[0], screen_size[1], 0.0)

        #Update the screen offset for display purposes    
        for i in range(0,len(self.car_list)):
            if self.car_list[i].crashed != True and self.car_list[i] != True:
                self.display_index = i
                break
        screen_offset_vec = 0.5 * screen_vec - self.car_list[self.display_index].position

        #Display each of the cars and associated lasers offset to the locatino of the first car in the list. 
        for i in range(0,len(self.car_list)):
            if i==self.display_index:
                self.car_list[i].display(screen_handle, screen_offset_vec, True)
            else:
                self.car_list[i].display(screen_handle, screen_offset_vec, False)

    def check_finish(self):

        if all((True == car.crashed or car.finishes>0) for car in self.car_list): 
            self.all_finished = True

        if self.total_time > 60:
            self.all_finished = True
        
        if self.all_finished:
            for i in range(0,len(self.car_list)):
                if self.car_list[i].log_data == True:
                    self.car_list[i].write_data('training_data.csv', 0)
