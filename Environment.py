from keras.models import model_from_json
import numpy
import euclid

import pygame.mixer
from pygame.locals import *
from pygame.key import *


import csv

from Car import Car
from Network import Network
import Functions

# control_type_list # a list of integers. the length of the list is the number of cars. The number in the list is an enumerated type representig the control reigeme 
# 0: manual control
# 1: neural training #data from the car is written to file. 

# reinforcement flag
 
#log_data 0 no 1 yes
# If the reinforcement flag is 1, all data logged on all cars in the circut is recorded and written to file. 
# : reinforcementi



class Environment():
    def __init__(self, num_cars, manual_control, model_file, weights_file, obstacle_list, display_option):


        self.car_list = []
        # Construct car list and network list
        for i in range(0,num_cars):

            if i==0 and manual_control>0:
                control_type = 'manual'
            else:
                control_type = 'neural' 

            # the positions and orientations can be costomized later. 
            self.car_list.append(Car([1000,80], 1.0, 0.0, control_type, True))

        self.network = Network.load(model_file, weights_file) 
        self.obstacle_list = obstacle_list

        self.display_option = display_option

        #Limits for manual control
        self.min_steering = 0.0045
        self.max_steering = 0.009

        self.manual_control = [0,0,0]

        self.all_finished = False #indicates if all cars have either crashed or finished
        self.display_index = 0 #The index of the car to follow on screen
        self.total_time = 0.0

        self.quit = False


    def get_user_input(self):
        #get all keys pressed
        keys = pygame.key.get_pressed()

        # Sets the quit flags
        if keys[pygame.K_x] == True:
            self.quit = True
        if keys[pygame.K_c] == True:
            self.all_finished = True
        if keys[pygame.K_v] == True:
            self.change_cars = True
        else:
            self.change_cars = False

        self.manual_control = [0,0,0]
        if keys[pygame.K_a] ==True:
            self.manual_control[0] = 1
        if keys[pygame.K_s] ==True:
            self.manual_control[0] = 1
        if keys[pygame.K_d] ==True:
            self.manual_control[2] = 1
        if keys[pygame.K_f] ==True:
            self.manual_control[2] = 1
        #Test for no inputs, set straight steering
        if all(value == 0 for value in self.manual_control):
            self.manual_control[1] = 1


    def control(self):


        # the first car in the list is controlled manually, others by nn

        #needs to be an index because we are also indexing the network list. 
        for car in self.car_list: 

            if car.control_type == 'manual':
                car.control_list(self.manual_control)

            elif car.control_type == 'neural':
                if car.crashed != True and car.finishes <1:
                    prediction = self.network.predict(car.get_inputs())
                    car.control_list(Functions.make_binary(prediction))


    def update(self, time_delta):
        for i in range(0,len(self.car_list)):
            if self.car_list[i].crashed != True and self.car_list[i] != True:
                self.car_list[i].update(time_delta, self.obstacle_list)
        
        self.total_time += time_delta

    def display(self, screen_handle, screen_size):
        car = self.car_list[self.display_index]
        if car.crashed or self.change_cars == True:
            self.display_index += 1

        if self.display_index > (len(self.car_list)-1):
            self.display_index = 0;

        screen_vec = euclid.Vector3(screen_size[0], screen_size[1], 0.0)

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
