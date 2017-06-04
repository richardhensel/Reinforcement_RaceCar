import math, pygame, pygame.mixer
import euclid
import Functions
from pygame.locals import *

import csv

class Car():

    def __init__(self, position_list, orientation, velocity, control_type, log_data):

        # Dynamics
        self.position = euclid.Vector3(position_list[0], position_list[1],0.0)  # location vectors
        self.prev_position = self.position
        self.orientation = euclid.Vector3(orientation, 0.0, 0.0)  # orientation vector unit vector centered at the car's origin. 
        self.velocity = velocity             # rate of change of positon in direction of travel ms-1
        self.steering =  0.0         #rate of change of yaw with respect to velocity rad/pixel -ve left, +ve right

        #Limits    
        self.velocity = 300

        #Define the control input
        self.num_steering_speeds = 1
        self.steering_increment = 0.0045
        #Create the array of steering rates
        if self.num_steering_speeds ==1:
            self.steering_array = [-1.5*self.steering_increment, 0, 1.5*self.steering_increment]
        elif self.num_steering_speeds ==2:   
            self.steering_array = [-1*self.max_steering, -1*self.min_steering, 0, self.min_steering, self.max_steering]
        else:
            raise AssertionError('Steering increment must be set to 1 or 2')

        self.control_bools = []

        # Trip metrics
        self.dist_travelled = 0.0       # total distance travelled
        self.total_time = 0.0
        self.avg_velocity = 0.0         # average velocity for the trip
        self.crashed = False            # did the car crash into an obstacle?
        self.finishes = 0            # did the car pass through the finish line? 
        self.fitness = 0.0             

        #Define the car geometry
        self.length = 30   #pixels
        self.width = 20    #pixels 
        self.origin_dist = 0.25 * self.length #Origin is 1 quarter of the length from the rear of the car
        self.color = 0,0,0              #black
        self.line_width = 3

        #Define the sensor array
        self.sensor_origin_dist = 0.5
        self.max_sensor_length = 500.0
        self.num_sensors = 15
        self.sensor_ranges = [self.max_sensor_length] * self.num_sensors


        self.screen_offset_vec = euclid.Vector3(0.0, 0.0, 0.0)

        self.finish_line = self.__set_finish_line()
        self.control_type = control_type # string either manual or neural
        self.log_data = log_data # bool either 1 or 0

        self.data_log = []

    def get_inputs(self):
        inputs = []
        #Inputs
        for s_range in self.sensor_ranges:
            inputs.append(Functions.truncate(s_range,2))
        return inputs 

    def control_list(self, control_list):
        if len(control_list) != len(self.steering_array):
            raise AttributeError( 'incorrect number of elements in steering list. ', str(len(control_list)), ' to ', str(len(self.steering_array)))
        if self.crashed == False:
            max_control = 0
            index = 0
            for i in range(0, len(control_list)):
                if control_list[i] > max_control:
                    self.steering = self.steering_array[i] 
                    index = i

        self.control_bools = control_list[:]
    
    def update(self, time_delta, obstacles):
        self.__reset_sensors()
        self.__update_dynamics(time_delta)
        self.__update_geometry()

        for obstacle in obstacles:
            self.__sense(obstacle)
            self.__detect_collision(obstacle)
        self.__detect_finish_line()

        if self.log_data ==True:
            self.__log_data(False)

    def display(self, screen_handle, screen_offset_vec, draw_vectors = True):
        #Compute offsets

        pos = self.position + screen_offset_vec

        bl = self.rear_left + screen_offset_vec #back left
        fl = self.front_left + screen_offset_vec
        fr = self.front_right + screen_offset_vec
        br = self.rear_right + screen_offset_vec #back right
        
        so = self.sensor_origin + screen_offset_vec
    
        sensor_vectors = self.__get_sensor_vectors(self.sensor_ranges)
        sv = []
        for line in sensor_vectors:
            sv.append((line[0]+screen_offset_vec, line[1]+screen_offset_vec))

        #Draw car

        #Draw the offset points
        point_list = [(bl.x, bl.y), (fl.x, fl.y), (fr.x, fr.y), (br.x, br.y)]
        pygame.draw.lines(screen_handle, self.color, True, point_list, self.line_width)

        #Draw origin
        pygame.draw.circle(screen_handle, (255,0,0), (int(pos.x),int(pos.y)), 2, 0)
        #Draw sensor origin
        #pygame.draw.circle(screen_handle, (0,0,255), (int(so.x), int(so.y)), 2, 0)

        #Draw sensor vectors
        if draw_vectors == True:
            sensor_list = []
            for line in sv:
                point_list = [(line[0].x, line[0].y),(line[1].x, line[1].y)]
                pygame.draw.lines(screen_handle, (0,0,255), False, point_list, 1)
                sensor_list.append((int(line[1].x), int(line[1].y)))

                #Draw the intersection if detected
                pygame.draw.circle(screen_handle, (255,0,0), (int(line[1].x), int(line[1].y)), 2, 0)
            pygame.draw.lines(screen_handle, (0,0,0), False, sensor_list, self.line_width)
                

    def write_data(self, file_name, overwrite):
        #Write data set to csv
        if overwrite:
            option = 'w'
        else:
            option = 'a'
        #Append the crash status to each line of the data log
        for line in self.data_log:
            if self.crashed:
                line.append(0)
            else:
                line.append(1)

        with open(file_name, option) as f:
            writer = csv.writer(f)
            for line in self.data_log:
                writer.writerow(line)
        print "Training data written to file"



    def __reset_sensors(self):
        self.sensor_ranges = [self.max_sensor_length] * self.num_sensors

    def __update_dynamics(self, time_delta):
        # stay still if crashed
        if self.crashed ==True:
            self.velocity = 0.0

        # store previous position for distance accumulation
        self.prev_position = self.position.copy()

        # update pose
        self.orientation = self.orientation.rotate_around(euclid.Vector3(0.,0.,1.),self.steering * self.velocity * time_delta)
        self.position += self.velocity * self.orientation * time_delta
        
        # Accumulate trip metrics
        if self.crashed == False: 
            self.dist_travelled += math.copysign(abs(self.position - self.prev_position), self.velocity)
            self.total_time += time_delta
            #update the fitness
            self.fitness = 2.0*self.dist_travelled + self.avg_velocity

    def __update_geometry(self):
    #Geometry points
        self.rear_left = self.position + self.orientation.rotate_around( euclid.Vector3(0.,0.,1.), -0.5*math.pi)* self.width/2 - self.orientation * self.origin_dist
        self.rear_right = self.position + self.orientation.rotate_around( euclid.Vector3(0.,0.,1.), 0.5*math.pi)* self.width/2  - self.orientation * self.origin_dist
        self.front_left = self.rear_left + self.orientation * self.length
        self.front_right = self.rear_right + self.orientation * self.length
        self.sensor_origin = self.position + self.sensor_origin_dist * self.length * self.orientation # sensor origin is 1 quarter of the length from the front of the car

    def __sense(self, obstacle):
        sensor_vectors = self.__get_sensor_vectors([self.max_sensor_length] * self.num_sensors)
        for i in range(0,len(sensor_vectors)):
            intersect_list = []

            ray = sensor_vectors[i]
            #unpack the start and end points of the sensor ray 
            p1 = (ray[0].x, ray[0].y)
            p2 = (ray[1].x, ray[1].y)
            for j in range(0,len(obstacle)):
                if j ==0:
                    p3 = obstacle[-1]
                else:
                    p3 = obstacle[j-1]
                p4 = obstacle[j]
                #Get intersection point in global frame
                ix, iy = Functions.get_line_intersection(p1, p2, p3, p4)
                #Append all non-null intersections to the list of intersections for that ray
                if ix != None and iy != None:
                    intersect_list.append(euclid.Vector3(ix,iy, 0.0))
            # find the minimum distance. range must be shorter than the current stored min range to be stored. 
            for vector in intersect_list:
                measured_range = abs(vector - ray[0])
                if measured_range < self.sensor_ranges[i]:
                    self.sensor_ranges[i] = measured_range
        #print self.sensor_ranges

    def __detect_collision(self, obstacle):
        car_bounds = [self.rear_left, self.front_left, self.front_right, self.rear_right]
        for i in range(0,len(car_bounds)):
            if i ==0:
                p1 = (car_bounds[-1].x, car_bounds[-1].y)
            else:
                p1 = (car_bounds[i-1].x, car_bounds[i-1].y)
            p2 = (car_bounds[i].x, car_bounds[i].y)

            for j in range(0,len(obstacle)):
                if j ==0:
                    p3 = obstacle[-1]
                else:
                    p3 = obstacle[j-1]
                p4 = obstacle[j]
               
                #Get intersection point in global frame
                ix, iy = Functions.get_line_intersection(p1, p2, p3, p4)
                if ix != None or iy != None:
                    self.crashed = True
                    return 0
                    
    def __detect_finish_line(self):
        # define a line protruding from the front of the car which is used to detect a line crossing. 
        finish_bar = self.sensor_origin + self.orientation * 20.0
        p1 = (self.sensor_origin.x, self.sensor_origin.y)
        p2 = (finish_bar.x, finish_bar.y)

        for j in range(0,len(self.finish_line)):
            if j ==0:
                p3 = self.finish_line[-1]
            else:
                p3 = self.finish_line[j-1]
            p4 = self.finish_line[j]
           
            #Get intersection point in global frame
            ix, iy = Functions.get_line_intersection(p1, p2, p3, p4)
            if ix != None or iy != None:
                self.finishes +=1
                return 0
        
        #Test for collision between the obstacle and each of the corners/lines of the car
        #self.crashed = True

    def __log_data(self, scale_inputs):
        # Scale the data to values between -1 and 1. append to list with one row per time step
        current_data = []
        #Inputs
        if scale_inputs:
            for s_range in self.sensor_ranges:
                current_data.append(Functions.truncate(Functions.translate(s_range, 0.0, self.max_sensor_length, 0.0, 1.0),2))

        else:
            for s_range in self.sensor_ranges:
                current_data.append(Functions.truncate(s_range,2))


        #Outputs
            for item in self.control_bools:
                current_data.append(item)

        self.data_log.append(current_data)
    
    def __get_sensor_vectors(self, ray_lengths):
        angle = math.pi / (self.num_sensors-1)
        vectors = []
        count = 0
        for i in range(0,self.num_sensors):
            p1 = self.sensor_origin
            # sensor vector is count*angle radians from the right of the car, with an origin at sensor origin and a length of sensor_length 
            p2 = self.sensor_origin + self.orientation.rotate_around(euclid.Vector3(0.,0.,1.), 0.5*math.pi - count * angle) * ray_lengths[i]

            vectors.append((p1,p2)) 
            count +=1
        return vectors

    # set the finish line for the car. The finish line is a line 200 units long, perpendicular to the length of the car, starting at the origin. 
    def __set_finish_line(self):

        finish_left = self.position + self.orientation.rotate_around( euclid.Vector3(0.,0.,1.), -0.5*math.pi)* 100
        finish_right = self.position + self.orientation.rotate_around( euclid.Vector3(0.,0.,1.), 0.5*math.pi)*100

        return [(finish_right.x, finish_right.y), (finish_left.x, finish_left.y)]
