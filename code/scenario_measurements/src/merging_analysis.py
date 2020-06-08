#!/usr/bin/env python
#above thing has to stay

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
from scipy.optimize import curve_fit
from scipy.stats import linregress
import csv
from os.path import dirname,abspath
import os

folder_name = "/merging_measurements"
cars = 5
################################################################################

def read_values(filename):
    # print(filename)
    all_data = dict()


    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        reader_numbered = enumerate(csv_reader)
        nr_rosbots = (len([row for id,row in reader_numbered if id == 1][0])-1)//3
    prev_timestep = "0"
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')

        for index,row in enumerate(csv_reader):
            if row[0] == prev_timestep:

                continue
            prev_timestep = row[0]
            if index == 0:
                header = [entry.strip() for entry in row]
                # print(len(header))

                all_data[header[0]]= []
                for i in range(nr_rosbots):
                    j = i *3
                    all_data[header[j+1]] = []
                    all_data[header[j+2]] = []
                    all_data[header[j+3]] = []
            else:
                all_data[header[0]].append(float(row[0]))
                try:
                    for i in range(nr_rosbots):
                        j = i *3
                        all_data[header[j+3]].append(map(float,row)[j+3])
                        all_data[header[j+1]].append(map(float,row)[j+1])
                        all_data[header[j+2]].append(map(float,row)[j+2])

                except Exception as e:
                    pass
    for column in header:
        all_data[column] = all_data[column][:len(all_data["yaw_fi"])]
    return all_data,header


def extract_data(dict_data, header):
    cols = ["Timestamp", "x_f", "y_f", "x_se", "y_se", "x_th", "y_th", "x_fo", "y_fo", "x_fi", "y_fi"]

    data = [dict_data[column] for column in cols]

    data = np.array(data)
    data = np.transpose(data)

    for i in range(4):
        dt = np.diff(data[:,0])
        timeskip = np.where(dt < 0)
        timeskip = timeskip
        data = np.delete(data, timeskip, axis = 0)

    return data

def distance_among_cars(all_data,header,nr_rosbots):
    rosbots = ['first','second','third','fourth','fifth']
    rosbots = rosbots[:nr_rosbots]
    distances = []
    cars = []
    for name in rosbots:
        car = Car(name)
        car.positions(all_data,header)
        cars.append(car)
    for i in range(len(cars)):
        for car in cars:
            if cars[i] != car:
                cars[i].distance_to_car(car)
    minimum_distances = []
    for car in cars:
        car.minimums()
        minimum_distances.append(car.absolute_minimum())
    sim_abs_minimum = min(minimum_distances)
    return sim_abs_minimum

class Car():
    def __init__(self,name):
        self.name = name
        rosbots = ['first','second','third','fourth','fifth']
        self.index = rosbots.index(name)*3 +1
        self.distances = []
        self.minima = []
    def positions(self,all_data,header):
        self.x_coords = np.array(all_data[header[self.index]])
        self.y_coords = np.array(all_data[header[self.index+1]])
    def distance_to_car(self,other):
        distance_dict = dict()
        x = (self.x_coords-other.x_coords)**2
        y = (self.y_coords-other.y_coords)**2
        distance_dict[other.name] = np.sqrt(x+y)
        self.distances.append(distance_dict)
    def minimums(self):
        for distance in self.distances:
            minimum_dict = dict()
            minimum_dict[distance.keys()[0]]= np.min(distance[distance.keys()[0]])
            self.minima.append(minimum_dict)
    def absolute_minimum(self):
        mini = 10
        for minimum in self.minima:
            if minimum[minimum.keys()[0]] < mini:
                mini  = minimum[minimum.keys()[0]]
                minimu = minimum
        return minimu


    def __repr__(self):
        return "rosbot " + self.name + " distances " + str([str(dic.keys()) for dic in self.distances])+"\n"\
        +"minima: " +str(self.minima)
    def __eq__(self,other):
        return self.name == other.name


    # def red_distance_used(self):
    #     for bots in range(len(rosbots)):
    #         #zoek in de y waardes van alle rosbots
    #         #if y >= -0.15:
    #         #   take the index
    #         #find the x value of that index


        pass
def extract_sim_id(filename):
    return int(filename[7:filename.index("-")])

def main():
    fail_ID = []
    efficiency_1 = []
    efficiency_2 = []
    dist_1 = []
    dist_2 = []
    total = 0

    print(__file__)
    for file in os.listdir(abspath(__file__[:-19]) + "/measurements" + folder_name):
        filename = __file__[:-19] + "measurements" + folder_name + "/" + file
        sim_id = extract_sim_id(file)
        sim_num = str(sim_id)

        total += 1

        data, header = read_values(filename)
        data_dict = data

        data = extract_data(data, header)

        x_vals = data[:, (1, 3, 5, 7, 9)]
        y_vals = data[:, (2, 4, 6, 8, 10)]

        end_mask_seperate = x_vals > 4
        end_mask = np.all(end_mask_seperate, axis = 1)
        T_end_index = next((i for i, j in enumerate(end_mask) if j), None)

        ##Determine the failed simulations
        #all that didn't start correctly
        if T_end_index == None:
            total += -1
            print("\033[91m Gazebo\t failure, please rerun ID: {}\033[0m".format(sim_num))
            continue

        #all that drove off the track
        elif np.any(np.abs(y_vals[T_end_index]) > 0.45):
            fail_ID.append(sim_num)

        #all that did not merge
        elif np.any(y_vals[T_end_index] < -0.15):
            fail_ID.append(sim_num)
        else:
            ##Determine the length efficiency
            #get the cars that started in the red lane
            reds = np.where(y_vals[0] < -0.15)[0]
            for k in range(len(reds)):
                merge_mask = y_vals[:, k] > -0.15
                merge_index = next((p for p, j in enumerate(merge_mask) if j), None)

                merge_x = x_vals[merge_index, k]
                local_efficiency = 1.0 - merge_x/6.0
                if sim_id <= 14:
                    efficiency_1.append(local_efficiency)
                else:
                    efficiency_2.append(local_efficiency)
        if sim_id <=14:
            # print(distance_among_cars(data_dict,header,5))
            dist_1.append(distance_among_cars(data_dict,header,5))
        else:
            dist_2.append(distance_among_cars(data_dict,header,5))




    for i in range(len(fail_ID)):
        if not fail_ID[i][1] == "-":
            fail_ID[i] = int(fail_ID[i])
        else:
            fail_ID[i] = int(fail_ID[i][0])

    # Distance analysis
    dist_1_values = np.array([dist.values()[0] for dist in dist_1])
    dist_2_values = np.array([dist.values()[0] for dist in dist_2])
    dist_1_avg = np.mean(dist_1_values)
    dist_1_std = np.std(dist_1_values)
    dist_2_avg = np.mean(dist_2_values)
    dist_2_std = np.std(dist_2_values)

    dists = np.append(np.array(dist_1_values),np.array(dist_2_values))
    dist_all_avg = np.mean(dists)
    dist_all_std = np.std(dists)
    efficiency_all = np.append(np.array(efficiency_1),np,array(efficiency_2))
    avg_eff_all = np.mean(efficiency_all)
    std_eff_all = np.std(efficiency_all)

    avg_dists = np.array([dist_1_avg,dist_2_avg,dist_all_avg])
    dist_std = np.array([dist_1_std,dist_2_std,dist_all_std])
    plt.figure(1)
    plt.errorbar(["1 vehicle","2 vehicles"],avg_dists,dist_std,capsize=10,fmt='o')
    plt.grid(axis="y")
    plt.ylim((0,0.6))
    plt.xlim((-0.5,1.5))

    plt.title("Minimum distance and amount of vehicles merged")
    plt.xlabel("Number of vehicles")
    plt.ylabel("Average of minimum distance per simulation [m]")

    efficiency = [np.mean(efficiency_1),np.mean(efficiency_2),avg_eff_all]
    eff_std = [np.std(efficiency_1),np.std(efficiency_2),std_eff_all]


    plt.figure(2)
    plt.errorbar(["1 vehicle","2 vehicles"],efficiency,eff_std,capsize=10,fmt='o')
    plt.grid(axis="y")
    plt.ylim((0,0.8))
    plt.xlim((-0.5,1.5))

    plt.title("Efficiency score and amount of vehicles merged")
    plt.xlabel("Number of vehicles")
    plt.ylabel("Efficiency score [-]")
    plt.show()

    # avg_dists = [sum(dist)/len(dist) for dist in sort_dists]
    # max_dists = [max(dist) for dist in sort_dists]
    # min_dists = [min(dist) for dist in sort_dists]
    #
    # err_dists = [np.std(np.array(dist)) for dist in sort_dists]
    #
    # plt.figure(2)
    # plt.title("Minimum distance and amount of vehicles passed")
    # plt.errorbar(["1 vehicle","2 vehicles"],avg_dists,yerr=err_dists,capsize=10,fmt='o')
    # plt.xlabel("Number of vehicles")
    # plt.ylabel("Average of minimum distance per simulation [m]")
    # plt.grid(axis="y")
    # plt.ylim((0,0.5))
    # plt.show()



    # print("\033[94m Succes rate was {:.6f} \n N = {}\n \033[0m".format(1.0- float(len(fail_ID))/total, total))
    # print("\033[92m Avg. efficiency: {:.3f} \t std.dev.: {:.3f}\033[0m".format(np.mean(efficiency), np.std(efficiency)))



#
#
#
#
#         car_index = cars-1
#
#         dist[car_index].append(distance_among_cars(data,header,5))
#
#
#     avg_dists = [sum(dist)/len(dist) for dist in sort_dists]
#     err_dists = [np.std(np.array(dist)) for dist in sort_dists]
#
#     plt.figure(1)
#     plt.title("Minimum distance")
#     plt.bar([" "],avg_dists,yerr=err_dists,capsize=10)
#
#     plt.xlabel("Number of vehicles")
#     plt.ylabel("Average of minimum distance per simulation [m]")
#     plt.show()
#
#     plt.figure(2)
#     plt.title("Distance used on the red lane")
#
#
if __name__ == "__main__":
    main()
