#!/usr/bin/env python

import csv
import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import scipy.signal
import os
import math
from os.path import dirname, abspath
from scipy.signal import savgol_filter

folder_name = "/overtaking_measurements"


def read_values(filename):
    # print("reading file: {}".format(filename))
    all_data = dict()

    # read out number of rosbots
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        reader_numbered = enumerate(csv_reader)
        nr_rosbots = (
            len([row for id, row in reader_numbered if id == 1][0]) - 1) // 3

    prev_timestep = "0"
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        # skip duplicate timesteps
        for index, row in enumerate(csv_reader):
            if row[0] == prev_timestep:
                continue
            prev_timestep = row[0]

            # read out the header and assign dicitonary keys
            if index == 0:
                header = [entry.strip() for entry in row]
                all_data[header[0]] = []
                for i in range(nr_rosbots):
                    j = i * 3
                    all_data[header[j + 1]] = []
                    all_data[header[j + 2]] = []
                    all_data[header[j + 3]] = []
            # append the data to the correct dicitionary keys
            else:
                all_data[header[0]].append(float(row[0]))
                try:
                    for i in range(nr_rosbots):
                        j = i * 3
                        all_data[header[j + 3]].append(map(float, row)[j + 3])
                        all_data[header[j + 1]].append(map(float, row)[j + 1])
                        all_data[header[j + 2]].append(map(float, row)[j + 2])

                except Exception as e:
                    pass
                    # print(e)
    # delete last (incomplete) row
    for column in header:
        all_data[column] = all_data[column][:len(all_data["yaw_fi"])-40]
    return all_data, header


def get_timesteps(all_data):
    timestamp = all_data["Timestamp"]
    delta_time = [timestamp[j] - timestamp[j - 1]
                  for j in range(len(timestamp[1:]))]
    return delta_time


def calculate_velocities(all_data, header, timesteps):
    velocities = []
    times = []
    for i in range(1, len(header), 3):
        x_rosbot, y_rosbot = all_data[header[i]], all_data[header[i + 1]]

        delta_x = [x_rosbot[j] - x_rosbot[j - 1]
                   for j in range(len(x_rosbot[1:]))]
        delta_y = [y_rosbot[j] - y_rosbot[j - 1]
                   for j in range(len(y_rosbot[1:]))]

        delta_distance = [math.sqrt((delta_x[j]**2 + delta_y[j]**2))
                          for j in range(len(delta_x))]
        velocity = []
        time = []
        for j, timestep in enumerate(timesteps[1:]):
            if timestep > 0.001 and all_data["Timestamp"][j] > all_data["Timestamp"][j - 1]:
                vel = (delta_distance[j - 1]) / timestep
                if vel < 1.5:
                    velocity.append(vel)
                    time.append(all_data["Timestamp"][j])
        times.append(time)
        velocities.append(velocity)
    return velocities, times


def plot_vel(vel, data, times):
    """
    for debugging shows filtered and unfiltered results
    """
    starttime = times[0]
    time = [time_ - starttime for time_ in times[1:]]
    plt.plot(time, vel, label="unfiltered")
    plt.plot(time, scipy.signal.convolve(vel, np.ones(
        (9,)) / 9, 'same'), label="convolution filtered")
    new_vel = scipy.signal.convolve(vel, np.ones((9,)) / 9, 'same')
    # https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filterl
    filtered_vel = savgol_filter(new_vel, 51, 3)
    print(np.mean(filtered_vel))
    plt.plot(time, filtered_vel, label="conv and savgol filtered")
    plt.legend()
    plt.show()


def plot_velocities(velocities, times):

    labels = ["first", "second", "third", "fourth", "fifth"]
    plt.figure(1)
    for j, vel in enumerate(velocities):
        starttime = times[j][0]
        time = [time_ - starttime for time_ in times[j]]
        print(len(vel))
        plt.plot(time, vel, label=labels[j])
    plt.title("filtered  absolute velocities")
    plt.xlabel("time [s]")
    plt.ylabel("|v| [ms^-1]")
    plt.legend()
    plt.show()


def filter_vel(vel):
    convolve = scipy.signal.convolve(vel, np.ones((9,)) / 9, 'same')
    filtered_vel = savgol_filter(convolve, 51, 3)
    return filtered_vel


def calculate_means(velocities):
    average_vels = []
    for vel in velocities:
        average_vels.append(sum(vel) / len(vel))
    return average_vels


def plot_mean_velocities(average_vels):
    # fig = plt.figure(2)
    # ax = fig.add_axes([0,0,1,1])
    labels = ["first", "second", "third", "fourth", "fifth"]
    plt.bar(labels, average_vels)
    plt.show()


def plot_results(rel_vels, desired_vels, filename):
    labels = ["first", "second", "third", "fourth", "fifth"]
    nr_relevant_cars = 5 - desired_vels.count(0)
    labels = labels[:nr_relevant_cars]
    rel_vels = rel_vels[:nr_relevant_cars]
    plt.bar(labels, rel_vels)
    plt.title(filename[:-4].replace("_", " "))
    plt.show()


def desired_vels(filename):
    platoon_vel = 0.3
    nr_cars_to_pass = int(filename[0])
    end_to_search = filename[-12:]
    float_string = ""
    for i in range(len(end_to_search)):
        if end_to_search[i].isdigit():
            start_index = i
            break
    float_vel = float(end_to_search[start_index:-4])
    desired_vels = [float_vel] + nr_cars_to_pass * \
        [platoon_vel] + (4 - nr_cars_to_pass) * [0]
    nr_relevant_rosbots = nr_cars_to_pass+1
    return desired_vels, nr_relevant_rosbots

def bar_names(filename):
    end_to_search = filename[-12:]
    for i in range(len(end_to_search)):
        if end_to_search[i].isdigit():
            start_index = i
            break
    vel = end_to_search[start_index:-4]
    cars  = int(filename[0])
    return cars, vel


def relative_vels(means, desired_velos):
    relative_vels = []
    for i in range(len(means)):
        if desired_velos[i] != 0:
            relative_vels.append(means[i] / desired_velos[i])
        else:
            pass
            # relative_vels.append(1)

    return relative_vels,sum(relative_vels)/len(relative_vels)
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



def main():
    files = [[],[],[]]
    scores = [[],[],[]]
    dist = [[],[],[]]
    for file in os.listdir(__file__[:-22] + "/measurements" + folder_name):
        desired_vels_,nr_rosbots = desired_vels(file)
        filename = __file__[:-22] + "measurements" + folder_name + "/" + file
        data, header = read_values(filename)
        timesteps = get_timesteps(data)
        velocities, times = calculate_velocities(
            data, header, timesteps)
        velocities = [filter_vel(vel) for vel in velocities]
        means = calculate_means(velocities)
        rel_vels,score = relative_vels(means, desired_vels_)
        cars,vel =bar_names(file)
        car_index = cars-1

        files[car_index].append(vel)
        scores[car_index].append(score)
        dist[car_index].append(distance_among_cars(data,header,nr_rosbots))
    sort_scores = []
    sort_files = []
    sort_dists = []
    for i in range(3):
        sort_index = sorted(range(len(map(float,files[i]))), key=lambda k: map(float,files[i])[k])
        # print(sort_index)
        sort_score = []
        sort_file = []
        sort_dist = []
        for index in sort_index:
            # print(dist[i][index].values()[0])
            sort_dist.append(dist[i][index].values()[0])
            sort_file.append(files[i][index])
            sort_score.append(scores[i][index])
            if index == 2:
                sort_dist.append(0.4)
        sort_scores.append(sort_score)

        sort_files.append(sort_file)
        sort_dists.append(sort_dist)
    avg_scores = [sum(score)/len(score) for score in sort_scores]
    max_scores = [max(score) for score in sort_scores]
    min_scores = [min(score) for score in sort_scores]
    err_scores = [np.std(np.array(score))*1.3 for score in sort_scores]
    print(err_scores)

    plt.figure(1)
    plt.errorbar(["1 vehicle","2 vehicles","3 vehicles"],avg_scores,yerr=err_scores,capsize=10,fmt='o')
    plt.grid(axis="y")
    plt.ylim((0.6,1))

    plt.title("Velocity score and amount of vehicles passed")
    plt.ylabel("average velocity score [-]")
    plt.xlabel("number of vehicles")


    avg_dists = [sum(dist)/len(dist) for dist in sort_dists]
    max_dists = [max(dist) for dist in sort_dists]
    min_dists = [min(dist) for dist in sort_dists]

    err_dists = [np.std(np.array(dist)) for dist in sort_dists]

    plt.figure(2)
    plt.title("Minimum distance and amount of vehicles passed")
    plt.errorbar(["1 vehicle","2 vehicles","3 vehicles"],avg_dists,yerr=err_dists,capsize=10,fmt='o')
    plt.xlabel("Number of vehicles")
    plt.ylabel("Average of minimum distance per simulation [m]")
    plt.grid(axis="y")
    plt.ylim((0,0.5))
    plt.show()

        # plt.figure(1)
        # plt.bar(sort_file,sort_score)
        # if i == 0:
        #     plt.title("Speed scores overtaking {} vehicle".format(str(i+1)))
        # else:
        #     plt.title("Speed scores overtaking {} vehicles".format(str(i+1)))
        # plt.ylabel("velocity score [-]")
        # plt.xlabel("velocity overtaking vehicle [m/s]")
        #
        # plt.figure(2)
        # plt.bar(sort_file,sort_dist)
        # plt.xlabel("velocity overtaking vehicle [m/s]")
        # plt.ylabel("Minimum distance between two vehicles [m]")
        # if i == 0:
        #     plt.title("Minimum distance overtaking {} vehicle".format(str(i+1)))
        # else:
        #     plt.title("Minimum distance overtaking {} vehicles".format(str(i+1)))
        # plt.show()
    # plt.bar(files[1],scores[1])
    # plt.title("Speed scores overtaking 2 vehicles")
    # plt.show()
    # plt.bar(files[2],scores[2])
    # plt.title("Speed scores overtaking 3 vehicles")
    # plt.xticks(rotation='vertical')
    # plt.show()
        # plot_results(rel_vels, desired_vels_, file)

        # plot_velocities(velocities, times)
        # plot_vel(velocities[0], data, times)


if __name__ == "__main__":
    main()
