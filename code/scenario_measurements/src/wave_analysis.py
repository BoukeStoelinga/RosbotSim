#!/usr/bin/env python
#above thing has to stay

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
from scipy.optimize import curve_fit
from scipy.stats import linregress
import csv
from os.path import dirname,abspath


#define filenames to be analysed
filenames = ["5_cars_vel_0.1.csv", "5_cars_vel_0.15.csv", "5_cars_vel_0.2.csv", "5_cars_vel_0.25.csv"]

################################################################################

def read_values(filename):
    '''
    Extract values from csv file to dictionary
    '''
    print(filename)
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
    '''
    convert data from dictionary to numpy array and delete faulty datapoints
    '''
    cols = ["Timestamp", "x_f", "x_se", "x_th", "x_fo", "x_fi"] #selected collomns and order

    data = [dict_data[column] for column in cols]

    data = np.array(data)
    data = np.transpose(data)

    data = data[150:]

    for i in range(4):  #remove datapoints where time skips backwards
        dt = np.diff(data[:,0])
        timeskip = np.where(dt < 0)
        timeskip = timeskip
        data = np.delete(data, timeskip, axis = 0)

    return data

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = sig.butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = sig.lfilter(b, a, data, axis = 0)
    return y

def fit_func(t, c, lam):
    y = c * np.exp(lam*(t-9)) + 0.16
    return y

################################################################################
p=0
for fi in filenames: #per file analyse and plot
    plt.figure(figsize = (6.4*1.25, 4.8*1.25))
    plt.rcParams.update({'font.size': 19})
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.xlim(5, 35)
    plt.ylim(0.13, 0.52)

    f = abspath(__file__[:-16] + "measurements/platooning_measurements/" + fi)
    dict_data, header = read_values(f)
    data = extract_data(dict_data, header)
    time = data[:, 0] - data[1, 0]

    # Filter requirements.
    order = 6
    fs = 18.0       # sample rate, Hz
    cutoff = 0.3 # desired cutoff frequency of the filter, Hz



    k=0
    for ns in ["first"]:#, "second", "third", "fourth", "fifth"]:
        '''
        Calculate and plot different measurements; comment and uncomment what is needed
        '''
        vel = np.diff(data[:, k+1]) * fs

        vel_filtered = butter_lowpass_filter(vel, cutoff, fs, order)
        peaks_indices, z = sig.find_peaks(vel_filtered[150:350])
        peaks = vel_filtered[peaks_indices+150]
        peaks_time = time[peaks_indices+150]

        opt, pcov = curve_fit(fit_func, peaks_time, peaks)
        if k == 0:
            print(ns+ "_" + fi[-7:-4] + ":\t local lambda = " + str(opt[1]))

        plt.plot(time[:-1], vel_filtered, label = ns)
        plt.plot(peaks_time, peaks, "rx")
        plt.plot(time[:-1], fit_func(time[:-1], opt[0], opt[1]), "g", label = "exp through peaks")
        print("c_1 = {:.3} \t lambda = {:.3f}".format(opt[0], opt[1]))
        k += 1

    plt.title("Filtered dampening curve (v_{max} = " + str(0.15+0.05*p) + ")")
    p += 1

################################################################################
plt.legend()
plt.show()
