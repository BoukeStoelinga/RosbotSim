#!/usr/bin/env python

import csv
import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import os
from os.path import dirname,abspath
# d = dirname(dirname(abspath(__file__)))
folder_name = "/overtaking_measurements"
print(dirname(dirname(abspath(__file__[:-12]+"/measurements"+folder_name))))


def read_values(filename):
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
                print(len(header))

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
                    print(e)
    return all_data,header

def plot_data(data):
    plt.plot(data["x_f"],data["y_f"])
    plt.show()




def main():
    for file in os.listdir(__file__[:-12]+"/measurements"+folder_name):
        filename = __file__[:-12]+"measurements"+folder_name + "/"+file
        data,header = read_values(filename)
        filename = __file__[:-12]+"measurements"+folder_name +"_videos"+"/"+file[:-4]+".mp4"
        an = Animation(data,header,filename)
        an.do_animation()


    # print(print(data["x_f"])

    # plot_data(data)

class Animation:
    def __init__(self,data,header,filename):
        self.nr_cars = (len(data)-1)//3
        self.data = data
        self.header = header
        self.nr_frames = len(self.data["x_f"])-1 #[0::33]
        self.filename = filename

        print(self.nr_frames)
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-10, 2), ylim=(-5, 5), ylabel='Y-coordinate [m]', xlabel='X-coordinate [m]', title='Simulation representation') # Change scales
        self.lines = [self.ax.plot([], [], lw=1)[0] for i in range(2*self.nr_cars)]
        self.squares = [self.ax.plot([], [], linewidth=0.1,color="black")[0] for i in range(self.nr_cars)]

        self.line, = self.ax.plot([], [], lw=0.1)

        self.line2, = self.ax.plot([], [], linewidth=0.01)
        self.xdata, self.ydata = [], []
        self.xdata2, self.ydata2 = [], []
        self.setdata = [[] for i in range(self.nr_cars*2)]
        self.progress = 0
        # self.line.set_data([], [])
    def init(self):
        for line in self.lines:
            line.set_data([], [])
        self.line.set_data([], [])
        self.line2.set_data([], [])
        plt.plot([-10, 10], [0.49, 0.49], dashes = [5,2], color = 'black', lw =0.4)
        plt.plot([-10, 10], [0.17, 0.17], dashes = [5,2], color = 'black',lw =0.4)
        plt.plot([-10, 10], [-0.17, -0.17], dashes = [5,2], color = 'black',lw =0.4)
        plt.plot([-10, 6.5], [-0.49, -0.49], dashes = [5,2], color = 'black',lw =0.4)
        plt.plot([6.5, 7.5], [-0.49, -0.17], dashes = [5,2], color = 'black',lw =0.4)

        return self.lines

    def animate(self,i):
        """

        """

        namelist = self.header[1:3*self.nr_cars]
        yawlist = self.header[1:3*self.nr_cars+1]
        yawlist = yawlist[3-1::3]
        # yawlist = namelist[3-1::3]+[namelist[-1]]




        del namelist[3-1::3]


        for j in range(len(namelist)):
            self.setdata[j].append(self.data[namelist[j]][i]) #[j]][0::33][i]


        for k in range(len(self.lines)//2):
            self.lines[k].set_data(self.setdata[2*k],self.setdata[2*k+1])
            car_x = self.setdata[2*k]
            car_y = self.setdata[2*k+1]
        #
        for l in range(self.nr_cars,self.nr_cars*2):
            car_x = self.setdata[(l-self.nr_cars)*2][-1]
            car_y = self.setdata[(l-self.nr_cars)*2+1][-1]

            car_yaw = self.data[yawlist[l-self.nr_cars]][i] # [0::33][i]

            x_array = [\
            car_x+0.11*np.cos(car_yaw)-0.11*np.sin(car_yaw),
            car_x-0.11*np.cos(car_yaw)-0.11*np.sin(car_yaw),
            car_x-0.11*np.cos(car_yaw)+0.11*np.sin(car_yaw),
            car_x+0.11*np.cos(car_yaw)+0.11*np.sin(car_yaw),
            car_x+0.11*np.cos(car_yaw)-0.11*np.sin(car_yaw)]
            y_array = [\
            car_y + 0.11*np.sin(car_yaw)+0.11*np.cos(car_yaw),
            car_y - 0.11*np.sin(car_yaw)+0.11*np.cos(car_yaw),
            car_y - 0.11*np.sin(car_yaw)-0.11*np.cos(car_yaw),
            car_y + 0.11*np.sin(car_yaw)-0.11*np.cos(car_yaw),
            car_y + 0.11*np.sin(car_yaw)+0.11*np.cos(car_yaw)]

            self.lines[l].set_data(x_array,y_array)

        return self.lines

    def do_animation(self):
        anim = FuncAnimation(self.fig, self.animate, init_func=self.init,
                               frames=self.nr_frames, interval=20, blit=True)
        print("_"*50)
        print(self.filename)
        anim.save(self.filename,"ffmpeg")


if __name__ == "__main__":
    main()
