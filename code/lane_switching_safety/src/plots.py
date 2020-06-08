#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sys
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import sklearn.cluster as cluster
import matplotlib.lines as mlines


if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = str(sys.argv[1])

class DetectedVehicle():
    def __init__(self,namespace,plot=False):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'lane_switching_safety_node', anonymous=True)
        self.pub_flag = rospy.Publisher(self.namespace+ "/lane_switching_flag", UInt8MultiArray, queue_size=1)
        self.pub_objects = rospy.Publisher(self.namespace+ "/detected_objects", Float64MultiArray, queue_size=1)
        self.flag = [False, False]
        self.flag_history = np.zeros((5, 2))
        self.plot = plot
        self.cluster_centers = np.array([])
        self.previous_centers = np.array([])
        self.rel_vels_linked = np.array([])
        self.pub_rel_vels_linked = np.array([])
        self.current_time = rospy.get_time()
        self.safety_margin_min = -0.4
        self.safety_margin_max = 0.25

        self.lane_width = 0.3
        self.threshold = 0.2
        self.time_threshold = 0.5 #seconds

    def subscribers(self):
        """
        Excecute subscribers
        """
        rospy.Subscriber(self.namespace+'/scan', LaserScan, self.callback)
        rospy.spin()
        if self.plot == True:
            plt.show()

    def callback(self,data):
        # read sensor values
        ranges = np.array(data.ranges)
        coordinates = self.convert_xy(ranges)
        self.flag = [False,False]

        try:
            #Calculate clusters and relative velocities
            self.cluster(coordinates)
            try:
                self.calc_rel_vels()
                self.flag = self.decision_safety()
            except:
                self.flag = [True,True]

            # plot if values are found
            if self.plot == True:
                self._plot()

        except Exception as e:
            # If no points are found, or all are filtered
            pass
        self.publish()




    def decision_safety(self):

        # seperate into left and right lane current positions
        left_indices = np.where(self.nearest_points[:,1] > 0)
        left_cars = self.nearest_points[left_indices]

        right_indices = np.where(self.nearest_points[:,1] < 0)
        right_cars = self.nearest_points[right_indices]

        # Seperate into left and right lane future predictions
        safety_projection = self._get_projection(self.time_threshold)

        left_safety_projection_indices = np.where(safety_projection[:,1] > 0)
        left_safety_projection_cars = safety_projection[left_safety_projection_indices]

        right_safety_projection_indices = np.where(safety_projection[:,1] < 0)
        right_safety_projection_cars = safety_projection[right_safety_projection_indices]

        # get a list of left and right x coordinates
        left_x = np.zeros((len(left_cars)+len(left_safety_projection_cars)))
        left_x[0:len(left_cars)] = left_cars[:,0]
        left_x[len(left_cars):] = left_safety_projection_cars[:,0]
        right_x = np.zeros((len(right_cars)+len(right_safety_projection_cars)))
        right_x[0:len(right_cars)] = right_cars[:,0]
        right_x[len(right_cars):] = right_safety_projection_cars[:,0]

        # check if current and predicted x coordinates are outsite of the safety margin
        left_safety = all([(x<self.safety_margin_min or x>self.safety_margin_max) for x in left_x])
        right_safety = all([(x<self.safety_margin_min or x>self.safety_margin_max) for x in right_x])
        if self.plot: print("\n" + self.namespace + "\t Left flag: \t" + str(left_safety) + "\n\t Right flag: \t" + str(right_safety))
        return [left_safety, right_safety]


    def cluster(self,data):
        """
        https://scikit-learn.org/stable/modules/clustering.html#clustering
        Using DBSCAN clustering, dynamic clusters are trained,
        """
        Kmeans = cluster.DBSCAN(eps=0.2, min_samples=5).fit(data)

        labeled = np.ones((len(Kmeans.labels_),3))
        labeled[:,:2] = data
        labeled[:,-1] = Kmeans.labels_[np.newaxis, :]
        self.labeled = labeled
        self.nr_clusters = max(Kmeans.labels_)

        #get cluster and nearest points
        self._cluster_centers(labeled,Kmeans.labels_)
        self._get_nearest_points(labeled,Kmeans.labels_)

    def _plot(self):
        # plot what is seen
        plt.clf()
        for clus in range(self.nr_clusters+1):
            local_cluster = self.labeled[np.where(self.labeled[:,2] == clus)]
            plt.plot(local_cluster[:,0],local_cluster[:,1],"x")
            plt.xlim((-2, 2))
            plt.ylim((-1.5, 1.5))
            plt.grid('on')

        #plot all data (including unclustered)
        # plt.plot(self.labeled[:,0],self.labeled[:,1],"x",label="measured points filtered")
        # #plot cluster centers
        centers = plt.plot(self.cluster_centers[:,0],self.cluster_centers[:,1],"o",label="Cluster centers")[0]
        # #plot nearest_points
        closest = plt.plot(self.nearest_points[:,0],self.nearest_points[:,1],".",label="Closest point")[0]

        # plot projected_centers
        # plt.plot(self.projected_centers[:,0],self.projected_centers[:,1],"Dk",label="Projected centers")

        # plot middle rosbot
        rosbot = plt.plot([-0.1,0.1,0.1,-0.1,-0.1],[-0.11,-0.11,0.11,0.11,-0.11],'g',label="scanning rosbot")[0]
        plt.arrow(0,0,0.4,0,head_width=0.05)
        plt.title("Vehicle detection")
        plt.xlabel("relative x coordinate [m]")
        plt.ylabel("relative y coordinate [m]")
        blue_line = mlines.Line2D([], [], color='blue',linestyle='None', marker='x', label='Clustered datapoints')
        handles = [centers,closest,rosbot,blue_line]
        plt.legend(handles,["Cluster centers","Closest point","Scanning ROSbot","Clustered datapoints"])
        # plt.subfigure

        # plot roadlines
        plt.plot([-2, 2, 2, -2], [-self.lane_width/2, -self.lane_width/2, self.lane_width/2, self.lane_width/2],'--k')
        plt.plot([-2, 2, 2, -2], [-self.lane_width*1.5, -self.lane_width*1.5, self.lane_width*1.5, self.lane_width*1.5],'--k')

        # fill in red area
        # plt.fill_between(np.array([-2,2]),-self.lane_width*1.5,-1.5,facecolor='red', alpha=0.5,label='Filtered areas')
        # plt.fill_between(np.array([-2,2]),self.lane_width*1.5,1.5,facecolor='red', alpha=0.5)
        # plt.fill_between(np.array([-2,2]),-self.lane_width/2,self.lane_width/2,facecolor='red', alpha=0.5)
        # plt.get_legend_handles_labels()
        # legend elements = [Line2D([0], [0], color='b', lw=4, label='Line'),
        #            Line2D([0], [0], marker='o', color='w', label='Scatter',
        #                   markerfacecolor='g', markersize=15),
        #            Patch(facecolor='orange', edgecolor='r',
        #                  label='Color Patch')]
        # plt.legend(handles=legend_elements, loc='center')

        plt.pause(.01)


    def _cluster_centers(self,labeled_data,labels):
        """
        Calculate the cluster cluster_centers
        labeled_data = clustered points with labels
        labels = array can contain -1, is not counted
        """
        self.previous_centers = self.cluster_centers

        cluster_centers = []
        for i in range(max(set(labels))+1):
            local_cluster = labeled_data[np.where(labeled_data[:,2] == i)]
            mean_x = np.mean(local_cluster[:,0])
            mean_y = np.mean(local_cluster[:,1])
            cluster_centers.append(np.array([mean_x,mean_y]))

        self.cluster_centers = np.array(cluster_centers)


    def _get_nearest_points(self, labeled_data, labels):
        """
        returns the point nearest to [0,0] of each cluster
        """

        self.nearest_points = np.empty_like(self.cluster_centers)
        for clus in range(max(set(labels))+1):
            local_cluster = labeled_data[np.where(labeled_data[:,2] == clus)]
            local_cluster =local_cluster[:, :2]
            self.nearest_points[clus, :] = self._closest_point([0,0],local_cluster)

    def convert_xy(self,ranges):
        """
        Converts ranges with (-pi,pi) to xy coordinates
        ranges = np array (n,)
        coordinates = np array (n,2)
        """
        thetas = np.linspace(-1*np.pi,np.pi,len(ranges))

        #filter values bigger than 10 (innacurate ranges) #TODO remove coordinates in own lane
        threshold = 10
        keep_indices = np.where(ranges<threshold)
        ranges = ranges[keep_indices]
        thetas = thetas[keep_indices]

        #calculate x and y
        x_array = -1*ranges* np.cos(thetas)
        y_array = -1*ranges* np.sin(thetas)



        # #Filter out own lane
        # keep_indices = np.where(np.abs(y_array) > self.lane_width/2)
        # x_array = x_array[keep_indices]
        # y_array = y_array[keep_indices]
        #
        # # And filter out non adjacent lanes
        # keep_indices = np.where(np.abs(y_array) < self.lane_width*1.5)
        # x_array = x_array[keep_indices]
        # y_array = y_array[keep_indices]

        coordinates = np.array([x_array,y_array]).T

        return coordinates

    def _closest_point(self,startpoint,points):
        """
        startpoint: single array [x,y]
        points: array of arrays [[x,y],[x,y]]
        returns: point of poits closest to startpoint
        """
        nodes = np.asarray(points)
        dist_2 = np.sum((nodes - startpoint)**2, axis=1)
        return points[np.argmin(dist_2)]

    def calc_rel_vels(self):
        """
        Calculate relative velocities of cluster centers
        and determine the expected centers
        """

        # initialize velocities and get the time difference
        rel_vels = np.empty_like(self.cluster_centers)
        dt = self.d_time()

        # only execute when same number of clusters as last time
        if len(self.cluster_centers) == len(self.previous_centers):

            # loop over all clusters
            for k in range(len(self.cluster_centers)):

                # calculate the difference in distance between current center and closest previous center
                current = self.cluster_centers[k]
                ds = current - self._closest_point(current, self.previous_centers)

                # convert to velocity
                rel_vels[k] = ds/dt

            # Couple relative velocities to current cluster locations
            rel_vels_linked = np.zeros((len(self.cluster_centers),2,2))
            rel_vels_linked[:,0, :] = self.cluster_centers
            rel_vels_linked[:,1,:] = rel_vels
            self.rel_vels_linked = rel_vels_linked

            pub_rel_vels_linked = np.zeros((len(self.cluster_centers),4))
            pub_rel_vels_linked[:,:2] = self.cluster_centers
            pub_rel_vels_linked[:,2:] = rel_vels
            self.pub_rel_vels_linked = pub_rel_vels_linked


            # predict future position in next time step
            self.projected_centers = self._get_projection(dt)


    def _get_projection(self, dt=0.2):
        # predict future position in next time step
        projected_centers = np.zeros((len(self.cluster_centers),2))
        projected_centers = self.rel_vels_linked[:, 0,:] + dt*self.rel_vels_linked[:, 1,:]
        return projected_centers



    def d_time(self):
        """
        Update time difference
        """
        self.previous_time = self.current_time
        self.current_time = rospy.get_time()
        return self.current_time - self.previous_time



    def publish(self):
        """
        Publish the flags in the namespace
        """
        data = [False, False]
        flag = [1 if self.flag[0] else 0, 1 if self.flag[1] else 0]

        self.flag_history = np.roll(self.flag_history, -1, axis=0)
        self.flag_history[-1] = flag

        if len(np.unique(self.flag_history[:, 0])) == 1:
            data[0] = flag[0]
        else:
            data[0] = False

        if len(np.unique(self.flag_history[:, 1])) == 1:
            data[1] = flag[1]
        else:
            data[1] = False



        flag = UInt8MultiArray()
        flag.data = data

        self.pub_flag.publish(flag)

        objects = Float64MultiArray()
        # print(self.pub_rel_vels_linked.shape)
        objects.data = self.pub_rel_vels_linked.flatten().tolist()
        self.pub_objects.publish(objects)


detector = DetectedVehicle(namespace,plot=True)
detector.subscribers()
