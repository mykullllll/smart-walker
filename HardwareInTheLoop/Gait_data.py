from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN, KMeans
from matplotlib.animation import FuncAnimation 
import time
import rospy
import scipy
from scipy.signal import savgol_filter, find_peaks
from scipy.interpolate import interp1d
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, Float64
import csv

accurate = 0
clump = 0
occlusion = 0
noise = 0

def cluster_find(collisions,prev_leg_r=None,prev_leg_l=None):
    global accurate, clump, occlusion, noise, width_history
    isoccluded=False
    if len(collisions)==0:
        return prev_leg_l,prev_leg_r,False
    
    cluster=DBSCAN(eps=4e-2,min_samples=3).fit(collisions)
    labels=cluster.labels_
    unique_labels = [l for l in np.unique(labels) if l != -1]
    print(f"Number of clusters found: {len(unique_labels)}")
    centroids=[]

    if len(unique_labels)==0 or len(unique_labels)>2:
        noise+=1
        return prev_leg_l, prev_leg_r, isoccluded
    #Ideal Case (2 Clusters)
    if len(unique_labels)==2:   
        for index in unique_labels:
            leg_points= collisions[labels==index]
            centroid= (np.mean(leg_points,axis=0))
            centroids.append(centroid)
            global accurate
            accurate+=1


    elif len(unique_labels)==1:
        leg_points = collisions[labels==unique_labels[0]]

        width = np.max(leg_points[:,1])-np.min(leg_points[:,1])

        if width>0.15:
            kmeans= KMeans(n_clusters=2,n_init=10).fit(leg_points)
            centroids = kmeans.cluster_centers_
            print("Clumped Cluster Detected")
            global clump
            clump+=1

        
        else:
            single_centroid=np.mean(leg_points,axis=0)
            print("Occlusion Detected")
            global occlusion
            width_history.append(width)
            occlusion+=1
            isoccluded=True
            
            
            #Check which leg current cluster corresponds to
            if prev_leg_l is not None and prev_leg_r is not None:

                dist_center_r= np.linalg.norm(single_centroid-prev_leg_r)
                dist_center_l=np.linalg.norm(single_centroid-prev_leg_l)

                if dist_center_r < dist_center_l:
                    print(f"Right Leg {dist_center_r} Left Leg {dist_center_l}")
                    return prev_leg_l, single_centroid, isoccluded
                
                else:
                    print(f"Right Leg {single_centroid} Left Leg {prev_leg_l}")
                    return  single_centroid,prev_leg_r, isoccluded
                
            #If no history drop frame
            else:
                return [1,0], [1,0], isoccluded

    if centroids[0][1]<centroids[1][1]:
        left_leg=(centroids[1])
        right_leg=(centroids[0])

        #plt.scatter(left_leg[0],left_leg[1])
        #plt.scatter(right_leg[0],right_leg[1])
    if centroids[0][1]>centroids[1][1]:
        left_leg=(centroids[0])
        right_leg=(centroids[1])


    print(f"Right Leg {right_leg} Left Leg {left_leg}")
    return left_leg,right_leg,isoccluded

#Collision Scanner
def process_scan(angle_min,angle_max,angle_increment,range_min,range_max,ranges,angle_offset=0): 
        collisions = []
        for i in range(0,len(ranges)):
            #print(position)
            #print(walker_angle)
            if ranges[i] == float('Inf'):
                continue
            if ranges[i] > range_min and ranges[i] < range_max:
                angle = angle_min + i * angle_increment + angle_offset
                dx = ranges[i] * np.cos(angle)
                dy = ranges[i] * np.sin(angle)
                collisions.append((dx, dy))
        collisions=np.array(collisions)
        return collisions


current_scan = None
def scan_callback(msg:LaserScan):
    global current_scan
    current_scan=msg

encoder = None
def odom_callback(msg:Odometry):
    global encoder
    encoder=msg

if __name__== "__main__":
    rospy.init_node('Gait_data')

    sampling_frequency=10
    min_dist=0.25
    max_dist=2
    prev_right=[0,0]
    prev_left=[0,0]
    width_history=[]
    time_stamp=[]
    right_history=[]
    left_history=[]
    encoder_value=[]

    rate=rospy.Rate(sampling_frequency)
    laser_scan_sub = rospy.Subscriber("/scan_legs_filtered",LaserScan,scan_callback,queue_size=1)
    pub_shutdown = rospy.Publisher('/shutdown',Bool,queue_size=1)
    encoder_sub=rospy.Subscriber('/odom',Odometry,odom_callback,queue_size=1)
    rospy.sleep(1.0)

    start_time = rospy.Time.now().to_sec()

    try:
        while not rospy.is_shutdown():
            if current_scan == None or encoder is None:
                rate.sleep()
                continue
            collisions = process_scan(current_scan.angle_min,current_scan.angle_max,current_scan.angle_increment,min_dist,max_dist,current_scan.ranges)
            raw_left,raw_right,isoccluded= cluster_find(collisions,prev_right,prev_left)



    except rospy.ROSInterruptException:
        pass
    
    finally:
        rows=[]
        for left, right, t, enc in zip(left_history, right_history, time_stamp, encoder_value):
            # Extract individual X and Y components
            lx, ly = left[0], left[1]
            rx, ry = right[0], right[1]
            rows.append([lx, ly, rx, ry, t, enc])

        with open('straight_line.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            # Optional: Write a header row
            writer.writerow(['Left_x', 'Left_y', 'Right_x','Right_y','time(s)','Encoder'])
            # Write all data rows at once
            writer.writerows(rows)  

            print("Save Complete")







