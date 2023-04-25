#This is v2 . the size of graph is fixed regardless of the window.

#input: filename
#output: cluster of drop points plotted in a figure.
#automatically fix the size of graph.

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time
import cv2      #use this to fasten graph
import datetime

#Rotation matrixt
def rotate(theta,vector):       #vector = (xt, y)    theta = degree
    rad = theta * np.pi / 180
    vector_xt_votated = vector[0] * np.cos(rad) - vector[1] * np.sin(rad)
    vector_y_rotated = vector[0] * np.sin(rad) + vector[1] * np.cos(rad)
    return (vector_xt_votated, vector_y_rotated)


#show graph with open-cv, which allow to write graph in fastened size all the time.
def fasten_size_by_cv2(window_name,image_name):     #input window_name you want to and name of file you want to show
    image = cv2.imread(image_name)   #read image
    return cv2.imshow(window_name, image)


def nowtime():
    return str(datetime.datetime.now())


def csv2graph_dp(filename):                                     #dp means drop points
#reader
    df = pd.read_csv(filename)
    num_angle_x = int(df.at[0, df.columns[1]])                #get the number of angle
    num_wind = int(df.columns[1])                             #get the number of wind
    print(num_angle_x, num_wind)
#get xt array
#column is wind. row is angle
    x = []
    y = []
#xt for each wind velocity. x for vector of xt
    for j in range (num_wind):
        xt = []
        for i in range(2, 2+ num_angle_x):
            xt.append(df.at[i, df.columns[2*j]])
        x.append(xt)
        
#y
    for j in range (num_wind):
        yt = []
        for i in range(2, 2+ num_angle_x):
            yt.append(df.at[i, df.columns[2*j+1]])
        y.append(yt)
#rotate
    for i in range(len(x)):
        for j in range(len(x[0])):
            x[i][j], y[i][j]  = rotate(-1*df.at[1, df.columns[1]], (np.float64(x[i][j]), np.float64(y[i][j])))
#time       later
    ut = time.time()
    
#center 
    center = (np.float64(df.at[3,df.columns[0] ]), np.float64(df.at[3,df.columns[1]]))
    

#min and max
    x_max, y_max = center  #center later
    x_min = y_min = 0
    
    for i in range(len(x)):
        for j in range(len(x[0])):
            if x_max < np.float64(x[i][j]):
                x_max = np.float64((x[i][j]))
            if x_min > np.float64(x[i][j]):
                x_min = np.float64(x[i][j])
            if y_max < np.float64(y[i][j]):
                y_max = np.float64(y[i][j])
            if y_min > np.float64(y[i][j]):
                y_min = np.float64(y[i][j])
    
    a = max(x_max, y_max)
    b = min(x_min, y_min)
    
#make graph
    fig = plt.figure(figsize= (10, 10))           #adjust size
    ax = fig.add_subplot(111)
    ax.grid(which="both")
    ax.set_xlabel("x (m)\ncluster of drop points depend on wind velocity")
    ax.set_ylabel("y (m)")
    
    ax.set_xlim(b, a + 1000)                    #automatically size the length of x y axes.
    ax.set_ylim(b, a + 1000)
    
    for i in range(len(x)):
        x[i].append(x[i][0])
        y[i].append(y[i][0])
        ax.plot(x[i],y[i], marker = "o", linestyle = "solid", label = "wind velocity " + str(i)+" m/s", markersize = 1.5)
        ax.minorticks_on()
        ax.legend()
        
    plt.savefig("drop_history.png")
    
    fasten_size_by_cv2(nowtime() + " drop history","drop_history.png")   #"add date later"
    return 0



csv2graph_dp("profile.csv")

