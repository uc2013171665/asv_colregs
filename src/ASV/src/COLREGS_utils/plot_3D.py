#!/usr/bin/env python

#For Graphs
#import matplotlib
#matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D, get_test_data
from matplotlib import cm

def plot_f_function(f_function): #Function to generate a 3D color plot
    v_values = []
    heading_values = []
    f_values = []
    for j in range(360):
        for i in range(11):
            v_values.append(i) #Create a 360x11=3960 lines array with velocity ranging from [0,10]
            heading_values.append(j) #Create a 360x11=3960 lines array with heading ranging from [1,360]
            f_values.append(f_function[j,i]) #Create a 360x11=3960 lines array with f value obtained from the f_function function ranging from [0,100]

    X = v_values
    Y = heading_values
    Xv, Yv = np.meshgrid(X, Y) #Generate a grid
    Z = f_values

    # set up a figure twice as wide as it is tall
    fig = plt.figure(figsize=plt.figaspect(0.5))
    # set up the axes for the first plot
    ax = fig.add_subplot(111,projection='3d')

    dem3d=ax.plot_surface(Xv,Yv,f_values,cmap=cm.coolwarm, linewidth=0) #Define axes and parameters
    fig.colorbar(dem3d, shrink=0.5, aspect=10)

    #Define labels, titles and scale
    ax.set_title('F_function')
    ax.set_zlabel('cost')
    ax.set_xlabel('Velocity ($m/s$)')
    ax.set_ylabel('Heading ($degrees$)')
    ax.set_xlim(xmin=0,xmax=10)
    ax.set_ylim(ymin=0,ymax=360)
    ax.set_zlim(zmin=0,zmax=100)

    plt.show()
