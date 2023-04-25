#select 2 rows you want. After graph show up, you can close it and enter "yes" or "no" if you want to see another graph

import matplotlib.pyplot as plt
import pandas as pd


def csv2graph(filename):
    csv_df = pd.read_csv(filename)
    csv_df.columns
    
    row1 = int(input("Enter row number(integer) you want to use for x axe.\nX ="))
    row2 = int(input("Enter row number(integer) you want to use for y axe.\nY ="))
    
    #Write graph
    data_x = csv_df[csv_df.columns[row1]]          #data_x is time. Use this in x axe later
    data_y = csv_df[csv_df.columns[row2]]
    
    fig = plt.figure(figsize = (9, 6))              #object graph
    ax = fig.add_subplot(111)                       #make ax in fig
    ax.set_xlabel(csv_df.columns[row1])             #just label
    ax.set_ylabel(csv_df.columns[row2])
    ax.grid()                                       #add grid
    ax.plot(data_x, data_y, linestyle = "solid", marker = "o", linewidth = 0.1, markersize = 0.5)
    return plt.show()


def keep_csv2graph(filename):
    csv2graph(filename)
    x = input("Do you want to see graph again?\nEnter 'yes' or 'no '")
    if x == "yes":
        return keep_csv2graph(filename)
    else:
        return print("End the program")


keep_csv2graph("history_0deg_wind0.csv")
