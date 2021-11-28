import numpy as np
import matplotlib.pyplot as plt
import os
import datetime

def createDirectory(directory): 
    try: 
        if not os.path.exists(directory): 
            os.makedirs(directory) 
    except OSError: print("Error: Failed to create the directory.")

def plot_joints(desired_joints, qlog, jpos_controller, is_save=False, save_dir="pid_tmp"):
    fig = plt.figure("Current Qpos", figsize=(15,7.5), dpi= 80)
    joints = [[] for _ in range(qlog.shape[1])]
    for j in range(qlog.shape[1]):
        for i in range(qlog.shape[0]):
            joints[j].append(qlog[i][j])

    for i in range(qlog.shape[1]):
        gain = " kp: "+ str(jpos_controller.kp[i])+ " ki: "+ str(jpos_controller.ki[i]) + " kd: "+ str(jpos_controller.kd[i])
        plt.plot(joints[i], linewidth=2, label="joint"+str(i)+ gain)

    plt.plot([desired_joints for _ in range(qlog.shape[0])], 'k--', linewidth=0.5)
    plt.legend(loc='upper right')
    
    if is_save:
        createDirectory(save_dir)
        if os.path.exists(save_dir+'/result.png'):
            plt.savefig(save_dir+'/result_{}.png'.format(datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
        else:
            plt.savefig(save_dir+'/result.png')

    plt.show()