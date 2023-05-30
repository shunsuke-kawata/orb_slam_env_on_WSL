import os
import matplotlib.pyplot as plt
FOLDER_PATH = './trajectory_data'

def visualize_trajectory(file_path):
    with open(file_path, "r") as file:
    # ファイルの内容を読み込む
        lines = file.readlines()
    
    pre_data = None
    data = None
    for line in lines:
        line = line.rstrip("\n")
        data = line.split(" ")

        #各回の差を出力する
        if(pre_data!=None and data!=None):
            print(float(data[1])-float(pre_data[1]),float(data[2])-float(pre_data[2]),float(data[3])-float(pre_data[3]),float(data[4])-float(pre_data[4]),float(data[5])-float(pre_data[5]),float(data[6])-float(pre_data[6]))
        else:
            print("init data")
        
        pre_data = data
        
        
        plot_data = []
        plot_data.append(float(data[3])*1000)

    
# グラフの表示
    time_stamp = list(range(len(plot_data)))
    plt.plot(time_stamp, plot_data, marker="o")
    plt.show()



if __name__ == '__main__':
    path = "./trajectory_data/KeyFrameTrajectory_xchange.txt"
    visualize_trajectory(path)
