import csv
import numpy as np
import time
import os

a = []

def read():
    try:
        f_name = r'D:\work\sim\ACMSIMC_TUT\build\algorithm.dat'
        with open(f_name, mode='r') as f:
            print('found '+f_name)
    except:
        f_name = '../algorithm.dat'
    print('[Python] Read in data...')
    ll = [[], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], [],
        [], [], [], [], [], [], [], [], [], []]
    with open(f_name, mode='r') as f:
        buf = f.readlines()
        reader = csv.reader(buf)
        for idx, row in enumerate(reader):
            if idx == 0:
                continue
            try:
                for ind, el in enumerate(row):
                    ll[ind].append(float(el))
            except:
                break
    return ll

def run(*args):#reference angle load
    path = r"D:\work\sim\ACMSIMC_TUT"
    os.chdir(path)
    os.system("make")
    os.chdir(path+r"\build")
    cmdstr = r"main.exe"
    for i in args:
        cmdstr += " "
        cmdstr += str(i)
    os.system(cmdstr)
    time.sleep(3)

def startserver():
    path = r"D:\work\sim\ACMSIMC_TUT\scripts\node"
    os.chdir(path)
    try:
        a = os.system(r"node.exe" + " " + " server.js")
    except:
        pass

def show():
    import matplotlib.pyplot as plt
    for i in a:
        plt.plot(i)
    plt.show()

def seq(x, y, t = 1):
    for i in np.arange(x, y, t):
        run(i)
        dd = read()
        a.append(dd[25])

if __name__ == "__main__":
    pass


