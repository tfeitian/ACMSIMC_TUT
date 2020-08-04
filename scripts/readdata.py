import csv

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

def run(param):
    import os
    path = r"D:\work\sim\ACMSIMC_TUT\build"
    os.chdir(path)
    exefile = r"main.exe"
    os.system(exefile + ' ' + str(param))

def show():
    import matplotlib.pyplot as plt
    for i in a:
        plt.plot(i)
    plt.show()

def seq():
    for i in range(-10, 10):
        run(i)
        dd = read()
        a.append(dd[25])

if __name__ == "__main__":
    pass


