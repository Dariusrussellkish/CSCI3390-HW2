import csv
import sys

import numpy as np
from matplotlib import pyplot as plt
from scipy import stats


def readcsv(fname):
    numv, nume, maxf, time = [], [], [], []
    with open(fname, 'r') as fh:
        reader = csv.reader(fh, delimiter=',')
        for row in reader:
            numv.append(int(row[0]))
            nume.append(int(row[1]))
            maxf.append(int(row[2]))
            time.append(float(row[3]))
    return numv, nume, maxf, time


if __name__ == "__main__":
    numv, nume, maxf, time = readcsv(sys.argv[1])

    name = sys.argv[2]
    numv = np.array(numv)
    nume = np.array(nume)
    maxf = np.array(maxf)
    time = np.array(time)

    # plt.title(f"{name} Node Distribution")
    # plt.hist(numv)
    # plt.show()
    # plt.title(f"{name} Edge Distribution")
    # plt.hist(nume)
    # plt.show()
    # plt.title(f"{name} MaxFlow Distribution")
    # plt.hist(maxf)
    # plt.show()
    plt.show()
    plt.title(f"{name} O(nm^2)")
    x = numv * nume * nume
    gradient, intercept, r_value, p_value, std_err = stats.linregress(x, time)
    mn = np.min(x)
    mx = np.max(x)
    x1 = np.linspace(mn, mx, 500)
    y1 = gradient * x1 + intercept
    plt.plot(x, time, 'ob')
    plt.plot(x1, y1, '-r')
    plt.text(0, max(time), 'R-squared = %0.2f' % r_value)
    plt.show()
