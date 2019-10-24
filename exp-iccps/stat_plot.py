import csv
import sys
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


csv_name = sys.argv[1]

neg = 0

df = pd.read_csv(csv_name)
for it, t, m in zip(df['iter'], df['time'], df['map']):
    print(it, t/1000000000)
    x = 0
    y = 0
    mapGrid = np.zeros(shape=[22,22])
    for s in m:
        try:
            if s == '-':
                neg = 1
            i = int(s)
            if neg:
                neg = 0
                i = -i
            mapGrid[y][x] = i
            y += 1
            if y == 22:
                y = 0
                x += 1
        except ValueError:
            pass

    viz_map = np.full(shape=[22,22], fill_value=-1, dtype=np.int8)
    count = 0
    for x in range(22):
        for y in range(22):
            _x, _y = x - 11, y - 11
            viz_map[y][x] = mapGrid[_x][_y]  # XXX the image has to be transposed
            if viz_map[y][x] == -1:
                count += 1

    plt.imshow(viz_map, origin='lower')
    plt.pause(0.05)
    print(1-count/484)