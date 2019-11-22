import csv
import sys
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def cal_percent(m) -> float:
    num_known = 0
    for row in m:
        for e in row:
            if e == -1:
                continue
            num_known += 1
    return num_known / 4  # / 400 * 100


def plot_map(m, t):
    ax = plt.gca()
    viz_map = np.full(shape=[22,22], fill_value=-1, dtype=np.int8)
    for x in range(22):
        for y in range(22):
            _x, _y = x - 11, y - 11
            viz_map[y][x] = m[_x][_y]  # XXX the image has to be transposed
            if viz_map[y][x] == 1:
                ax.add_patch(mpl.patches.Rectangle((x-.5, y-.5),1,1,hatch='/////',fill=False,snap=False))
    cmap = mpl.colors.ListedColormap(["lightgray", "white", "white"])
    ax.imshow(viz_map, cmap=cmap, origin='lower')
    plt.title("At " + str(t/(10**9)) + "s")
    plt.axis("off")
    plt.draw()
    plt.pause(5)


csv_name = sys.argv[1]

with open(csv_name) as csv_file:
    reader = csv.DictReader(csv_file)

    try:
        row = next(reader)
        t_0 = int(row['time'])

        for row in reader:
            i = int(row['iter'])
            t = int(row['time']) - t_0
            m = eval(row['map'])

            if i == 13:
                plot_map(m, t)

            t_5m = 300 * (10 **9)
            if t > t_5m:
                plot_map(m, t)
                break

        for row in reader:
            i = int(row['iter'])
            t = int(row['time']) - t_0
            m = eval(row['map'])
        plot_map(m, t)

    except KeyboardInterrupt:
        pass

"""
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
"""
