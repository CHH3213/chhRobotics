# main.py

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Rectangle

import random_map
import a_star
from celluloid import Camera  # 保存动图时用，pip install celluloid

fig=plt.figure(figsize=(5, 5))

map = random_map.RandomMap()

ax = plt.gca()
ax.set_xlim([0, map.size])
ax.set_ylim([0, map.size])

for i in range(map.size):
    for j in range(map.size):
        if map.IsObstacle(i,j):
            rec = Rectangle((i, j), width=1, height=1, color='gray')
            ax.add_patch(rec)
        else:
            rec = Rectangle((i, j), width=1, height=1, edgecolor='gray', facecolor='w')
            ax.add_patch(rec)

rec = Rectangle((0, 0), width = 1, height = 1, facecolor='b')
ax.add_patch(rec)

rec = Rectangle((map.size-1, map.size-1), width = 1, height = 1, facecolor='r')
ax.add_patch(rec)

plt.axis('equal')
plt.axis('off')
plt.tight_layout()
#plt.show()
# 保存动图用
camera = Camera(fig)
a_star = a_star.AStar(map)
a_star.RunAndSaveImage(ax, plt,camera)

