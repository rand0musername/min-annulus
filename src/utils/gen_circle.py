import random
import math

# generates and prints points randomly sampled from a given annulus

r_inner = 100
r_outer = 110
center = (0, 0)

def random_point():
    alpha = 2 * math.pi * random.random()
    r = (r_outer - r_inner) * random.random() + r_inner
    x = r * math.cos(alpha) + center[0]
    y = r * math.sin(alpha) + center[1]
    return x, y

n = 20
print(n)
for i in range(n):
    pt = random_point()
    print('{} {}'.format(pt[0], pt[1]))