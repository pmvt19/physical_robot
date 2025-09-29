import numpy as np

start_pos = np.array([20291, 58575])
end_pos = np.array([18395, 60478])

diff = end_pos - start_pos

print(diff)

revs = diff / 4096

print(revs)

cir = np.pi * 66.5

dists = revs * cir 

print(dists)