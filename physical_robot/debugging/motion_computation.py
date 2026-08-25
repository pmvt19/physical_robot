import numpy as np


def compute_linear_motion(init_mp, final_mp):
    print(f"Compute Linear Motion")
    init_mp = np.array(init_mp)
    final_mp = np.array(final_mp)
    print(f"Initial Motor Positions: {init_mp}")
    print(f"Final Motor Positions: {init_mp}")

    # diff = final_mp - init_mp
    half_max = (2**32) / 2
    diff = (((final_mp - init_mp) + (half_max)) % (2**32)) - half_max
    print(f"Motor Differentials: {diff}")

    revs = diff / 4096

    cir = np.pi * 66.5 # TODO: Avoid Magic Number

    dists = revs * cir 
    return dists

init_mp = [4294967250, 4294924486]
final_mp = [3433, 4294921014]
dists = compute_linear_motion(init_mp, final_mp)
print(f"Dists: {dists}")