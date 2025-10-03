import numpy as np

def rotational_main(controller, ri):
    init_motor_pos = ri.get_motor_positions()
    print(init_motor_pos)
    ri.rotate_deg(deg=np.pi/2)
    final_motor_pos = ri.get_motor_positions()
    print(final_motor_pos)

    init_motor_pos = np.array(init_motor_pos)
    final_motor_pos = np.array(final_motor_pos)

    motor_pos_diff = final_motor_pos - init_motor_pos
    print(motor_pos_diff)

    # motor_pos_diff = np.array([-62379, -62388])
    rev_diff = motor_pos_diff / 4096
    print(rev_diff)
    # rot_diff = ri.r * rev_diff / (ri.L / 2)
    rot_diff = 2 * ri.r * rev_diff / (ri.L)

    print(rev_diff, rot_diff)

    rad_rotated = rot_diff * (2*np.pi)

    print(rad_rotated)

def linear_main(controller, ri):
    init_motor_pos = ri.get_motor_positions()
    print(init_motor_pos)
    ri.move_mm()
    # exit()
    final_motor_pos = ri.get_motor_positions()
    print(final_motor_pos)

    init_motor_pos = np.array(init_motor_pos)
    final_motor_pos = np.array(final_motor_pos)

    diff = final_motor_pos - init_motor_pos

    print(diff)

    revs = diff / 4096

    print(revs)

    cir = np.pi * 66.5

    dists = revs * cir 

    print(dists)
