import numpy as np
from scipy.spatial.transform import Rotation as R


if __name__=="__main__":
    # quat = [0.00542158673884506,0.9700904425576314,-0.06438140916851579,0.23398755054286074]
    # tran = [0.021550492376032543, 0.6225167861440358,0.7153224520221988]
    # rot_matrix_b_c = R.from_quat(quat).as_matrix()
    # t_b_c = np.reshape(np.array(tran),(3,1))

    # rot_matrix_c_b = rot_matrix_b_c.T
    # t_c_b = -rot_matrix_c_b @ t_b_c

    # quat_c_t = R.from_matrix(rot_matrix_c_b).as_quat()
    # print(t_c_b)
    # print(quat_c_t)
    q = R.from_euler('y',np.pi).as_quat()
    print(q)



