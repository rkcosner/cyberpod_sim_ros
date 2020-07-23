import numpy as np
import sys
sys.path.append('../src/src_python/')
from dynamics import dynamics

# code from ECOS filter and ecos.launch

L_Lgh = 1# 0.5
L_Lfh = 1 #0.1
L_ah = 5
alpha = 10
C = 1 # TODO: not sure which param this should be

# specifying upper_bd, lower_bd, num_grid
default_grid_limits = np.array([[-1, 1, 20], # x
	                        [-6, 6, 20], # xd
	                        [0, 0, 1],
	                        [0, 0, 1],
	                        [0, 0, 1],
	                        [-1, 1, 20], # theta
	                        [-6,6, 20]]) # thetad

def get_fns(x):
    
    f,g = dynamics(x)
    x5_eq = 0.138
    # todo: should only certain components count?
    Lfh = -x[6]*(x[5] - x5_eq + f[6])
    Lgh = -x[6]*g[6,[0,1]]  
    ah = alpha * (-1/2*((x[5]-x5_eq)**2 + x[6]**2) + C)
    
    return Lgh, Lfh, ah

def get_bd(x):
    Lgh, Lfh, ah = get_fns(x)
    return max(np.linalg.norm(Lgh) / (2*L_Lgh), (Lfh + ah)/(2*(L_Lfh+L_ah)))

def get_gridded_eps(grid_limits=default_grid_limits, selected_inds=[0,5]):
	assert grid_limits.shape[0] == 7

	# computing bound on eps for all of these values
	eps_of_x = -np.inf * np.ones(grid_limits[:,2])
	for i0, x0 in enumerate(np.linspace(*grid_limits[0])):
	    for i1, x1 in enumerate(np.linspace(*grid_limits[1])):
	        for i2, x2 in enumerate(np.linspace(*grid_limits[2])):
	            for i3, x3 in enumerate(np.linspace(*grid_limits[3])):
	                for i4, x4 in enumerate(np.linspace(*grid_limits[4])):
	                    for i5, x5 in enumerate(np.linspace(*grid_limits[5])):
	                        for i6, x6 in enumerate(np.linspace(*grid_limits[6])):
	                            eps_of_x[i0, i1, i2, i3, i4, i5, i6] = get_bd(np.array([x0, x1, x2, x3, x4, x5, x6]))
	# want to plot only for x, theta
	axis = tuple(i for i in range(7) if i not in selected_inds)
	return np.min(eps_of_x, axis=axis)

if __name__ == '__main__':
	import matplotlib.pyplot as plt
	colors = get_gridded_eps()

	plt.figure()
	plt.imshow(colors)
	# plt.scatter(states[:,0].flatten(), states[:,1].flatten(), c=colors.flatten())
	plt.colorbar()
	plt.show()