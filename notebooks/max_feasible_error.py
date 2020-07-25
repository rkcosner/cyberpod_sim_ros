import numpy as np
import sys
sys.path.append('../src/src_python/')
from dynamics import dynamics

# code from ECOS filter and ecos.launch

L_Lgh = 0.24338437673585678
L_Lfh = 15.99427512538
L_ah = 100
alpha = 10
alpha_e = 10
C = 0.4

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

    # previously,
    # todo: should only certain components count?
    # Lfh = -x[6]*(x[5] - x5_eq + f[6])
    # Lgh = -x[6]*g[6,[0,1]]  
    # ah = alpha * (-1/2*((x[5]-x5_eq)**2 + x[6]**2) + C)
    
    Lfh_above = -f[6] - alpha_e*x[6] 
    Lgh_above = -g[6] 
    ah_above = alpha*(-x[6] + alpha_e*(C + x5_eq - x[5]))

    Lfh_below = f[6] + alpha_e*x[6] 
    Lgh_below = g[6] 
    ah_below = alpha*(x[6] + alpha_e*(C - x5_eq + x[5]))

    return (Lgh_above, Lfh_above, ah_above), (Lgh_below, Lfh_below, ah_below)

def get_bd(x):
    # Lgh, Lfh, ah = get_fns(x)
    fns_above, fns_below = get_fns(x)
    Lgh_above, Lfh_above, ah_above = fns_above
    Lgh_below, Lfh_below, ah_below = fns_below

    # it seems that we will not be able to satisfy for any u\neq 0, so

    # bd_above = max(np.linalg.norm(Lgh_above) / (2*L_Lgh), (Lfh_above + ah_above)/(2*(L_Lfh+L_ah)))
    bd_above = (Lfh_above + ah_above)/(2*(L_Lfh+L_ah))
    # bd_below = max(np.linalg.norm(Lgh_below) / (2*L_Lgh), (Lfh_below + ah_below)/(2*(L_Lfh+L_ah)))
    bd_below = (Lfh_below + ah_below)/(2*(L_Lfh+L_ah))
    return min(bd_above, bd_below)

def get_gridded_eps(grid_limits=default_grid_limits, selected_inds=[0,5]):
	assert grid_limits.shape[0] == 7

	# computing bound on eps for all of these values
	eps_of_x = -np.inf * np.ones(grid_limits[:,2].astype(int))
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