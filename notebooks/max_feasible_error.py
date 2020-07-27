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
x5_eq = 0.138 # 0.138324423615

# specifying upper_bd, lower_bd, num_grid
default_grid_limits = np.array([[0, 0, 1], # x 
                        [0, 0, 1], # y
                        [0, 0, 1], # theta_z
                        [-1, 1, 20], # xd
                        [0, 0, 1], # thetad_z
                        [-0.5+x5_eq, 0.5+x5_eq, 30], # theta_y
                        [None, None, 30]]) # thetad_y depends on theta_y


def get_fns(x):
    
    f,g = dynamics(x)

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

def get_gridded_eps():
    # computing bound on eps for all of these values
    eps_vals = []
    state_vals = []
    x0 = 0
    x1 = 0
    x2 = 0
    x4 = 0
    grid_limits = default_grid_limits
    for x5 in (np.linspace(*grid_limits[5])):
        # thetad_y is gridden to depend on theta_y
        # for x6 in (np.linspace(-alpha_e*C,alpha_e*C,grid_limits[6,2])-alpha_e*(x5-x5_eq)):
        # for x6 in (np.linspace(- alpha_e*(C - x5_eq + x5),alpha_e*(C + x5_eq - x5),grid_limits[6,2])):
        upper = min(1.5, alpha_e*(C + x5_eq - x5))
        lower = max(-1.5,  - alpha_e*(C - x5_eq + x5))
        for x6 in (np.linspace(lower,upper,grid_limits[6,2])):
            max_over_x3 = []
            for x3 in (np.linspace(*grid_limits[3])):
                max_over_x3.append(get_bd(np.array([x0, x1, x2, x3, x4, x5, x6])))
            eps_vals.append(np.max(max_over_x3))
            state_vals.append([x5, x6])
    return np.array(eps_vals), np.array(state_vals)
    

def get_gridded_eps_general(grid_limits=default_grid_limits, selected_inds=[6,5]):
    assert grid_limits.shape[0] == 7

    # computing bound on eps for all of these values
    eps_of_x = -np.inf * np.ones(grid_limits[:,2].astype(int))
    for i0, x0 in enumerate(np.linspace(*grid_limits[0])):
        for i1, x1 in enumerate(np.linspace(*grid_limits[1])):
            for i2, x2 in enumerate(np.linspace(*grid_limits[2])):
                for i3, x3 in enumerate(np.linspace(*grid_limits[3])):
                    for i4, x4 in enumerate(np.linspace(*grid_limits[4])):
                        for i5, x5 in enumerate(np.linspace(*grid_limits[5])):
                            # thetad_y is gridden to depend on theta_y
                            for i6, x6 in enumerate(np.linspace(-alpha_e*C,alpha_e*C,grid_limits[6,2])-alpha_e*(x5-x5_eq)):
                                eps_of_x[i0, i1, i2, i3, i4, i5, i6] = get_bd(np.array([x0, x1, x2, x3, x4, x5, x6]))
    # want to plot only for some values
    axis = tuple(i for i in range(7) if i not in selected_inds)


    Xs = []; Ys = []
    for x5 in np.linspace(*grid_limits[5]):
        # thetad_y is gridden to depend on theta_y
        for x6 in (np.linspace(-alpha_e*C,alpha_e*C,grid_limits[6,2])-alpha_e*(x5-x5_eq)):
            Xs.append(x5); Ys.append(x6)

    return np.min(eps_of_x, axis=axis), (Xs, Ys)

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    colors = get_gridded_eps()

    plt.figure()
    xRange = np.linspace(default_grid_limits[5][0], default_grid_limits[5][1], default_grid_limits[5][2])
    yRange = np.linspace(default_grid_limits[6][0], default_grid_limits[6][1], default_grid_limits[0][2])
    ax = plt.imshow(colors)
    plt.xticks(range(20), xRange, rotation=90)
    plt.yticks(range(20), yRange)
    plt.xlabel(r'$\theta$')
    plt.ylabel(r'$\dot{\theta}$')
    plt.colorbar()
    plt.show()