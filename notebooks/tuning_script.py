import matplotlib.pyplot as plt
import numpy as np

from train_models import get_test_train_data
from sklearn.metrics.pairwise import rbf_kernel
from sklearn.kernel_ridge import KernelRidge

theta_eq = 0.138324423615

train_data_fns = ['gridded2_train.csv', 'gridded2_sampled.csv']  # two different training sets
greyscale = True
downscale = 2
image_tag = ''
if greyscale:
    image_tag += '_grey'
if downscale:
    image_tag += '_' + str(downscale)

test_data = 'uniform'
# test_data = [[-1, -0.5], [-1, 0.5],
#                  [1, -0.5], [1, 0.5],
#                  [0, 0]]


val_data_fn = 'gridded2_evaluation.csv' # TODO: need validation data
Xs, ys = get_test_train_data(val_data_fn, None,
                             greyscale=True, downscale=2)
data = np.load('../data/' + val_data_fn.split('.')[0] + '_processed.npz')
states = data['states']

n_features = Xs.shape[1]

train_data = []
for train_data_fn in train_data_fns:
    
    Xs_train, ys_train = get_test_train_data(train_data_fn, None,
                                 greyscale=True, downscale=2)
    train_data.append((Xs_train, ys_train))

for alpha in np.logspace(-9,-1, 5):
    for gamma in np.logspace(-4,1, 5)/n_features:
        print(alpha, gamma)

        model_list = []
        for (Xs_train, ys_train) in train_data:


            reg = KernelRidge(kernel='rbf')
            reg.set_params(alpha=alpha, gamma=gamma)
            reg.fit(Xs_train, ys_train)
            coeff = reg.dual_coef_
            model_list.append((Xs_train,coeff,gamma))

        

        plt.figure(figsize=[10,4])
        i = 0
        for (Xs_train,coeff,gamma) in model_list:
            assert states.shape[0] == ys.shape[0]


            # prediction
            K = rbf_kernel(Xs, Xs_train, gamma=gamma)
            ys_pred = np.dot(K, coeff)

            # todo: should we look only at theta errs?
            errs = np.linalg.norm(ys-ys_pred, axis=1)
            errs = np.abs(ys[:,1]-ys_pred[:,1])
            
            plt.subplot(1,2,1+i)
            i += 1

            plt.scatter(states[:,5]-theta_eq, states[:,0], c=errs, marker='s')

            plt.ylabel('$x$')
            plt.xlabel('$\\theta - \\theta_*$')
            plt.clim([0, 0.25])
            plt.colorbar()
        plt.title('a={},g={}'.format(alpha,gamma))
        plt.tight_layout()
            


            # plt.figure(figsize=[10,5])
            # i = 0
            # for train_data_fn in train_data_fns:
            #     data = np.load('../data/' + train_data_fn.split('.')[0] + '_processed.npz')
            #     states = data['states']
            #     plt.subplot(1,2,1+i)
            #     i += 1
            #     plt.scatter(states[:,0], states[:,5], marker='x')
            #     plt.xlabel('$x$')
            #     plt.ylabel('$\\theta$')
            # plt.tight_layout()


        # plt.figure(figsize=[5,4])
        # i = 0
        # for (Xs_train,coeff,gamma) in model_list:

        #     # prediction
        #     K = rbf_kernel(Xs, Xs_train, gamma=gamma)
        #     ys_pred = np.dot(K, coeff)

        #     # todo: should we look only at theta errs?
        #     errs = np.linalg.norm(ys-ys_pred, axis=1)
        #     errs = np.abs(ys[:,1]-ys_pred[:,1])
            
        # #     plt.scatter(states[:,5]-0, states[:,0], c=errs, marker='s')
        #     errs_theta = []
        #     for theta in np.unique(states[:,5]):
        #         errs_theta.append(np.mean(errs[np.where(states[:,5]==theta)]))
        #     plt.plot(np.unique(states[:,5]), errs_theta)

        #     plt.ylabel('$x$')
        #     plt.xlabel('$\\theta - \\theta_*$')
        # plt.tight_layout()



        # plt.figure(figsize=[5,4])
        # i = 0
        # for (Xs_train,coeff,gamma) in model_list:
        #     assert states.shape[0] == ys.shape[0]
            
        #     # prediction
        #     K = rbf_kernel(Xs, Xs_train, gamma=gamma)
        #     ys_pred = np.dot(K, coeff)

        #     # todo: should we look only at theta errs?
        #     errs = np.linalg.norm(ys-ys_pred, axis=1)
        #     errs = np.abs(ys[:,1]-ys_pred[:,1])
        #     plt.scatter(ys[:,1],ys_pred[:,1])

        #     plt.xlabel('$\\theta$')
        #     plt.ylabel('$\widehat{\\theta}$')
        # plt.plot([-0.4, 0.7], [-0.4, 0.7])
        # plt.tight_layout()

plt.show()