import numpy as np
from sklearn.kernel_ridge import KernelRidge
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import (make_scorer, max_error, mean_squared_error,
                            mean_absolute_error, median_absolute_error)
import matplotlib.pyplot as plt
import pickle

def get_test_train_data(train_data_fn, test_data):
    # loading training data
    data = np.load('../data/' + train_data_fn.split('.')[0] + '_processed.npz')
    states = data['states']
    images = data['images']
    features = images.reshape(images.shape[0], -1)
    labels = states[:,[0,5]]

    # loading testing data
    if test_data == 'uniform': # random uniform 
        num_test = int(len(states)*0.2)
        selected_inds = np.random.choice(len(states), size=num_test, replace=False)
        test_inds = np.zeros(len(states), dtype=bool)
        test_inds[selected_inds] = True
        train_inds = np.logical_not(test_inds)

        ys = {'test': labels[test_inds], 'train': labels[train_inds]}
        Xs = {'test': features[test_inds], 'train': features[train_inds]}

    elif isinstance(test_data,(list,np.ndarray)): # local sampling
        num_test = int(len(states)*0.2)
        # computing distance with labels
        dists = np.array([np.linalg.norm(labels-np.array(point), axis=1) for point in test_data])
        # finding closest datapoints
        print(dists.shape)
        selected_inds = np.argsort(np.min(dists, axis=0))[:num_test]
        test_inds = np.zeros(len(states), dtype=bool)
        test_inds[selected_inds] = True
        train_inds = np.logical_not(test_inds)

        ys = {'test': labels[test_inds], 'train': labels[train_inds]}
        Xs = {'test': features[test_inds], 'train': features[train_inds]}

    else:
        data2 = np.load('../data/' + test_data.split('.')[0] + '_processed.npz')
        labels2 = data2['states'][:,[0,5]]
        features2 = data2['images'].reshape(data2['images'].shape[0], -1)

        ys = {'test': labels2, 'train': labels}
        Xs = {'test': features2, 'train': features}

    return Xs, ys

def main(train_data_fn, test_data, retrain_metric='mean'):
    # loading data
    Xs, ys = get_test_train_data(train_data_fn, test_data)

    # setting up model

    # TODO: possibly use alternate regressors
    reg = KernelRidge(kernel='rbf')
    n_features = Xs['train'].shape[1]
    param_grid = {'alpha': np.linspace(0,1,1),
                  'gamma': np.linspace(1/n_features,1/n_features, 1)}
    # TODO: additional metrics
    scoring = {'mean': mean_squared_error,}
               # 'median': np.median((ys['train']-y_pred['train'][params])**2, axis=1),
               # 'max': max_error,'percentile': }

    # evaluation 
    res = {metric:{} for metric in scoring.keys()}
    y_pred = {'train':{}, 'test':{}}
    for alpha in param_grid['alpha']:
        for gamma in param_grid['gamma']:
            # fit regressor
            reg.set_params(alpha=alpha, gamma=gamma)
            reg.fit(Xs['train'], ys['train'])
            y_pred['train'][(alpha,gamma)] = reg.predict(Xs['train'])

            # evaluate on test data
            y_pred['test'][(alpha,gamma)] = reg.predict(Xs['test'])
            for metric in scoring.keys():
                res[metric][(alpha,gamma)] = scoring[metric](ys['test'], y_pred['test'][(alpha,gamma)])
    
    for metric in scoring.keys():
        params, err = min(res[metric].items(), key=lambda x: x[1]) 
        plt.figure(figsize=[4,4])
        plt.title('Best for {}={:.2g} '.format(metric, err)
                  + 'a={:.2g}, g={:.2g}'.format(*params))
        # TODO: eventually want to plot with true y in case there is noise
        errs = np.sum((ys['train']-y_pred['train'][params])**2, axis=1)
        plt.scatter(ys['train'][:,0], ys['train'][:,1], c=errs)
        errs = np.sum((ys['test']-y_pred['test'][params])**2, axis=1)
        plt.scatter(ys['test'][:,0], ys['test'][:,1], c=errs, marker='s')

    plt.show()

    # save results
    save_obj(res, 'train_res')
    params, _ = min(res[retrain_metric].items(), key=lambda x: x[1]) 
    reg.set_params(alpha=params[0], gamma=params[1])
    # TODO: refit with test AND train?
    reg.fit(Xs['train'], ys['train'])
    np.savez('../data/coeff_train{}_test{}'.format(train_data_fn, test_data), 
             coeff=reg.dual_coef_, alpha=params[0], gamma=params[1],
             Xs_train=Xs['train'],
             y_pred=reg.predict(Xs['train']))

def save_obj(obj, name ):
    with open('../data/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('../data/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

if __name__ == '__main__':
    train_data_fn = 'gridded_data.csv'
    test_data = 'uniform'
    # test_data = 'test_case_2_no_EKF_var_1.csv'
    # test_data = 'test_case_2_no_EKF_var_0_1.csv'
    # test_data = [[-1, 1], [-1, -1]]
    main(train_data_fn, test_data)