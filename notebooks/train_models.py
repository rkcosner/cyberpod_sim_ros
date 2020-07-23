import numpy as np
from sklearn.kernel_ridge import KernelRidge
from sklearn.metrics import mean_squared_error
# (make_scorer, max_error, mean_absolute_error, median_absolute_error)
import matplotlib.pyplot as plt
import pickle

def get_test_train_data(train_data_fn, test_data,
                        greyscale=False, downscale=False):
    # loading training data
    data = np.load('../data/' + train_data_fn.split('.')[0] + '_processed.npz')
    states = data['states']
    images = data['images']
    if greyscale:
        images = np.dot(images, [0.299, 0.587, 0.114])
    if downscale:
        # TODO: this is crude, should try method like: https://stackoverflow.com/questions/48121916/numpy-resize-rescale-image
        images = images[:,::downscale,::downscale]
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
    elif test_data is None:
        ys = labels
        Xs = features
    else:
        data2 = np.load('../data/' + test_data.split('.')[0] + '_processed.npz')
        labels2 = data2['states'][:,[0,5]]
        features2 = data2['images'].reshape(data2['images'].shape[0], -1)

        ys = {'test': labels2, 'train': labels}
        Xs = {'test': features2, 'train': features}

    return Xs, ys

def main(train_data_fn, test_data, retrain_metric='mean',
         greyscale=False, downscale=False):
    # loading data
    Xs, ys = get_test_train_data(train_data_fn, test_data,
                                 greyscale=greyscale, downscale=downscale)

    # setting up model

    # TODO: possibly use alternate regressors
    reg = KernelRidge(kernel='rbf')
    n_features = Xs['train'].shape[1]
    param_grid = {'alpha': np.linspace(0,1,5),
                  'gamma': np.logspace(-4,1, 5)/n_features}
    # TODO: additional metrics
    scoring = {'mean': mean_squared_error,}
               # 'median': np.median((ys['train']-y_pred['train'][params])**2, axis=1),
               # 'max': max_error,'percentile': }

    # evaluation 
    res = {metric:{} for metric in scoring.keys()}
    y_pred = {'train':{}, 'test':{}}
    for alpha in param_grid['alpha']:
        for gamma in param_grid['gamma']:
            print('Now training for {:.2g},{:.2g}'.format(alpha, gamma))
            # fit regressor
            reg.set_params(alpha=alpha, gamma=gamma)
            reg.fit(Xs['train'], ys['train'])
            y_pred['train'][(alpha,gamma)] = reg.predict(Xs['train'])

            # evaluate on test data
            y_pred['test'][(alpha,gamma)] = reg.predict(Xs['test'])
            for metric in scoring.keys():
                res[metric][(alpha,gamma)] = scoring[metric](ys['test'], y_pred['test'][(alpha,gamma)])
            print([res[metric][(alpha,gamma)] for metric in scoring.keys()])

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
    filetag = get_filetag(train_data_fn, test_data, greyscale, downscale)
    save_obj(res, 'train_res{}'.format(filetag))
    params, _ = min(res[retrain_metric].items(), key=lambda x: x[1]) 
    reg.set_params(alpha=params[0], gamma=params[1])
    # TODO: refit with test AND train?
    reg.fit(Xs['train'], ys['train'])
    np.savez('../data/coeff{}'.format(filetag), 
             coeff=reg.dual_coef_, alpha=params[0], gamma=params[1],
             Xs_train=Xs['train'],
             y_pred=reg.predict(Xs['train']))

def get_filetag(train_data_fn, test_data, greyscale, downscale):
    image_tag = ''
    if greyscale:
        image_tag += '_grey'
    if downscale:
        image_tag += '_' + str(downscale)
    filetag = '_train{}_test{}{}'.format(train_data_fn, test_data, image_tag)
    return filetag

def save_obj(obj, name ):
    with open('../data/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('../data/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

if __name__ == '__main__':
    # file containing training data
    train_data_fn = 'gridded_data.csv'
    
    # strategy for determining test/train split
    test_data = 'uniform' # 20% of train data uniformly at random
    # test_data = 'test_case_2_no_EKF_var_0_1.csv' # data from additional file
    # test_data = [[-1, 1], [-1, -1]] # 20% of train data closest to the listed points
    
    # whether or not to greyscale images
    greyscale = True

    # factor by which to downscale images
    downscale = 2
    main(train_data_fn, test_data,
         greyscale=greyscale, downscale=downscale)