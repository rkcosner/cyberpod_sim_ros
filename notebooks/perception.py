import numpy as np
import scipy.linalg

class Predictor():
    """Base class for predictors.

    Parameters
    ----------
    zs_train : list
        List of training observations.
    ys_train : list
        List of training measurements.

    """

    def __init__(self, zs=[], ys=[], online=False):
        """Create a predictor."""
        self.zs_train = zs
        self.ys_train = ys

    def pred(self, zs):
        """Prediction function.

        Parameters
        ----------
        zs : list
            New observations.

        Returns
        -------
        preds : list
            Predicted labels.

        """
        preds, _ = self.compute_pred(zs)
        return preds

class KernelRidgePredictor(Predictor):
    """Kernel ridge regression estimator.

    Parameters
    ----------
    lam : float
        Regularization parameter.
    kernel: str
        Type of kernel function.

    """

    def __init__(self, lam=1, ys=[], zs=[],
                 kernel='rbf'):
        super().__init__(zs=zs, ys=ys)
        self.lam = lam
        self.gamma = 1e-9
        kernel_dict = {'rbf': self._default_kernel}
        self.kernel = kernel_dict[kernel]
        self.trained = False
        self.K = None

    def _default_kernel(self, zs, ys=None):
        znew = None if ys is None else [y.flatten() for y in ys]
        kernel = sklearn.metrics.pairwise.rbf_kernel([z.flatten() for z in zs], znew,
                                                     gamma=self.gamma).T
        return kernel

    def train(self):
        ys = np.array(self.ys_train)
        zs = np.array(self.zs_train)
        if self.K is None:
            self.K = self.kernel(zs)
        sv_sq, U = scipy.linalg.eigh(self.K)
        sv_sq[(sv_sq < 0)] = 0
        self.coeff = ys.T @ U @ np.diag(1 / (sv_sq + self.lam)) @ U.T
        self.trained = True

    def compute_pred(self, zs):
        if not self.trained:
            self.train()
        preds = self.kernel(self.zs_train, zs) @ self.coeff.T
        return preds, None

class FeatureRidgePredictor(Predictor):
    """ Ridge regression estimator.

    Parameters
    ----------
    lam : float
        Regularization parameter.
    features: str
        Type of feature function.

    """

    def __init__(self, lam=1, ys=[], zs=[],
                 features='identity'):
        super().__init__(zs=zs, ys=ys)
        self.lam = lam
        feature_dict = {'identity': self._default_features,
                        'fast': self._fast_features,
                        'canny': self._canny_features}
        self.features = feature_dict[features]
        self.phis = None
        self.trained = False

    def _default_features(self, zs):
        return np.array([z[:,:,0].flatten() for z in zs])

    def _fast_features(self, zs):
        from skimage.feature import corner_peaks, corner_fast

        vecs = []
        for z in zs:
            fast_img = corner_peaks(corner_fast(z[:,:,0], n=4, threshold=1.5e-17),
                         indices=False)
            vecs.append(fast_img.flatten())
        return np.array(vecs)

    def _canny_features(self, zs):
        from skimage.feature import canny

        vecs = []
        for z in zs:
            canny_img = canny(z[:,:,0], low_threshold=50, high_threshold=50)
            vecs.append(canny_img.flatten())
        return np.array(vecs)

    def train(self):
        ys = np.array(self.ys_train)
        if self.phis is None:
            zs = np.array(self.zs_train)
            self.phis = self.features(zs)
        U, s, VT = scipy.linalg.svd(self.phis, full_matrices=False)
        sv_sq = s**2
        D_sigma = s / (sv_sq + self.lam)
        self.ahat = ys.T @ U @ np.diag(D_sigma) @ VT 
        self.trained = True

    def compute_pred(self, zs):
        zs_features = self.features(zs)
        if not self.trained:
            self.train()
        preds = zs_features @ self.ahat.T
        return preds, None
