import glob
import numpy as np
from scipy.signal import resample

def rolling_window(array, window, time_axis=0):
    if time_axis == 0:
        array = array.T

    elif time_axis == -1:
        pass

    else:
        raise ValueError('Time axis must be 0 (first dimension) or -1 (last)')

    assert window >= 1, "`window` must be at least 1."
    assert window < array.shape[-1], "`window` is too long."

    # with strides
    shape = array.shape[:-1] + (array.shape[-1] - window, window)
    strides = array.strides + (array.strides[-1],)
    arr = np.lib.stride_tricks.as_strided(array, shape=shape, strides=strides)

    if time_axis == 0:
        return np.rollaxis(arr.T, 1, 0)
    else:
        return arr

def _data_format(array, history):
    x = rolling_window(array, history)
    y = array[history:,:2]
    return x, y

def load_data(path, history, shuffle=True):
    """ Read and reshape data
    Keyword arguments:
    path -- data directory
    history -- number of previous steps
    shuffle -- shuffle videos (not frames)
    """
    npyfiles = glob.glob(path+'*.npy')
    data = []
    for npyfile in npyfiles:
        raw = np.load(npyfile)
        downsample = resample(raw, len(raw)/15)
        data.append(downsample)

    # NOTE: Must shuffle before concatenate, because rolling window works on a 
    #       time series.
    if shuffle:
        np.random.shuffle(data)
    data = np.concatenate(data,axis=0)
    X, y = _data_format(data,history)
    return X, y
