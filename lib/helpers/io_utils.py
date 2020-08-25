import cPickle as pk
from os import makedirs, remove
from os.path import join, exists, abspath, dirname, basename, isfile


def check_and_save_data(results, save_path, save_name):
    '''
    Create a dictionary for saving the input results under the key
    save_name, i.e., data[save_name] = results, and save the dictionary
    to save_path. The function will load the data dict first if save_path
    already exists.
    '''

    if exists(save_path):
        with open(save_path, 'r') as f:
            data = pk.load(f)
            data[save_name] = results
    else:
        data = {save_name: results}
        if not exists(dirname(save_path)):
            makedirs(dirname(save_path))

    with open(save_path, 'w') as f:
        pk.dump(data, f)
