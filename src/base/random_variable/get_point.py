import numpy as np

def get_point(mean, cov):
    return list(np.random.multivariate_normal(mean, cov))

def get_point_dim2(mean_1, mean_2, cov_11, cov_22, cov_12):
    return get_point([mean_1, mean_2], [[cov_11, cov_12], [cov_12, cov_22]])

if __name__ == '__main__':
    print(get_point_dim2(1, 2, 1, 2, 0))
    print(get_point([1, 2], [[1, 0], [0, 2]]))
