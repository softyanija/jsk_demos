import numpy as np

X = np.array([[28, 31, 34, 17],
              [19, 27, 35, 39],
              [56, 36, 52, 56]])

x = np.array([[1,2,3,4],
              [4,3,2,1],
              [2,3,4,1],
              [4,1,2,3]])

x_inv = np.linalg.inv(x)
A = np.dot(X, x_inv)

print(A)
