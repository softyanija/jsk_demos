import numpy as np

A = np.array([[2, 4, 5, 0],
              [9, 2, 1, 0],
              [6, 2, 3, 9]])

x = np.array([[1,2,3,4],
              [4,3,2,1],
              [2,3,4,1],
              [4,1,2,3]])
X = np.dot(A, x)

print(X)

#[[28, 31, 34, 17]
# [19, 27, 35, 39]
# [56, 36, 52, 56]]

