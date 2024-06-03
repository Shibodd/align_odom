import point_set_registration
import numpy as np

m = np.random.random(2)
A = np.array([[m[0], -m[1]],
              [m[1],  m[0]]]) * np.random.random() * 5
T = np.random.random((2,1))

# S = np.linalg.norm(A)
# R = A / S

x = np.random.random((2, 20))
y = np.matmul(A, x) + T


print(A)
print(T)
print(point_set_registration.lsq_2d_rototranslation_uniform_scaling(x, y))