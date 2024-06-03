import numpy as np
from odom_samples import ArrayOdomSamplesView

def __lsq_2d_generic_translation(pos_x: np.ndarray, pos_y: np.ndarray, compute_t: callable):
  centroid_x = np.mean(pos_x, axis=1).reshape((2,1))
  centroid_y = np.mean(pos_y, axis=1).reshape((2,1))
  qx = pos_x - centroid_x
  qy = pos_y - centroid_y

  T = compute_t(ArrayOdomSamplesView(False, True, False, qx), ArrayOdomSamplesView(False, True, False, qy))
  T[:2, 2] = (centroid_y - np.dot(T[:2,:2], centroid_x)).reshape((2,))
  return T

def __lsq_2d_generic_notranslation(x: np.ndarray, y: np.ndarray, H: np.ndarray, d):
  X = (x.transpose() @ H).reshape((-1,d))
  Y = y.transpose().reshape((-1, 1))
  
  m, residuals, _, _ = np.linalg.lstsq(X, Y, rcond=None)
  # residuals / (x.shape[1] ** 2) # TODO: this should be useful for checking the result quality
  return m.flatten()

def lsq_2d_rotation_uniform_scaling_translation(x: ArrayOdomSamplesView, y: ArrayOdomSamplesView):
  assert(x.use_position and y.use_position)
  return __lsq_2d_generic_translation(x.positions(), y.positions(), lsq_2d_rotation_uniform_scaling)

def lsq_2d_free(x: ArrayOdomSamplesView, y:  ArrayOdomSamplesView):
  assert(x.use_position and y.use_position)
  return __lsq_2d_generic_translation(x.positions(), y.positions(), lsq_2d_free_notranslation)

def lsq_2d_free_notranslation(x: ArrayOdomSamplesView, y: ArrayOdomSamplesView):
  if x.use_position and y.use_position and x.use_orientation and y.use_orientation:
    H = np.array([[1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1]])
    d = 4

    x_data = np.vstack((
      x.positions(),
      np.cos(x.angles()),
      np.sin(x.angles())
    ))
    y_data = np.vstack((
      y.positions(),
      np.cos(y.angles()),
      np.sin(y.angles())
    ))

  elif x.use_position and y.use_position:
    H = np.array([[1, 0, 0, 0, 0, 0, 1, 0],
                  [0, 1, 0, 0, 0, 0, 0, 1]])
    d = 4

    x_data = x.positions()
    y_data = y.positions()
  else:
    assert False
  
  m = __lsq_2d_generic_notranslation(x_data, y_data, H, d)
  return np.array([[ m[0], m[1], 0],
                   [ m[2], m[3], 0],
                   [    0,    0, 1]])

def lsq_2d_rotation_uniform_scaling(x: ArrayOdomSamplesView, y: ArrayOdomSamplesView):
  assert(x.use_position and y.use_position)
  H = np.array(([ 1,  0, 0, 1 ],
                [ 0, -1, 1, 0 ]))
  m = __lsq_2d_generic_notranslation(x.positions(), y.positions(), H, 2)
  return np.array([[ m[0], -m[1], 0 ],
                   [ m[1],  m[0], 0 ],
                   [    0,     0, 1 ]])

def cpd_2d_rotation_uniform_scaling_translation(x: ArrayOdomSamplesView, y: ArrayOdomSamplesView):
  assert(x.use_position and y.use_position)
  
  import pycpd
  reg = pycpd.RigidRegistration(np.eye(2), np.zeros((1,2)), 1, X=y.positions().transpose(), Y=x.positions().transpose())

  _, (s_reg, R_reg, t_reg) = reg.register()
  R_reg = R_reg.transpose()

  return np.vstack((
    np.hstack((s_reg * R_reg, t_reg.reshape((2,1)))),
    np.array([0, 0, 1])
  ))