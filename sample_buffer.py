import numpy as np

class SampleBuffer:
  def __init__(self, n, chunk_size):
    self.__chunk_size = chunk_size
    self.__data = np.zeros((n, chunk_size))
    self.__size = 0

  def dim(self):
    return self.__data.shape[0]

  def add_sample(self, sample):
    sample = np.array(sample)
    if self.__size >= self.__data.shape[1]:
      col = sample.reshape((self.__data.shape[0], 1))
      pad = np.zeros((self.__data.shape[0], max(self.__chunk_size - 1, 0)))
      self.__data = np.hstack((self.__data, col, pad))

    self.__data[:,self.__size] = sample
    self.__size = self.__size + 1

  def retrieve(self):
    ans = self.__data[:, :self.__size]
    return ans
  
  def size(self):
    return self.__size