import sample_buffer
import tf_transformations


class ArrayOdomSamplesView:
  def __init__(self, use_timestamp, use_position, use_orientation, data):
    self.use_timestamp = use_timestamp
    self.use_position = use_position
    self.use_orientation = use_orientation
    assert data.shape[0] == self.dim()
    self.data = data

  def dim_of(use_timestamp, use_position, use_orientation):
    return use_timestamp * 1 + use_position * 2 + use_orientation * 1

  def dim(self):
    return ArrayOdomSamplesView.dim_of(self.use_timestamp, self.use_position, self.use_orientation)

  def timestamps(self):
    assert self.use_timestamp
    return self.data[0,:]

  def positions(self):
    assert self.use_position
    start = ArrayOdomSamplesView.dim_of(self.use_timestamp, False, False)
    return self.data[start:start+2,:]

  def angles(self):
    assert self.use_orientation
    start = ArrayOdomSamplesView.dim_of(self.use_timestamp, self.use_position, False)
    return self.data[start:start+1,:]
  
  def retrieve(self):
    return self.data

class OdometrySamples:
  def __init__(self, use_timestamp, use_position, use_orientation, chunk_size):
    assert use_position or use_orientation or use_timestamp

    self.__use_timestamp = use_timestamp
    self.__use_position = use_position
    self.__use_orientation = use_orientation
    self.__buffer = sample_buffer.SampleBuffer(ArrayOdomSamplesView.dim_of(use_timestamp, use_position, use_orientation), chunk_size)

  def as_view(self):
    return ArrayOdomSamplesView(self.__use_timestamp, self.__use_position, self.__use_orientation, self.__buffer.retrieve())

  def add_sample(self, t=None, position=None, orientation=None):
    sample = []
    if self.__use_timestamp:
      assert t is not None
      sample.append(t)
    if self.__use_position:
      assert position is not None
      sample.extend(position[:2])
    if self.__use_orientation:
      assert orientation is not None
      sample.append(tf_transformations.euler_from_quaternion(orientation)[2])
    self.__buffer.add_sample(sample)

  def retrieve(self):
    return self.__buffer.retrieve()

  def size(self):
    return self.__buffer.size()