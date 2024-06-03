import numpy as np

def get_overlapping_ranges(a: np.ndarray, b: np.ndarray):
  self_ts = a
  other_ts = b

  if self_ts.size == 0 or other_ts.size == 0 or self_ts[0] > other_ts[-1] or self_ts[-1] < other_ts[0]:
    return (range(0), range(0))

  def ith_or_default(r, i, default):
    r = np.nonzero(r)[0]
    if r.size == 0:
      return default
    return r[i]
  def first_or_default(r, default):
    return ith_or_default(r, 0, default)
  def last_or_default(r, default):
    return ith_or_default(r, -1, default)

  # Smallest subset of self which "contains" (in time sense) the maximum amount of samples from other
  self_start = last_or_default(self_ts <= other_ts[0], 0)
  self_end = first_or_default(self_ts >= other_ts[-1], self_ts.size - 1)

  # The samples from other that are "contained" (time) in the subset we just found, or None if there are none
  other_start = first_or_default(other_ts >= self_ts[self_start], None)
  other_end = last_or_default(other_ts <= self_ts[self_end], None)

  if other_start is None or other_end is None:
    return (range(0), range(0))
  
  return (range(self_start, self_end+1), range(other_start, other_end+1))

if __name__ == '__main__': # look ma! no unit test framework!
  def assert_overlap(a, b, ans):
    assert get_overlapping_ranges(a, b) == ans, "Test failed"

  # No overlap
  assert_overlap(np.array([]), np.array([]), None)
  assert_overlap(np.array([]), np.array([1,2]), None)
  assert_overlap(np.array([1,2]), np.array([]), None)
  assert_overlap(np.array([1,2,3]), np.array([4,5,6]), None)
  assert_overlap(np.array([4,5,6]), np.array([1,2,3]), None)

  # a larger than b
  assert_overlap(
    np.array([0,1,2,3,4]),
    np.array([1,2,3]),
    ((1, 3), (0, 2)))

  # a larger than b, left ext
  assert_overlap(
    np.array([0,1,2,3,4]),
    np.array([1.5,2,3]),
    ((1,3), (0,2)))

  # a larger than b, right ext
  assert_overlap(
    np.array([0,1,2,3,4]),
    np.array([1.5,2,3.5]),
    ((1,4), (0,2)))

  # overlap, but b ends later
  assert_overlap(
    np.array([0,1,2,3,4]),
    np.array([3.5, 50]),
    ((3,4), (0,0)))

  # overlap, but b starts earlier
  assert_overlap(
    np.array([0,1,2,3,4]),
    np.array([-20, 0.5]),
    ((0,1), (1,1)))
  
  # overlap, but b contains a
  assert_overlap(
    np.array([2,3,4]),
    np.array([0, 2.5, 3.5, 4.5]),
    ((0,2), (1,2)))

  # full overlap
  assert_overlap(
    np.array([1,2,3]),
    np.array([1,2,3]),
    ((0,2), (0,2))
  )