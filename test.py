import random
import time
import math

offset = {'pan': 22, 'tilt': 0}

def normal_to_radian(vec):
  """
  Converts from camera normalized form to radians.
  """
  return {
    'x': (vec['x']+1.0)*180+offset['pan'],
    'y': -(vec['y']-1.0)*45+offset['tilt']
  }

def radian_to_normal(vec):
  """
  Converts from radians to camera normalized form.
  """
  return {
    'x': 2*((vec['x']-offset['pan'])%360)/360 - 1.0,
    'y':-2*((vec['y']-offset['tilt'])%90)/90 + 1.0
  }

if __name__ == '__main__':
  while True:
    v = {'x': random.uniform(0,360), 'y': random.uniform(0,90)}
    n = radian_to_normal(v)
    vv = normal_to_radian(n)
    print(v, n, vv, all([math.isclose(v[k], vv[k]) for k in v.keys()]))
    time.sleep(1)