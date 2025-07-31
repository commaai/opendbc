import numpy as np


def rate_limit(new_value, last_value, dw_step, up_step):
  return float(np.clip(new_value, last_value + dw_step, last_value + up_step))
