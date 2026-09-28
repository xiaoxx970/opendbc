"""Car types for the cluster's lead and side cars, published by openpilot's optional yolo_leadd.

yolo_leadd classifies the radar objects the cluster draws (lead, nearest left / right lane car) and
rewrites a small JSON file about ten times per second. Anything missing, malformed or older than
MAX_AGE (the process is off, not installed, or paused) means "car", i.e. the stock icon.
"""
import json
import time

PATH = "/dev/shm/yolo_lead.json"
MAX_AGE = 0.5            # s
READ_PERIOD = 0.1        # s, card calls this at the ACC_19 rate

# ACC_19 Lead_Type / Lead_Type_Left / Lead_Type_Right values, verified on a Golf 8 cluster
CAR_TYPE_PERSON, CAR_TYPE_TRUCK, CAR_TYPE_CAR, CAR_TYPE_MOTORCYCLE, CAR_TYPE_TWO_WHEELER = 1, 2, 3, 4, 5
VALID_TYPES = (CAR_TYPE_PERSON, CAR_TYPE_TRUCK, CAR_TYPE_CAR, CAR_TYPE_MOTORCYCLE, CAR_TYPE_TWO_WHEELER)
ALL_CARS = (CAR_TYPE_CAR, CAR_TYPE_CAR, CAR_TYPE_CAR)


def _car_type(d: dict, slot: str) -> int:
  entry = d.get(slot)
  return entry["type"] if isinstance(entry, dict) and entry.get("type") in VALID_TYPES else CAR_TYPE_CAR


class VisionCarTypes:
  def __init__(self, path: str = PATH, clock=time.monotonic):
    self.path = path
    self.clock = clock
    self.checked = -1e9
    self.types = ALL_CARS
    self.stamp = -1e9

  def get(self) -> tuple[int, int, int]:
    """(lead, left, right) ACC_19 car types."""
    now = self.clock()
    if now - self.checked >= READ_PERIOD:
      self.checked = now
      try:
        with open(self.path) as f:
          d = json.load(f)
        self.types = (_car_type(d, "lead"), _car_type(d, "left"), _car_type(d, "right"))
        self.stamp = float(d["t"])
      except (OSError, ValueError, KeyError, TypeError):
        pass
    return self.types if now - self.stamp < MAX_AGE else ALL_CARS


vision_car_types = VisionCarTypes()
