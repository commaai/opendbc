import json

from opendbc.sunnypilot.car.platform_list import get_car_list, CAR_LIST_JSON_OUT


class TestCarList:
  def test_generator(self):
    generated_car_list = json.dumps(get_car_list(), indent=2, ensure_ascii=False)
    with open(CAR_LIST_JSON_OUT) as f:
      current_car_list = f.read()

    assert generated_car_list == current_car_list, "Run opendbc/sunnypilot/car/platform_list.py to update the car list"
