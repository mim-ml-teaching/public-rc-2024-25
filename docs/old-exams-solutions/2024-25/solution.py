import mujoco
import random
import numpy as np
import math
import cv2
from typing import Tuple


class Detector:
    def __init__(self) -> None:
        pass

    def detect(self, img) -> None:
        pass

    def result(self) -> str:
        return "none"


class DetectorPos:
    def __init__(self) -> None:
        pass

    def detect(self, img) -> Tuple[float, float]:
        return 0, 0
