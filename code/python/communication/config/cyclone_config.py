from dataclasses import dataclass
import numpy as np


@dataclass
class CycloneConfig:
    num_topics: int = 1
    topic_names: np.ndarray = np.empty(num_topics, dtype=str)
