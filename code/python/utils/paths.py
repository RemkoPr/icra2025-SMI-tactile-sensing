import os
from loguru import logger

data_logging_path = os.path.expanduser("~/data/sensor_comm_dds/")
python_src_root = os.path.dirname(os.path.dirname(__file__))
logger.info(f'Python source root is {python_src_root}')
