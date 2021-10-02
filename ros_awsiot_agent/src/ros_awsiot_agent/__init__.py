#!/usr/bin/python3
import logging

def set_module_logger(modname: str, handler: logging.Handler = logging.StreamHandler(), level: int = logging.WARN):
    logger=logging.getLogger(modname)
    handler.setFormatter(logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s]: %(message)s'))
    logger.addHandler(handler)
    logger.setLevel(level)
