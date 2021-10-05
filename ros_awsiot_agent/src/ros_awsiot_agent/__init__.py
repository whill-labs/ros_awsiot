#!/usr/bin/python3
import logging

default_logging_handler = logging.StreamHandler()


def set_module_logger(
    modname: str,
    handler: logging.Handler = default_logging_handler,
    level: int = logging.WARN,
) -> None:
    logger = logging.getLogger(modname)
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter("[%(asctime)s][%(name)s][%(levelname)s]: %(message)s")
    )
    logger.addHandler(handler)
    logger.setLevel(level)
