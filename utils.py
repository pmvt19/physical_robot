import logging

def register_logger(logger_name, log_filename, level=logging.INFO, std_err=False):
    logger = logging.getLogger(logger_name)
    logger.setLevel(level)
    handler = logging.FileHandler(f"logs/{log_filename}.log", mode="w")
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    if std_err:
        logger.addHandler(logging.StreamHandler())
    return logger
