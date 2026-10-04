"""Application logging; configure output only at the CLI entry point."""

import logging


LOGGER_NAMESPACE = "wg_lo_filter"
LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
CONSOLE_HANDLER_NAME = "wg-lo-filter-console"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_DATE_FORMAT = "%H:%M:%S"

# Importing utility functions should not itself install terminal output.
logging.getLogger(LOGGER_NAMESPACE).addHandler(logging.NullHandler())


def get_logger(name):
    return logging.getLogger(f"{LOGGER_NAMESPACE}.{name}")


def add_logging_arguments(parser):
    parser.add_argument(
        "--log-level", type=str.upper, choices=LOG_LEVELS, default="INFO",
        help="出力するログの最低レベル（既定: INFO）。DEBUGで詳細ステータスを表示します。",
    )


def configure_logging(level="INFO"):
    """Set this application's console threshold without changing root logging."""
    if not isinstance(level, str) or level.upper() not in LOG_LEVELS:
        raise ValueError(f"Invalid log level: {level}")
    application_logger = logging.getLogger(LOGGER_NAMESPACE)
    application_logger.setLevel(level.upper())
    application_logger.propagate = False
    # Replace only our own handler, so repeated configuration does not duplicate output.
    for handler in list(application_logger.handlers):
        if handler.get_name() == CONSOLE_HANDLER_NAME:
            application_logger.removeHandler(handler)
            handler.close()
    console = logging.StreamHandler()  # Standard logging output goes to stderr.
    console.set_name(CONSOLE_HANDLER_NAME)
    console.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
    application_logger.addHandler(console)
