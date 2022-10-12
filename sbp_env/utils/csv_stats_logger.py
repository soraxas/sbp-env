import os
import logging
from datetime import datetime


import io
import csv


class CsvFormatter(logging.Formatter):
    def __init__(self):
        super().__init__()
        self.output = io.StringIO()
        self.writer = csv.writer(self.output, quoting=csv.QUOTE_ALL)

    def format(self, record):
        # self.writer.writerow([record.levelname, record.msg])
        self.writer.writerow(record.msg)
        data = self.output.getvalue()
        self.output.truncate(0)
        self.output.seek(0)
        return data.strip()


def get_non_existing_filename(fname_template="runs/%Y-%m-%d_%H-%M{}.csv"):
    """Return a string that represent a path to file that does not exists.
    The filename is the current timestamp."""
    suffix_num = 0
    while True:
        fname = datetime.now().strftime(
            fname_template.format("" if suffix_num == 0 else f".{suffix_num}")
        )
        if not os.path.exists(fname):
            break
        suffix_num += 1
    return fname


def setup_csv_stats_logger(file_name, logger_name="CSV_STATS"):
    # create any missing intermediate folder
    os.makedirs(os.path.dirname(file_name), exist_ok=True)

    # setup a special logger for the logging stats
    csv_logger = logging.getLogger(logger_name)
    csv_logger.setLevel(logging.INFO)

    # only use our own file handle, and disable propagate to console
    csv_logger.propagate = False

    # setup our handler
    fh = logging.FileHandler(file_name)
    fh.setFormatter(CsvFormatter())
    csv_logger.addHandler(fh)
