from datetime import datetime


class EventLog(object):
    def __init__(self, datetime_format='ISO 8601'):
        format_lookup = {"ISO 8601": "[%Y-%m-%dT%H:%M:%S]"}
        self.datetime_format = format_lookup[datetime_format]

    def entry(self, classification, message):
        entry = datetime.strftime(datetime.now(), self.datetime_format)
        entry = entry + " ({0}) ".format(classification)
        entry = entry + message
        print(entry)
