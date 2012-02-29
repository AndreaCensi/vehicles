import datetime


# TODO: move away
def unique_timestamp_string():
    now = datetime.datetime.now()
    s = now.isoformat()
    s = s.replace('-', '').replace(':', '')
    s = s.replace('T', '_').replace('.', '_')
    return s


def isodate():
    now = datetime.datetime.now()
    date = now.isoformat('-')[:19]
    return date
