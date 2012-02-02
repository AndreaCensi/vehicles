from contracts import describe_type
from types import NoneType


def check_yaml_friendly(s):
    try:
        check_yaml_friendly_fast(s)
    except ValueError:
        #pprint(s)
        check_yaml_friendly_detailed(s)


def check_yaml_friendly_fast(s):
    if isinstance(s, (str, int, float, NoneType)):
        return
    elif isinstance(s, list):
        for x in s:
            check_yaml_friendly(x)
    elif isinstance(s, dict):
        for k, v in s.items():
            check_yaml_friendly(k)
            check_yaml_friendly(v)
    else:
        msg = 'Invalid type for YAML serialization %s' % describe_type(s)
        raise ValueError(msg)


def check_yaml_friendly_detailed(s, context=None):
    if context is None:
        context = []
    if isinstance(s, (str, int, float, NoneType)):
        return
    elif isinstance(s, list):
        for i, x in enumerate(s):
            context.append('- %dth element of array' % i)
            check_yaml_friendly_detailed(x, context)
            context.pop()
    elif isinstance(s, dict):
        for k, v in s.items():
            context.append('- key of dictionary')
            check_yaml_friendly_detailed(k, context)
            context.pop()

            context.append('- value of %r' % k)
            check_yaml_friendly_detailed(v, context)
            context.pop()
    else:
        msg = ('Invalid type %s for YAML serialization.\n%s' %
               (describe_type(s), "\n".join(context)))
        raise ValueError(msg)


