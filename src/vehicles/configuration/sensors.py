from conf_tools import check_generic_code_desc


def check_valid_sensor_config(x):
    return check_generic_code_desc(x, 'sensor')

def check_valid_skin_config(x):
    return check_generic_code_desc(x, 'skin')
