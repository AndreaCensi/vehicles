
class Rangefinder(ImageRangeSensor):
    """ This is a very shallow wrap around ImageRangeSensor """
    def sensor_type_string(self):
        return 'rangefinder'
    
    def compute_observations(self, sensor_pose):
        data = self.render(sensor_pose)
        data['sensels'] = data['readings']
        return data

class Nearnessfinder(ImageRangeSensor):
    """ Same as Rangefinder, but we return nearness instead of ranges
    as sensels """
    def sensor_type_string(self):
        return 'nearnessfinder'
    
    def compute_observations(self, sensor_pose):
        data = self.render(sensor_pose)
        data['sensels'] = 1.0 / array(data['readings'])
        return data
    
class Optics(ImageRangeSensor):
    """ This is a very shallow wrap around ImageRangeSensor """
    def sensor_type_string(self):
        return 'optics'
    
    def compute_observations(self, sensor_pose):
        data = self.render(sensor_pose)
        data['sensels'] = data['luminance']
        return data
    
