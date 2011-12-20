

#def add_blind_vehicles():
#    for id_dynamics in Configuration.dynamics:
#        id_vehicle = '_blind-%s' % id_dynamics
#        vehicle = {
#           'id': id_vehicle,
#           'desc': 'Blind vehicle with dynamics %s.' % id_dynamics,
#           'id_dynamics': id_dynamics,
#           'sensors': [{'id_sensor': 'random_5', 'pose': [0, 0, 0]}],
#           'radius': 0.5
#        }
#        check_valid_vehicle_config(vehicle)
#        Configuration.vehicles[id_vehicle] = vehicle
