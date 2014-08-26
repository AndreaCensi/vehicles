from vehicles.library.sensors.utils import get_uniform_directions
import numpy as np
from vehicles.library.sensors.raytracer.photoreceptors_smooth import Smoother

def test_smoother():
    
    directions = get_uniform_directions(120, 180)
    upsample = 7
    spatial_sigma_deg = 5
    
    smoother = Smoother(directions=directions, 
                        upsample=upsample, 
                        spatial_sigma_deg=spatial_sigma_deg)
    
    d2 = smoother.get_new_directions()
    d2 = np.array(d2)
    
    
    from vehicles.library.textures.checkerboard import OneEdge
    f = OneEdge(v1=1.0, v0=0.0,where=0,hardness=5.0)
    #from vehicles.library.textures.checkerboard import sigmoid
#     y2 = sigmoid(d2*5)
    radius = 10
    s2 = d2*radius
    y2 = f(s2)
#     y2 = d2* 1.1
    y1 = smoother.smooth(y2)
    
    from reprep import Report
    r = Report()
    r.text('upsample', upsample)
    r.text('spatial_sigma_deg', spatial_sigma_deg)
#     r.text('directions', directions)
    with r.plot('f1') as pylab:
        pylab.plot(np.rad2deg(directions), y1, 'k.') 
        pylab.plot(np.rad2deg(d2), y2, 'g.')
        pylab.xlabel('deg')

    with r.plot('f2') as pylab:
        dd = np.linspace(-10,10,10000)
        f_dd = f(dd)
        pylab.plot(np.rad2deg(dd), f_dd, 'k.') 
    
    with r.plot('coeff_vs_delta') as pylab:
        pylab.plot(np.rad2deg(smoother.delta), smoother.coeff, 'k-')
        pylab.plot(np.rad2deg(smoother.delta), smoother.coeff, 'rs')
        pylab.plot(np.rad2deg(smoother.delta), 0*smoother.coeff, 'b--')


#     r.array_as_image('M', smoother.M)
    
    
    
    r.to_html('test_smoother.html')