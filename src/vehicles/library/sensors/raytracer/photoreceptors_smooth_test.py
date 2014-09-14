from vehicles.library.sensors.utils import get_uniform_directions
import numpy as np
from reprep.plot_utils.axes import x_axis_set
from .smoother import Smoother, Smoother2

def test_smoother():
    directions = get_uniform_directions(360, 360)
    upsample = 7
    spatial_sigma_deg = 4
    upsample = 5
    smoother = Smoother2(directions=directions.copy(), 
                        upsample=upsample, 
                        spatial_sigma_deg=spatial_sigma_deg)
    
    d2 = smoother.get_new_directions()
    d2 = np.array(d2)
    
    from vehicles.library.textures.checkerboard import OneEdge
    f = OneEdge(v1=1.0, v0=0.0,where=0,hardness=5.0)
    f = OneEdge(v1=1.0, v0=0.0,where=0,hardness=2.0)
    from scipy.special import expit
    f = lambda x: expit(x.astype('float64')*2.0)
    f = lambda x: np.sign(np.cos(0.2*x))
    #from vehicles.library.textures.checkerboard import sigmoid
#     y2 = sigmoid(d2*5)
    radius = 10
    s2 = d2*radius
    y2 = f(s2)

    print('d2 deg: %s' % np.rad2deg(d2))
#     y2 = d2* 1.1
    y1 = smoother.smooth(y2.copy())
    
    from reprep import Report
    
    r = Report()
    r.text('upsample', upsample)
    r.text('spatial_sigma_deg', spatial_sigma_deg)
#     r.text('directions', directions)

    with r.plot('d2') as pylab:
        pylab.plot(np.rad2deg(d2), '.')
        x_axis_set(pylab, 0, 30)
    with r.plot('y1') as pylab:
        pylab.plot(np.rad2deg(directions), y1, 'k.') 
        pylab.xlabel('deg')
    with r.plot('y2') as pylab: 
        pylab.plot(np.rad2deg(d2), y2, 'g.')
        pylab.xlabel('deg')

    with r.plot('both') as pylab:
        pylab.plot(np.rad2deg(directions), y1, 'k.') 
        pylab.plot(np.rad2deg(d2), y2, 'g.')
        pylab.xlabel('deg')

    with r.plot('f2') as pylab:
        dd = np.linspace(-10,10,10000)
        f_dd = f(dd)
        pylab.plot(np.rad2deg(dd), f_dd, 'k.') 

    if hasattr(smoother, 'coeff'):    
        with r.plot('coeff_vs_delta') as pylab:
            pylab.plot(np.rad2deg(smoother.delta), smoother.coeff, 'k-')
            pylab.plot(np.rad2deg(smoother.delta), smoother.coeff, 'rs')
            pylab.plot(np.rad2deg(smoother.delta), 0*smoother.coeff, 'b--')
    
    
#     r.array_as_image('M', smoother.M)
    fn = 'test_smoother.html'
    r.to_html(fn)
    print(fn)
# 
# def test_exp():
#     from reprep import Report
#     r = Report()
#     xs = get_uniform_directions(120, 180)
#     
#     from scipy.special import expit
# 
#     with r.plot('id' )as pylab:
#         pylab.plot(xs, xs, '.')
#     with r.plot('d' )as pylab:
#         pylab.plot(np.diff(xs), '.')
# 
# 
#     for h in [0.5, 1.0, 1.5, 2.0]:
#         f = lambda x: expit(x*h)
#         y = f(xs)
#         with r.plot('a%s'%h )as pylab:
#             pylab.plot(xs, y, '.')
#     
#     f = 'test_exp.html'
#     r.to_html(f)
#     print(f)
#     
#     