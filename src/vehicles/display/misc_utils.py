from . import contract


@contract(center='seq[2](number)', radius='>0', zorder='int')
def plot_circle(pylab, center, radius, zorder=1000, edgecolor='k',
                facecolor='none',
                alpha=1, **args):
    cir = pylab.Circle((center[0], center[1]), radius=radius, zorder=zorder,
                       edgecolor=edgecolor, facecolor=facecolor, alpha=alpha,
                       **args)
    pylab.gca().add_patch(cir)
