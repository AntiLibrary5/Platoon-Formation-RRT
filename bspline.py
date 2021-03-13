#bspline.py

def bspline_planning(x, y, sn):
    N = 3
    x = np.array(x) 
    y = np.array(y)
    t = range(len(x))
    x_tup = scipy.interpolate.splrep(t, x, k=N)
    y_tup = scipy.interpolate.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = numpy.linspace(0.0, len(x) - 1, sn)
    rx = scipy.interpolate.splev(ipl_t, x_list)
    ry = scipy.interpolate.splev(ipl_t, y_list)

    return rx, ry


