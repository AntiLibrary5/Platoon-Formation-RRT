function [rx,ry] = bspline_planning(x, y)
    N = 3;
    t = py.range(py.len(x));
    x_tup = py.scipy.interpolate.splrep(t, x);
    y_tup = py.scipy.interpolate.splrep(t, y);

    x_list = py.list(x_tup);
    xl = py.list(x);
    q = [0.0, 0.0, 0.0, 0.0];
    q=py.list(q);
    x_list{2} = xl+q;

    y_list = py.list(y_tup);
    yl = py.list(y);
    y_list{2} = yl + q;

    ipl_t = py.numpy.linspace(0.0, py.len(x) - 1, 100);%sn = 100
    rx = py.scipy.interpolate.splev(ipl_t, x_list);
    ry = py.scipy.interpolate.splev(ipl_t, y_list);
    rx = double(rx);
    ry = double(ry);

    end

