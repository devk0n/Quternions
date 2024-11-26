function q_out = quatMultiply(q, r)
    w0 = q(1); x0 = q(2); y0 = q(3); z0 = q(4);
    w1 = r(1); x1 = r(2); y1 = r(3); z1 = r(4);
    q_out = [
        w0*w1 - x0*x1 - y0*y1 - z0*z1;
        w0*x1 + x0*w1 + y0*z1 - z0*y1;
        w0*y1 - x0*z1 + y0*w1 + z0*x1;
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ];
end