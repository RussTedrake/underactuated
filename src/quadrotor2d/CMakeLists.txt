
add_pystyle_test(quadrotor2d.py)

add_pytest(lqr.py --duration 0.1 --trials 1)
add_pytest(differential_flatness.py)
