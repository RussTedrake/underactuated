
add_pytest(force_slider_demo.py --duration 0.1)

# TODO: pass in specific initial conditions and test for successful balancing.
add_pytest(balancing_lqr.py --duration 0.1 --trials 1)

add_pytest(dircol_swingup.py)
