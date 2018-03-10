
add_pytest(torque_slider_demo.py --duration 1.0)

add_pytest(value_iteration.py)

# TODO: pass in specific initial conditions and test for successful balancing.
add_pytest(energy_swingup_and_balance.py --duration 1.0 --trials 2)

add_pytest(dircol_swingup.py)
