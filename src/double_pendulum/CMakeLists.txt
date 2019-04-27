
# Note: no duration arguments to keep this first textbook example simple.
# Just pay the 10 second cost.
add_pytest(simulate.py)
add_nbtest(simulate.ipynb)

add_pytest(manipulator.py)
add_pytest(as_single_pendulum.py --duration 1.0)
