add_pytest(planar_rigid_body_visualizer.py --torque 1.0 --duration 1.0 -a -m pend dpend)
add_pytest(meshcat_rigid_body_visualizer.py --torque 1.0 --duration 1.0 --test -a -m pend dpend)
add_pytest(planar_multibody_visualizer.py --duration 1.0)
