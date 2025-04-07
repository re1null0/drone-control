"""
Physical parameters of the laboratory Crazyflie quadrotors.
Additional sources:
    https://bitcraze.io/2015/02/measuring-propeller-rpm-part-3
    https://wiki.bitcraze.io/misc:investigations:thrust
    https://commons.erau.edu/cgi/viewcontent.cgi?article=2057&context=publication
Notes:
    k_thrust is inferred from 14.5 g thrust at 2500 rad/s
    k_drag is mostly made up
"""

quad_params = {
    'mass': 1.25,     # kg
    'Ixx':  0.00592, # kg*m^2
    'Iyy':  0.00592, # kg*m^2
    'Izz':  0.01184, # kg*m^2
    'arm_length': 0.225, # meters
    'rotor_speed_min': 0,    # rad/s
    'rotor_speed_max': 1050, # rad/s or 10,000 rpm
    'k_thrust': 4.45e-6, # N/(rad/s)**2
    'k_drag':   4.45e-7, # Nm/(rad/s)**2
}
