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
    'mass': 1.78,#1.2,#75,#1.09,     # kg
    'Ixx':  0.005213, # kg*m^2
    'Iyy':  0.005962, # kg*m^2
    'Izz':  0.017282, # kg*m^2
    'arm_length': 0.225, # meters
    'rotor_speed_min': 0,    # rad/s
    'rotor_speed_max': 1000, # rad/s or 10,000 rpm
    'k_thrust': 4.45e-6, # N/(rad/s)**2
    'k_drag':   2.225e-7, # Nm/(rad/s)**2
    'max_thrust': 31.78 # N or 3.240 [kg] * 9.81 [m/s^2]
}
