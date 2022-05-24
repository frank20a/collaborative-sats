force = 0.7         # Maximum thruster force

slider_tuning = {
    'mpc_state_weights': [
        70,    # position
        0,      # velocity
        7,    # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        50,      # force
        150      # torque
    ],
}
slider_tuning['mpc_final_weights'] = [50 * i for i in slider_tuning['mpc_state_weights']]