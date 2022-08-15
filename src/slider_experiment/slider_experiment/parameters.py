force = 1.8
torque = 0.54

slider_tuning = {
    'mpc_state_weights': [
        150,    # position
        0,      # velocity
        2,    # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        5,      # force
        150      # torque
    ],
}
slider_tuning['mpc_final_weights'] = [50 * i for i in slider_tuning['mpc_state_weights']]