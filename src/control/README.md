# Using the OpEnGen Optimizer

## Building the optimizer

Download the dependences and run the `generate_optimizer.py` script from inside its folder
- `pip3 install opengen casadi`     The required libraries
- `sudo apt-get install cargo`      The Rust compiler

Be careful to comment-out the unused binding in the following lines:
```py
    .with_build_python_bindings()
    .with_tcp_interface_config()                    # IP 127.0.0.1 and port 8333
```

## Using the optimizer with TCP/IP

```py

mng = og.tcp.OptimizerTcpManager('python_build/the_optimizer')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
response = mng.call([1.0, 50.0])  # call the solver over TCP


if response.is_ok():
    # Solver returned a solution
    solution_data = response.get()
    u_star = solution_data.solution
    exit_status = solution_data.exit_status
    solver_time = solution_data.solve_time_ms
else:
    # Invocation failed - an error report is returned
    solver_error = response.get()
    error_code = solver_error.code
    error_msg = solver_error.message


mng.kill()

```

## Using the optimizer with Python bindings

First import the optimizer:

```py
from ament_index_python import get_package_share_directory
import sys, os
sys.path.insert(1, os.path.join(get_package_share_directory('control'), 'python_build/chaser_mpc')
import chaser_mpc
```

Then use it:

```py
solver = chaser_mpc.solver()
result = solver.run(p=[20., 1.])
u_star = result.solution
```