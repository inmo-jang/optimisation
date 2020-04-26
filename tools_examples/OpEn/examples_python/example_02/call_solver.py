import opengen as og

mng = og.tcp.OptimizerTcpManager('python_build/example_02')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
response = mng.call([-10.0, 5])  # call the solver over TCP


if response.is_ok():
    # Solver returned a solution
    solution_data = response.get()
    u_star = solution_data.solution
    exit_status = solution_data.exit_status
    solver_time = solution_data.solve_time_ms
    print("Solution: x = ", u_star)
    print("Solver_time: = ", solver_time)
else:
    # Invocation failed - an error report is returned
    solver_error = response.get()
    error_code = solver_error.code
    error_msg = solver_error.message



mng.kill()