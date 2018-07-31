# Moveback recovery
A simple recovery behavior that moves the robot back. Implements the move base flex interface.
See the `config/planner_sample.yaml` for an example configuration.

# Parameters
`controller_frequency` (Default 20.0): The frequency velocity commands are published

`linear_vel_back` (Defualt -0.3): The velocity for the robot. Note the minus; it's oriented.

`step_back_length` (Default 1.0): The total length of the way moved back

`step_back_timeout` (Default 15.0): If the moveback action takes longer then this timeout, it will be aborted.
