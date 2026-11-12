# HVAC PID

*Disclaimer: This library was implemented with the supervised help of AI.*

HVAC PID is a general-purpose Elixir PID controller for discrete control
loops. It supports direct and reverse action, independent P, I, and D
terms, bounded output with anti-windup, a hold deadzone, and smooth
transitions between manual and automatic control. The caller supplies each
sample, its elapsed time, and the actuator interface.

## Installation

Add `hvac_pid` to your dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:hvac_pid, "~> 0.1.0"}
  ]
end
```

The published docs will be at <https://hexdocs.pm/hvac_pid>.

## Quick start

```elixir
{:ok, pid} = HvacPid.Controller.new(out_min: 0, out_max: 100, kp: 8, tn: 20)
{:ok, pid} = HvacPid.Controller.set_mode(pid, :automatic)
{:ok, command, pid} = HvacPid.Controller.step(pid, 21.0, 19.0, 0.25)
```

Here the measurement is 2 units below the setpoint, so the default
`:direct` action gives a first command of 16. Use `direction: :reverse`
when increasing the command lowers the measured value. Pass the returned
`pid` to the next call. The `HvacPid.Controller` module docs explain the
control law and how to handle retuning, pauses, and actuator overrides.

## Autotuning

`HvacPid.Autotuner` can estimate controller gains from a step response or
from relay oscillations. During the experiment, the application sends
samples to the tuner and writes its bounded commands through the usual
actuator interface. The module docs show how to review the result and
resume automatic control from the output currently in effect.

## Development

Run `mix test --cover` for the unit tests, simulated heating and cooling
loops, and the 95% coverage gate. Use `mix deps.get` and `mix docs` to build
the module documentation locally.

Licensed under [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0).
