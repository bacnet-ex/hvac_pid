defmodule HvacPid.Controller do
  @moduledoc """
  A general-purpose PID controller for discrete control loops. The caller
  supplies the clock, samples, and actuator interface.

  Keep one state per loop and pass the state returned by each operation into
  the next one. The controller starts disabled. Once enabled, it computes
  commands as samples arrive through `step/4`.

  ## Choosing a direction

  Choose the direction based on how a larger command affects the measured
  value.

  - `:direct` is the default. The signed error is `setpoint - measurement`,
    so it is positive when the measured value is below the setpoint. Use
    this direction when a larger command raises the measured value, as
    with a heater.
  - `:reverse` uses `measurement - setpoint`, so the error is positive when
    the measured value is above the setpoint. Use this direction when a
    larger command lowers the measured value, as with a cooler.

  The direction sets the sign convention for all three PID terms.

  ## Control law

  The controller uses the positional form of PID. The integral uses the
  current error over the whole sample interval (rectangular integration).
  The derivative uses the difference between successive samples.

  ```text
  s       = +1 for direct action, -1 for reverse action
  e       = s * (setpoint - measurement)
  P       = Kp * e
  delta_I = Kp * (elapsed_seconds / Tn) * e
  change  = -s * (measurement - previous_measurement)    on measurement
  change  = e - previous_error                          on error
  D       = Kp * Td * (change / elapsed_seconds)
  raw     = bias + tracking_adjustment + P + I + D
  output  = clamp(raw, out_min, out_max)
  ```

  `Kp` is in output units per measurement unit. `Tn` and `Td` are in seconds.
  `P`, `I`, `D`, the bias, and the tracking adjustment are all in output
  units. Set `tn` to enable integral action and `td` to enable derivative
  action. Set either to `nil` to switch that term off. You can switch off the
  proportional term with `p_enabled: false` while keeping `kp` for I and D.

  The derivative defaults to `:measurement`, so moving the setpoint alone
  leaves D at zero. With `derivative_on: :error`, a changing setpoint also
  contributes to D. That can help a loop follow a ramped setpoint. A sudden
  setpoint change can produce a large one-sample response called derivative
  kick. Both sources give the same derivative at a fixed setpoint.

  ### Saturation and the deadzone

  Before adding the new integral increment, the controller computes the raw
  output with the previous integral. A positive increment can use the
  headroom up to the upper bound. A negative increment can use the headroom
  down to the lower bound. The controller discards any remainder, then clamps
  the output. This prevents the integral from accumulating in the direction
  of saturation while allowing it to help bring the output back into range.
  The limits apply to the combined output. For instance, I can be negative
  with an output range of 0 to 100.

  A positive `deadzone` holds the previous output when the absolute error is
  at or below that radius. The integral and tracking adjustment stay fixed.
  Each held sample still updates the derivative history, so on exit the
  derivative uses the sample immediately before the exit. A zero radius
  disables the hold. Startup and reanchoring take precedence over the hold.

  ## Startup, manual control, and retuning

  Call `set_mode/2` with `:automatic` to start. On the first valid sample,
  the controller emits the clamped `start_value` if one is configured.
  Otherwise it emits the clamped sum of the bias and P. I and D start at zero
  and run from the second sample onward. Setting `:disabled` clears the
  controller's history and immediately selects the clamped `stop_value` or,
  by default, the lower bound. Read that command with `output/1`.

  Manual mode selects a command immediately. If an external override applies
  a different command, report the effective value before returning to
  automatic control. For example, the controller can request 35% while an
  actuator's priority logic holds it at 42%:

      iex> {:ok, pid} = HvacPid.Controller.new(out_min: 0, out_max: 100, kp: 10)
      iex> {:ok, pid} = HvacPid.Controller.set_mode(pid, {:manual, 35})
      iex> {:ok, pid} = HvacPid.Controller.track_output(pid, 42)
      iex> HvacPid.Controller.output(pid)
      35.0
      iex> {:ok, pid} = HvacPid.Controller.set_mode(pid, :automatic)
      iex> {:ok, 42.0, pid} = HvacPid.Controller.step(pid, 21, 19, 1)
      iex> {:ok, command, _pid} = HvacPid.Controller.step(pid, 21, 20, 1)
      iex> command
      32.0

  After a manual transfer, a change to the active control law, or an explicit
  call to `track_output/2`, the next automatic sample emits the clamped
  target. The controller resets I and D and sets its tracking adjustment to
  `target - bias - P` using the fresh sample. The adjustment remains in the
  control law on later samples. This gives proportional-only loops the same
  smooth transfer as loops with an integral term. Tuning changes can be
  grouped in one call to `configure/2`.

  ## Timing, pauses, and retries

  Pass the actual elapsed seconds between accepted samples. If you timestamp
  samples with `System.monotonic_time(:microsecond)`, divide the difference by
  `1_000_000`. For millisecond timestamps, divide by `1_000`. For example,
  1 ms is `0.001` seconds and 250 ms is `0.25` seconds. On the first sample
  after startup or reanchoring, any valid positive interval works because
  that step skips the time-dependent terms.

  You can skip a sample if its timestamp matches the previous one. For a
  longer pause, decide how the actuator should behave while data is missing.
  When you resume, call `track_output/2` with the command currently in effect.
  The next step uses the fresh measurement to seed D and starts a new
  integration interval without integrating across the gap.

  A rejected sample leaves the old state available for retry. If you continue
  from that state, measure the next interval from the last accepted sample.
  Similarly, if a write fails and you need to resume control from a command
  the actuator actually applied, report that value before stepping again.

  ## A PI example

  With a gain of 8, each unit of signed error contributes 8 units to the
  proportional term. The default direct direction gives a positive error
  of 2 when the measurement is 19 and the setpoint is 21.

      iex> {:ok, pid} = HvacPid.Controller.new(out_min: 0, out_max: 100, kp: 8, tn: 20)
      iex> HvacPid.Controller.output(pid)
      0.0
      iex> {:ok, pid} = HvacPid.Controller.set_mode(pid, :automatic)
      iex> {:ok, command, pid} = HvacPid.Controller.step(pid, 21, 19, 0.25)
      iex> command
      16.0
      iex> {:ok, command, _pid} = HvacPid.Controller.step(pid, 21, 19, 0.25)
      iex> command
      16.2

  With the same tuning in reverse, a measurement of 23 gives the same
  positive error and first command.

      iex> {:ok, pid} = HvacPid.Controller.new(out_min: 0, out_max: 100, kp: 8, tn: 20, direction: :reverse)
      iex> {:ok, pid} = HvacPid.Controller.set_mode(pid, :automatic)
      iex> {:ok, command, _pid} = HvacPid.Controller.step(pid, 21, 23, 0.25)
      iex> command
      16.0
  """

  # Internal configuration state
  @typep configuration :: %{
           out_min: float(),
           out_max: float(),
           kp: float(),
           p_enabled: boolean(),
           tn: float() | nil,
           td: float() | nil,
           derivative_on: :measurement | :error,
           bias: float(),
           deadzone: float(),
           direction: :direct | :reverse,
           start_value: float() | nil,
           stop_value: float() | nil
         }

  @typedoc """
  The reason an operation was rejected.

  Option errors identify a missing bound, an unknown or duplicate key, or an
  invalid value. `:invalid_options` means the options were given in a form
  other than a keyword list. `:invalid_output_limits` means the lower bound
  exceeded the upper bound.

  An invalid input error identifies the affected argument. A
  `:numeric_overflow` means a calculation exceeded the available floating
  point range even though the supplied inputs were valid.
  """
  @type error ::
          :invalid_options
          | {:unknown_option, atom()}
          | {:duplicate_option, atom()}
          | {:missing_option, :out_min | :out_max}
          | {:invalid_option, atom()}
          | :invalid_output_limits
          | :invalid_mode
          | {:invalid_input,
             :setpoint | :measurement | :elapsed_seconds | :manual_output | :applied_output}
          | :numeric_overflow

  @typedoc """
  A controller mode or a manual command in output units.
  """
  @type mode :: :disabled | :automatic | {:manual, number()}

  @typedoc """
  One configuration setting. See `new/1` for defaults and units.
  """
  @type option ::
          {:out_min | :out_max | :kp | :bias | :deadzone, number()}
          | {:tn | :td | :start_value | :stop_value, number() | nil}
          | {:p_enabled, boolean()}
          | {:direction, :direct | :reverse}
          | {:derivative_on, :measurement | :error}

  @typedoc """
  Opaque controller state returned by the controller operations.
  """
  @opaque t :: %__MODULE__{
            config: configuration(),
            mode: :disabled | :manual | :automatic,
            output: float(),
            pending: :startup | float() | nil,
            sample: {float(), float(), float() | nil} | nil,
            integral: float(),
            tracking: float()
          }

  @fields [:config, :mode, :output, :pending, :sample, :integral, :tracking]
  @enforce_keys @fields
  defstruct @fields

  @defaults %{
    kp: 1.0,
    p_enabled: true,
    tn: nil,
    td: nil,
    derivative_on: :measurement,
    bias: 0.0,
    deadzone: 0.0,
    direction: :direct,
    start_value: nil,
    stop_value: nil
  }
  @retuning_keys [
    :out_min,
    :out_max,
    :kp,
    :p_enabled,
    :tn,
    :td,
    :derivative_on,
    :bias,
    :direction
  ]

  @doc """
  Creates a disabled controller with the specified settings.

  | Option | Default | Meaning |
  | --- | --- | --- |
  | `:out_min`, `:out_max` | Required | Lower and upper output bounds. Equal bounds give a fixed command. |
  | `:kp` | `1.0` | Shared, nonnegative gain in output units per measurement unit. |
  | `:p_enabled` | `true` | Whether to include the proportional term. |
  | `:tn` | `nil` | Positive integral time in seconds, or `nil` to disable I. |
  | `:td` | `nil` | Positive derivative time in seconds, or `nil` to disable D. |
  | `:derivative_on` | `:measurement` | Use changes in measurement, or include setpoint changes with `:error`. |
  | `:bias` | `0.0` | Constant contribution in output units. |
  | `:deadzone` | `0.0` | Nonnegative hold radius in measurement units. Zero disables the hold. |
  | `:direction` | `:direct` | Signed error is setpoint minus measurement in `:direct` and measurement minus setpoint in `:reverse`. |
  | `:start_value` | `nil` | Optional output for the first automatic sample after enabling. |
  | `:stop_value` | `nil` | Optional output for disabled mode. The lower bound is used when omitted. |

  Numeric options accept integers and floats that can be represented as
  finite floats. Each key can appear once. See `t:error/0` for errors.

      iex> {:ok, pid} = HvacPid.Controller.new(out_min: -20, out_max: 80, stop_value: 0)
      iex> HvacPid.Controller.output(pid)
      0.0
      iex> HvacPid.Controller.new(out_max: 100)
      {:error, {:missing_option, :out_min}}
  """
  @spec new([option()]) :: {:ok, t()} | {:error, error()}
  def new(opts) do
    with {:ok, config} <- configuration(opts, @defaults) do
      {:ok,
       %__MODULE__{
         config: config,
         mode: :disabled,
         output: stop_output(config),
         pending: nil,
         sample: nil,
         integral: 0.0,
         tracking: 0.0
       }}
    end
  end

  @doc """
  Applies a batch of settings to an existing controller.

  Any options omitted from the batch keep their current values. Both bounds
  are checked together, so a range can be moved in one operation. An invalid
  batch returns an error and leaves the caller's state available.

  During automatic control, a change to gain, term selection, derivative
  source, direction, bias, or bounds reanchors the next sample to the current
  command or pending tracked value. Further changes before that sample retain
  the target within the new bounds. Changing only the deadzone or the start
  and stop values keeps the current control history. A controller still
  awaiting its startup sample uses the latest settings for startup.

  Bounds clamp the current command immediately. Widening the range later
  keeps the clamped manual command. In disabled mode the command is
  recalculated from the new stop value and bounds.

      iex> {:ok, pid} = HvacPid.Controller.new(out_min: 0, out_max: 100)
      iex> {:ok, pid} = HvacPid.Controller.configure(pid, out_min: 120, out_max: 200, kp: 3)
      iex> HvacPid.Controller.output(pid)
      120.0
  """
  @spec configure(t(), [option()]) :: {:ok, t()} | {:error, error()}
  def configure(%__MODULE__{} = state, opts) do
    with {:ok, config} <- configuration(opts, state.config) do
      {:ok, apply_configuration(state, config)}
    end
  end

  @doc """
  Selects disabled, automatic, or manual operation.

  Disabled and manual commands take effect immediately. Automatic operation
  waits for a fresh sample. When coming from manual mode it anchors to the
  last manual command or to the most recent tracked value. Reapplying the
  current mode is idempotent. Two manual requests that clamp to the same
  command count as the same mode.

  Returns `{:error, :invalid_mode}` for an unsupported mode, or
  `{:error, {:invalid_input, :manual_output}}` for an invalid manual value.
  """
  @spec set_mode(t(), mode()) :: {:ok, t()} | {:error, error()}
  def set_mode(state, mode)

  def set_mode(%__MODULE__{} = state, :disabled) do
    {:ok,
     %{
       state
       | mode: :disabled,
         output: stop_output(state.config),
         pending: nil,
         sample: nil,
         integral: 0.0,
         tracking: 0.0
     }}
  end

  def set_mode(%__MODULE__{mode: :automatic} = state, :automatic), do: {:ok, state}

  def set_mode(%__MODULE__{mode: :disabled} = state, :automatic) do
    {:ok, %{state | mode: :automatic, pending: :startup}}
  end

  def set_mode(%__MODULE__{mode: :manual} = state, :automatic) do
    {:ok, %{state | mode: :automatic, pending: state.pending || state.output}}
  end

  def set_mode(%__MODULE__{} = state, {:manual, value}) do
    with {:ok, value} <- input_number(:manual_output, value) do
      command = clamp(value, state.config)

      if state.mode == :manual and command == state.output do
        {:ok, state}
      else
        {:ok, %{state | mode: :manual, output: command, pending: nil}}
      end
    end
  end

  def set_mode(%__MODULE__{}, _), do: {:error, :invalid_mode}

  @doc """
  Reports the output currently applied at the actuator.

  In automatic mode the latest report becomes the target for the next
  sample, including when reported between enabling and the startup sample.
  In manual mode it supplies the later transfer target while the manual
  command stays in effect. Selecting a different manual command clears that
  target. Disabled mode keeps its stop command and starts afresh on enable.

  Tracking clamps the reported value to the bounds and leaves `output/1`
  reporting the current command until the next automatic sample. Returns
  `{:error, {:invalid_input, :applied_output}}` for an invalid value.
  """
  @spec track_output(t(), number()) :: {:ok, t()} | {:error, error()}
  def track_output(%__MODULE__{} = state, applied_output) do
    with {:ok, applied_output} <- input_number(:applied_output, applied_output) do
      if state.mode == :disabled do
        {:ok, state}
      else
        {:ok, %{state | pending: clamp(applied_output, state.config)}}
      end
    end
  end

  @doc """
  Advances the controller using a setpoint, measurement, and elapsed seconds.

  The setpoint and measurement must share units. All three arguments must
  be finite numbers, and elapsed seconds must be strictly positive. Fractional
  intervals are used as supplied. Manual and disabled steps validate their
  inputs and return the current command. Manual steps also remember the
  sample, while transfer to automatic control seeds history from a fresh one.

  Returns `{:ok, command, state}` on success, or `{:error, reason}` for
  invalid input or numeric overflow. See `t:error/0`.
  """
  @spec step(t(), number(), number(), number()) :: {:ok, float(), t()} | {:error, error()}
  def step(%__MODULE__{} = state, setpoint, measurement, elapsed_seconds) do
    with {:ok, setpoint} <- input_number(:setpoint, setpoint),
         {:ok, measurement} <- input_number(:measurement, measurement),
         {:ok, elapsed} <- input_number(:elapsed_seconds, elapsed_seconds) do
      sample(state, setpoint, measurement, elapsed)
    end
  rescue
    ArithmeticError -> {:error, :numeric_overflow}
  end

  @doc """
  Returns the most recent output command in output units.

  Manual and disabled mode changes update the command immediately. This
  value represents the controller's command. Use `track_output/2` to report
  the effective actuator output when an override applies.
  """
  @spec output(t()) :: float()
  def output(%__MODULE__{output: output}), do: output

  @spec configuration([option()], map()) :: {:ok, configuration()} | {:error, error()}
  defp configuration(opts, base) do
    if Keyword.keyword?(opts) do
      with {:ok, updates} <- options(opts) do
        validate_limits(Map.merge(base, updates))
      end
    else
      {:error, :invalid_options}
    end
  end

  @spec options([option()]) :: {:ok, map()} | {:error, error()}
  defp options(opts) do
    Enum.reduce_while(opts, {:ok, %{}}, fn {key, value}, {:ok, updates} ->
      if Map.has_key?(updates, key) do
        {:halt, {:error, {:duplicate_option, key}}}
      else
        case option_value(key, value) do
          {:ok, value} -> {:cont, {:ok, Map.put(updates, key, value)}}
          {:error, _} = error -> {:halt, error}
        end
      end
    end)
  end

  @spec option_value(atom(), term()) :: {:ok, term()} | {:error, error()}
  defp option_value(key, value)

  defp option_value(:p_enabled, value) when is_boolean(value), do: {:ok, value}

  defp option_value(:direction, value) when value in [:direct, :reverse], do: {:ok, value}

  defp option_value(:derivative_on, value) when value in [:measurement, :error], do: {:ok, value}

  defp option_value(key, nil) when key in [:tn, :td, :start_value, :stop_value], do: {:ok, nil}

  defp option_value(key, value)
       when key in [:out_min, :out_max, :bias, :start_value, :stop_value],
       do: option_number(key, value)

  defp option_value(key, value) when key in [:kp, :deadzone] and is_number(value) and value >= 0,
    do: option_number(key, value)

  defp option_value(key, value) when key in [:tn, :td] and is_number(value) and value > 0,
    do: option_number(key, value)

  defp option_value(key, _) when is_map_key(@defaults, key),
    do: {:error, {:invalid_option, key}}

  defp option_value(key, _), do: {:error, {:unknown_option, key}}

  @spec option_number(atom(), term()) :: {:ok, float()} | {:error, error()}
  defp option_number(key, value) do
    case normalize_number(value) do
      {:ok, number} -> {:ok, number}
      :error -> {:error, {:invalid_option, key}}
    end
  end

  @spec input_number(atom(), term()) :: {:ok, float()} | {:error, error()}
  defp input_number(key, value) do
    case normalize_number(value) do
      {:ok, number} when key != :elapsed_seconds or number > 0.0 -> {:ok, number}
      _ -> {:error, {:invalid_input, key}}
    end
  end

  @spec normalize_number(term()) :: {:ok, float()} | :error
  defp normalize_number(value)

  defp normalize_number(value) when is_number(value) do
    {:ok, :erlang.float(value)}
  rescue
    ArgumentError -> :error
  end

  defp normalize_number(_), do: :error

  @spec validate_limits(map()) :: {:ok, configuration()} | {:error, error()}
  defp validate_limits(config)

  defp validate_limits(%{out_min: low, out_max: high} = config) when low <= high,
    do: {:ok, config}

  defp validate_limits(%{out_min: _, out_max: _}), do: {:error, :invalid_output_limits}
  defp validate_limits(%{out_min: _}), do: {:error, {:missing_option, :out_max}}
  defp validate_limits(_), do: {:error, {:missing_option, :out_min}}

  @spec apply_configuration(t(), configuration()) :: t()
  defp apply_configuration(state, config)

  defp apply_configuration(%{mode: :disabled} = state, config) do
    %{state | config: config, output: stop_output(config)}
  end

  defp apply_configuration(state, config) do
    output = clamp(state.output, config)

    pending =
      cond do
        is_float(state.pending) -> clamp(state.pending, config)
        state.pending == :startup -> :startup
        state.mode == :automatic and retuned?(state.config, config) -> output
        true -> nil
      end

    %{state | config: config, output: output, pending: pending}
  end

  @spec retuned?(configuration(), configuration()) :: boolean()
  defp retuned?(old, new), do: Map.take(old, @retuning_keys) != Map.take(new, @retuning_keys)

  @spec stop_output(configuration()) :: float()
  defp stop_output(config), do: clamp(config.stop_value || config.out_min, config)

  @spec sample(t(), float(), float(), float()) :: {:ok, float(), t()}
  defp sample(state, setpoint, measurement, elapsed)

  defp sample(%{mode: :disabled} = state, _, _, _) do
    {:ok, state.output, state}
  end

  defp sample(%{mode: :manual} = state, setpoint, measurement, _) do
    {:ok, state.output, %{state | sample: {setpoint, measurement, nil}}}
  end

  defp sample(%{mode: :automatic} = state, setpoint, measurement, elapsed) do
    sign = direction_sign(state.config.direction)
    error = sign * (setpoint - measurement)
    state = control(state, {setpoint, measurement, error}, elapsed, sign)
    {:ok, state.output, state}
  end

  @spec control(t(), {float(), float(), float()}, float(), float()) :: t()
  defp control(state, sample, elapsed, sign)

  defp control(%{pending: :startup, config: config} = state, {_, _, error} = sample, _, _) do
    p = proportional(config, error)

    case config.start_value do
      nil ->
        %{state | output: clamp(config.bias + p, config), sample: sample, pending: nil}

      value ->
        anchor(state, sample, p, clamp(value, config))
    end
  end

  defp control(%{pending: target, config: config} = state, {_, _, error} = sample, _, _)
       when is_float(target) do
    anchor(state, sample, proportional(config, error), target)
  end

  defp control(%{config: %{deadzone: radius}} = state, {_, _, error} = sample, _, _)
       when radius > 0.0 and abs(error) <= radius do
    %{state | sample: sample}
  end

  defp control(%{config: config} = state, {_, _, error} = sample, elapsed, sign) do
    p = proportional(config, error)
    d = derivative(config, sample, state.sample, elapsed, sign)
    raw = config.bias + state.tracking + p + state.integral + d
    increment = integral_increment(config, error, elapsed)
    accepted = limit_increment(increment, raw, config)

    %{
      state
      | output: clamp(raw + accepted, config),
        sample: sample,
        integral: state.integral + accepted
    }
  end

  @spec anchor(t(), {float(), float(), float()}, float(), float()) :: t()
  defp anchor(state, sample, p, target) do
    %{
      state
      | output: target,
        pending: nil,
        sample: sample,
        integral: 0.0,
        tracking: target - state.config.bias - p
    }
  end

  @spec direction_sign(:direct | :reverse) :: float()
  defp direction_sign(direction)
  defp direction_sign(:direct), do: 1.0
  defp direction_sign(:reverse), do: -1.0

  @spec proportional(configuration(), float()) :: float()
  defp proportional(config, error)
  defp proportional(%{p_enabled: true, kp: kp}, error), do: kp * error
  defp proportional(%{p_enabled: false}, _), do: 0.0

  @spec integral_increment(configuration(), float(), float()) :: float()
  defp integral_increment(config, error, elapsed)

  defp integral_increment(%{tn: tn, kp: kp}, error, elapsed)
       when is_float(tn) and kp > 0.0 and error != 0.0,
       do: kp * (elapsed / tn) * error

  defp integral_increment(_, _, _), do: 0.0

  @spec derivative(
          configuration(),
          {float(), float(), float()},
          {float(), float(), float() | nil} | nil,
          float(),
          float()
        ) :: float()
  defp derivative(config, sample, previous, elapsed, sign)

  defp derivative(%{td: nil}, _, _, _, _), do: 0.0
  defp derivative(%{kp: kp}, _, _, _, _) when kp == 0.0, do: 0.0

  defp derivative(
         config,
         {_, measurement, error},
         {_, previous_measurement, previous_error},
         elapsed,
         sign
       ) do
    change =
      case config.derivative_on do
        :measurement -> -sign * (measurement - previous_measurement)
        :error -> error - previous_error
      end

    if change == 0.0, do: 0.0, else: config.kp * config.td * (change / elapsed)
  end

  @spec limit_increment(float(), float(), configuration()) :: float()
  defp limit_increment(increment, raw, config)

  defp limit_increment(increment, raw, %{out_max: high}) when increment > 0.0 and raw < high,
    do: min(increment, high - raw)

  defp limit_increment(increment, raw, %{out_min: low}) when increment < 0.0 and raw > low,
    do: max(increment, low - raw)

  defp limit_increment(_, _, _), do: 0.0

  @spec clamp(float(), configuration()) :: float()
  defp clamp(value, config), do: min(max(value, config.out_min), config.out_max)
end
