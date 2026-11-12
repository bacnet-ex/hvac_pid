defmodule HvacPid.Autotuner do
  @moduledoc """
  Runs a bounded plant experiment and calculates PID settings for review.

  Choose the experiment with `:method` in `start/2`. The default `:step`
  method changes the output once and observes how the plant settles. The
  `:relay` method switches between two output levels and measures the
  resulting oscillation.

  Once the experiment returns a model, choose the controller form and
  tuning profile with `gains/3`. The form selects which PID terms are
  enabled. The profile selects the formula for their gains. Both choices
  happen after identification, so you can compare several sets of gains
  from a single experiment.

  ## Running an experiment

  The application supplies measurements and elapsed time and writes the
  returned commands. Keep the controller state for later and route samples
  to the tuner while the experiment is running. Send each returned command
  through the usual actuator writer before collecting the next sample. The
  measured interval should reflect the time the previous command was in effect.
  For a step test, start near the intended operating point and choose a
  baseline settling duration that accounts for the plant's response lag.

  On completion or failure, apply the returned exit command and discard
  the tuner state. Use `cancel/1` if an actuator write fails or an override
  changes the applied command. Use an application watchdog to detect
  missing samples. The tuner can check limits and timeouts when a sample
  arrives. The plant can continue moving after an exit command.

  ## Complete controller and autotuner example

  This example runs entirely in memory. A heating loop starts in automatic
  mode at 20 °C with a 30% valve command. The simulated room has a 12-second
  time constant and a three-second delay. The `tick` function represents
  applying a valve command, waiting one second, and reading the next
  measurement. The command history accounts for the delay.

  The controller produces the initial valve command. The tuner takes over
  for the step test, then returns the valve to its baseline while the
  result is reviewed. The example accepts a PI setpoint profile, applies
  its gains, and verifies that the first automatic command matches the
  output in effect at handoff. The following sample shows the tuned
  controller beginning to reduce the valve command.

      iex> alias HvacPid.{Autotuner, Controller}
      iex> setpoint = 20.0
      iex> plant = {20.0, [30.0, 30.0, 30.0]}
      iex> tick = fn command, {measurement, [delayed | history]} ->
      ...>   equilibrium = 20.0 + 0.4 * (delayed - 30.0)
      ...>   measurement = equilibrium + (measurement - equilibrium) * :math.exp(-1.0 / 12.0)
      ...>   {measurement, history ++ [command]}
      ...> end
      iex> {:ok, pid} = Controller.new(out_min: 0, out_max: 100, bias: 30)
      iex> {:ok, pid} = Controller.set_mode(pid, :automatic)
      iex> {measurement, _history} = plant
      iex> {:ok, baseline, pid} = Controller.step(pid, setpoint, measurement, 1.0)
      iex> opts = [
      ...>   out_min: 0, out_max: 100, baseline_output: baseline,
      ...>   step_output: 50, exit_output: baseline,
      ...>   measurement_min: 15, measurement_max: 35, min_response: 1,
      ...>   settle_tolerance: 0.05, settle_seconds: 8,
      ...>   timeout_seconds: 180, max_sample_interval_seconds: 2
      ...> ]
      iex> {:ok, command, tuner} = Autotuner.start(opts, measurement)
      iex> result =
      ...>   Enum.reduce_while(1..180, {tuner, command, plant}, fn _second, {tuner, command, plant} ->
      ...>     {measurement, _history} = plant = tick.(command, plant)
      ...>     case Autotuner.step(tuner, measurement, 1.0) do
      ...>       {:running, next_command, next_tuner} ->
      ...>         {:cont, {next_tuner, next_command, plant}}
      ...>       {:ok, exit_command, model} ->
      ...>         {:halt, {{:ok, model}, exit_command, plant}}
      ...>       {:error, reason, exit_command} ->
      ...>         {:halt, {{:error, reason}, exit_command, plant}}
      ...>     end
      ...>   end)
      iex> {{:ok, model}, exit_command, plant} = result
      iex> {measurement, _history} = plant = tick.(exit_command, plant)
      iex> {:ok, gains} = Autotuner.gains(model, :pi, :chr_aperiodic_setpoint)
      iex> {Float.round(gains[:kp], 2), Float.round(gains[:tn], 2)}
      {3.45, 14.19}
      iex> {:ok, pid} = Controller.configure(pid, gains)
      iex> {:ok, pid} = Controller.set_mode(pid, :automatic)
      iex> {:ok, pid} = Controller.track_output(pid, exit_command)
      iex> {:ok, command, pid} = Controller.step(pid, setpoint, measurement, 1.0)
      iex> command
      30.0
      iex> {measurement, _history} = tick.(command, plant)
      iex> {:ok, command, _pid} = Controller.step(pid, setpoint, measurement, 1.0)
      iex> Float.round(command, 2)
      28.05

  On a real actuator, apply the exit command on either terminal result
  and read back the effective output for `Controller.track_output/2`.
  Take a fresh measurement when automatic control resumes. Use the same
  controller direction for the experiment and subsequent control. If the
  gains are declined or the run fails, the saved controller settings are
  available for the same handoff when the application requests it.

  ## Step-response experiment

  Use `:step` for a self-regulating process that approaches a steady value
  at each fixed output. For example, a room may settle at one temperature
  with a valve at 30% and a different temperature with the valve at 40%.
  The fitted model describes the plant around those operating conditions.

  1. The tuner holds `:baseline_output` and watches the measurement. A
     settled window contains at least three samples, spans at least
     `:settle_seconds`, and has a maximum-to-minimum spread of at most
     `:settle_tolerance`. If the spread grows too large, the window starts
     again at the latest sample. The average of the settled window is the
     baseline measurement.
  2. The tuner issues `:step_output` and starts recording the response.
     The endpoint window starts when the measurement moves by at least
     `:min_response` from the baseline. A sample that returns closer than
     that discards the endpoint window. The same settling rule applies
     at the endpoint. The tuner divides the change between the two
     averaged measurements by the change in output to get the signed
     process gain, `K`. A positive gain corresponds to direct action and
     a negative gain to reverse action. A sign that disagrees with
     `:direction` ends the run with `:direction_mismatch`.
  3. The tuner expresses each measurement as a fraction of the final
     change. This makes a falling response advance from zero toward one
     just like a rising response. It linearly interpolates the first
     crossings of 28.3% and 63.2% at times `t₁` and `t₂` after the output
     step. A first-order response reaches those fractions one third of
     a time constant and one full time constant after the dead time.
     The estimates are `T = 1.5 × (t₂ − t₁)` for the time constant and
     `L = t₂ − T` for the dead time, both in seconds.
  4. The delay must span at least the wider of the two sample intervals
     used for interpolation. The tuner waits until at least `L + 4T`
     has elapsed after the step and checks every recorded measurement
     against the fitted response. The maximum allowed error is 10% of
     the final measurement change. A run can end with
     `:unresolved_dynamics` when the crossings or delay cannot be
     resolved, `:poor_fit` when the model misses the trace, or `:timeout`
     when the plant takes too long to settle or move far enough.

  ## Relay experiment

  Use `:relay` when the two configured output levels can repeatedly move
  the measurement across both sides of a target. With a target of 21 and
  a `:hysteresis` half-width of 0.5, the thresholds are 20.5 and 21.5.
  In direct action, the lower threshold selects `:high_output` and the
  upper threshold selects `:low_output`. In reverse action, the upper
  threshold selects the high output. Between thresholds the relay holds
  its current command. At startup it chooses the command that drives the
  measurement toward the target, using the high output at equality.

  The tuner records peaks and valleys in the timestamped measurements.
  For reverse action it flips the sign of the measurement relative to
  the target, so both directions use the same valley-to-valley period
  calculation. For each cycle, the amplitude is half the difference
  between the intervening peak and the mean of the two valleys. The
  first complete cycle is discarded. The tuner then
  collects ten cycles, requiring at least three samples in each
  half-cycle. Every amplitude and period in the last five cycles must
  be within 20% of its respective median. A plant that fails to cross
  both thresholds eventually times out.

  The median period gives the estimated ultimate period `Tu`, in seconds.
  If `d` is half the difference between the high and low output commands
  and `a` is the median measurement amplitude, the estimated ultimate
  gain is `Ku ≈ 4d / (πa)`. For example, outputs of 30% and 50% give
  `d = 10` percentage points. The estimates characterize the observed
  oscillation. See [Åström and Hägglund's relay method](https://lucris.lub.lu.se/ws/files/6340936/8509157.pdf).

  ## Choosing a gain profile

  The step-response profiles use the fitted `K`, `T`, and `L` to calculate
  a common gain scale `G = T / (abs(K) × L)`. Each profile sets `Kp` as a
  multiple of `G` and uses either the dead time or time constant for
  `Tn` and `Td`. The controller's direction accounts for the sign of
  the process gain. Within a given controller form, a larger `Kp` gives
  a stronger proportional response. Integral strength scales with
  `Kp / Tn` and derivative strength with `Kp × Td`.

  A **setpoint** profile targets a change in the requested value, such
  as raising a temperature setpoint. A **disturbance** profile targets
  recovery at a fixed setpoint after a load change, such as opening a
  door. The Chien–Hrones–Reswick (CHR) aperiodic profiles aim to avoid
  overshoot. The 20% profiles allow overshoot in their design target
  and use larger proportional multipliers than their aperiodic
  counterparts. Actual overshoot depends on the plant and the loop.

  The entries below give `(Kp, Tn, Td)`, with omitted times set to
  `nil`. The formulas follow the WAGO *CODESYS Modules for Control
  Engineering Applications* note, pages 9–10.

  | Profile | Tuning objective | P | PI | PID |
  | --- | --- | --- | --- | --- |
  | `:ziegler_nichols` | Extended Ziegler–Nichols step-response rule. It gives the largest proportional multiplier for each form, tied with 20% disturbance for PID. | `(G, nil, nil)` | `(0.9G, 3.33L, nil)` | `(1.2G, 2L, 0.5L)` |
  | `:chr_aperiodic_disturbance` | Recover from a load change at a fixed setpoint with an aperiodic response. | `(0.3G, nil, nil)` | `(0.6G, 4L, nil)` | `(0.95G, 2.4L, 0.42L)` |
  | `:chr_aperiodic_setpoint` | Track a new setpoint with an aperiodic response. | `(0.3G, nil, nil)` | `(0.35G, 1.2T, nil)` | `(0.6G, T, 0.5L)` |
  | `:chr_20_disturbance` | Recover from a load change at a fixed setpoint with a 20% overshoot design target. | `(0.7G, nil, nil)` | `(0.7G, 2.3L, nil)` | `(1.2G, 2L, 0.42L)` |
  | `:chr_20_setpoint` | Track a new setpoint with a 20% overshoot design target. | `(0.7G, nil, nil)` | `(0.6G, T, nil)` | `(0.95G, 1.35T, 0.47L)` |

  For the same step model with `K = 0.5`, `T = 20 s`, and `L = 5 s`,
  the aperiodic setpoint PI rule gives `Kp = 2.8` and `Tn = 24 s`. The
  aperiodic disturbance PI rule gives `Kp = 4.8` and `Tn = 20 s`.
  That second set has both stronger proportional action and stronger
  integral action for this plant. The two CHR objectives use identical
  gains within each overshoot category for the P-only form.

  The relay result supports `:relay_pid` with the `:pid` form. Its
  rule uses the oscillation estimates directly, with `Kp = 0.25 × Ku`,
  `Tn = Tu`, and `Td = Tu / 8`. For example, `Ku = 8` and `Tu = 24 s`
  give `Kp = 2`, `Tn = 24 s`, and `Td = 3 s`.
  """

  alias HvacPid.Controller

  @typedoc "Method for exciting the plant and identifying its response."
  @type method :: :step | :relay

  @typedoc "Controller form for the generated settings."
  @type form :: :p | :pi | :pid

  @typedoc "Tuning rule applied to the identified model."
  @type profile ::
          :ziegler_nichols
          | :chr_aperiodic_disturbance
          | :chr_aperiodic_setpoint
          | :chr_20_disturbance
          | :chr_20_setpoint
          | :relay_pid

  @typedoc "Measured plant characteristics returned by a successful experiment."
  @type model ::
          %{
            process_gain: float(),
            time_constant_seconds: float(),
            dead_time_seconds: float()
          }
          | %{
              method: :relay,
              ultimate_gain: float(),
              ultimate_period_seconds: float()
            }

  @typedoc "Experiment configuration. See `start/2` for required options and units."
  @type option ::
          {:method, method()}
          | {:direction, :direct | :reverse}
          | {:max_samples, pos_integer()}
          | {:out_min
             | :out_max
             | :exit_output
             | :measurement_min
             | :measurement_max
             | :timeout_seconds
             | :max_sample_interval_seconds
             | :baseline_output
             | :step_output
             | :min_response
             | :settle_tolerance
             | :settle_seconds
             | :low_output
             | :high_output
             | :target
             | :hysteresis, number()}

  @typedoc "A rejected configuration, sample, experiment, or gain calculation."
  @type error ::
          :invalid_options
          | {:unknown_option, atom()}
          | {:duplicate_option, atom()}
          | {:missing_option, atom()}
          | {:invalid_option, atom()}
          | :invalid_output_limits
          | :invalid_measurement_limits
          | {:invalid_input, :measurement | :elapsed_seconds}
          | :measurement_limit
          | :sample_gap
          | :timeout
          | :sample_limit
          | :direction_mismatch
          | :unresolved_dynamics
          | :poor_fit
          | :undersampled_oscillation
          | :irregular_oscillation
          | :invalid_model
          | :invalid_form
          | :invalid_profile
          | :incompatible_profile
          | :invalid_gains
          | :numeric_overflow

  @typep sample :: {float(), float()}
  @typep window :: %{
           since: float(),
           min: float(),
           max: float(),
           mean: float(),
           count: pos_integer()
         }

  @typep experiment ::
           %{phase: :baseline, window: window()}
           | %{
               phase: :response,
               baseline: float(),
               started_at: float(),
               samples: [sample()],
               window: window() | nil
             }
           | %{
               phase: :relay,
               high?: boolean(),
               extreme: sample(),
               half_samples: non_neg_integer(),
               valley: sample() | nil,
               peak: {sample(), pos_integer()} | nil,
               warmed_up?: boolean(),
               cycles: [{float(), float()}]
             }

  @typedoc "State of a running experiment. Carry it forward only on a running result."
  @opaque t :: %__MODULE__{
            config: map(),
            elapsed: float(),
            sample_count: pos_integer(),
            experiment: experiment()
          }

  @typep progress ::
           {:running, float(), t()} | {:ok, float(), model()} | {:error, error(), float()}

  @fields [:config, :elapsed, :sample_count, :experiment]
  @enforce_keys @fields
  defstruct @fields

  @common_required [
    :out_min,
    :out_max,
    :exit_output,
    :measurement_min,
    :measurement_max,
    :timeout_seconds,
    :max_sample_interval_seconds
  ]
  @step_required [
    :baseline_output,
    :step_output,
    :min_response,
    :settle_tolerance,
    :settle_seconds
  ]
  @relay_required [:low_output, :high_output, :target, :hysteresis]
  @defaults %{method: :step, direction: :direct, max_samples: 10_000}
  @positive_options [
    :timeout_seconds,
    :max_sample_interval_seconds,
    :min_response,
    :settle_tolerance,
    :settle_seconds,
    :hysteresis
  ]

  # Each entry gives the P gain multiplier, then the PI and PID gain
  # multipliers and time factors. Time factors use the fitted time constant
  # or dead time as indicated.
  @step_profiles %{
    ziegler_nichols: {1.0, {0.9, {3.33, :delay}}, {1.2, {2.0, :delay}, {0.5, :delay}}},
    chr_aperiodic_disturbance: {
      0.3,
      {0.6, {4.0, :delay}},
      {0.95, {2.4, :delay}, {0.42, :delay}}
    },
    chr_aperiodic_setpoint: {
      0.3,
      {0.35, {1.2, :time_constant}},
      {0.6, {1.0, :time_constant}, {0.5, :delay}}
    },
    chr_20_disturbance: {0.7, {0.7, {2.3, :delay}}, {1.2, {2.0, :delay}, {0.42, :delay}}},
    chr_20_setpoint: {
      0.7,
      {0.6, {1.0, :time_constant}},
      {0.95, {1.35, :time_constant}, {0.47, :delay}}
    }
  }

  @early_fraction 1 - :math.exp(-1 / 3)
  @late_fraction 1 - :math.exp(-1)
  @relay_cycles 10
  @relay_recent_cycles 5

  @doc """
  Starts a tuning experiment and returns its initial actuator command.

  Both methods require `:out_min`, `:out_max`, and `:exit_output` in actuator
  units, `:measurement_min` and `:measurement_max` in measurement units, and
  positive `:timeout_seconds` and `:max_sample_interval_seconds`.
  Measurements must be strictly inside their limits. Commands can use
  either output bound.

  The default `method: :step` also requires distinct `:baseline_output` and
  `:step_output`, positive `:settle_seconds`, and positive
  `:settle_tolerance` and `:min_response` in measurement units. The
  tolerance must be smaller than the minimum response.

  For `method: :relay`, supply `:low_output` below `:high_output`, a
  measurement `:target`, and a positive `:hysteresis` half-width. Both
  switching thresholds must lie strictly inside the measurement limits.

  The default direction is `:direct`. The default sample budget is
  `10_000` and includes the initial measurement.
  """
  @spec start([option()], number()) :: {:ok, float(), t()} | {:error, error()}
  def start(opts, initial_measurement) do
    with {:ok, config} <- configuration(opts),
         {:ok, measurement} <- input_number(:measurement, initial_measurement),
         :ok <- check_measurement(config, measurement) do
      {command, experiment} = initialize(config, measurement)

      {:ok, command,
       %__MODULE__{config: config, elapsed: 0.0, sample_count: 1, experiment: experiment}}
    end
  rescue
    ArithmeticError -> {:error, :numeric_overflow}
  end

  @doc """
  Processes a measurement and the positive elapsed time since the previous sample.

  Apply the command in each `:running` result and retain its tuner state.
  Terminal results include the configured exit command. A measurement on
  either limit aborts the experiment. Timeout is measured from `start/2`.
  """
  @spec step(t(), number(), number()) :: progress()
  def step(%__MODULE__{config: config} = tuner, measurement, elapsed_seconds) do
    with {:ok, measurement} <- input_number(:measurement, measurement),
         {:ok, interval} <- input_number(:elapsed_seconds, elapsed_seconds),
         :ok <- check_measurement(config, measurement),
         :ok <- check_interval(config, interval),
         elapsed = tuner.elapsed + interval,
         :ok <- check_progress(tuner, elapsed) do
      tuner = %{tuner | elapsed: elapsed, sample_count: tuner.sample_count + 1}
      advance(tuner, measurement)
    else
      {:error, reason} -> {:error, reason, config.exit_output}
    end
  rescue
    ArithmeticError -> {:error, :numeric_overflow, config.exit_output}
  end

  @doc """
  Returns the exit command so the caller can apply it and discard the experiment.
  """
  @spec cancel(t()) :: {:ok, float()}
  def cancel(%__MODULE__{config: config}), do: {:ok, config.exit_output}

  @doc """
  Calculates controller options from an identified model.

  Step-response profiles support `:p`, `:pi`, and `:pid`. The relay
  profile `:relay_pid` supports `:pid`. See
  [Choosing a gain profile](#module-choosing-a-gain-profile) for the
  formulas and their design objectives. Every form enables P. Use the
  returned `:tn` and `:td` values as-is to select I and D, including
  the `nil` values for disabled terms.

      iex> alias HvacPid.Autotuner
      iex> model = %{process_gain: 0.5, time_constant_seconds: 20.0, dead_time_seconds: 5.0}
      iex> Autotuner.gains(model, :pi, :chr_aperiodic_setpoint)
      {:ok, [kp: 2.8, p_enabled: true, tn: 24.0, td: nil]}
      iex> Autotuner.gains(model, :pi, :chr_aperiodic_disturbance)
      {:ok, [kp: 4.8, p_enabled: true, tn: 20.0, td: nil]}

      iex> model = %{method: :relay, ultimate_gain: 8.0, ultimate_period_seconds: 24.0}
      iex> HvacPid.Autotuner.gains(model, :pid, :relay_pid)
      {:ok, [kp: 2.0, p_enabled: true, tn: 24.0, td: 3.0]}
  """
  @spec gains(model(), form(), profile()) :: {:ok, [Controller.option()]} | {:error, error()}
  def gains(model, form, profile)

  def gains(_, form, _) when form not in [:p, :pi, :pid], do: {:error, :invalid_form}

  def gains(_, _, profile) when profile != :relay_pid and not is_map_key(@step_profiles, profile),
    do: {:error, :invalid_profile}

  def gains(model, form, profile) do
    calculate_gains(model, form, profile)
  rescue
    ArithmeticError -> {:error, :numeric_overflow}
  end

  @spec configuration(term()) :: {:ok, map()} | {:error, error()}
  defp configuration(opts) do
    if Keyword.keyword?(opts) do
      method = Keyword.get(opts, :method, :step)

      with {:ok, required} <- required_options(method),
           {:ok, config} <- parse_options(opts, required),
           :ok <- validate_configuration(config) do
        {:ok, config}
      end
    else
      {:error, :invalid_options}
    end
  end

  @spec required_options(term()) :: {:ok, [atom()]} | {:error, error()}
  defp required_options(method)
  defp required_options(:step), do: {:ok, @common_required ++ @step_required}
  defp required_options(:relay), do: {:ok, @common_required ++ @relay_required}
  defp required_options(_), do: {:error, {:invalid_option, :method}}

  @spec parse_options(keyword(), [atom()]) :: {:ok, map()} | {:error, error()}
  defp parse_options(opts, required) do
    parsed =
      Enum.reduce_while(opts, {:ok, %{}}, fn {key, value}, {:ok, config} ->
        cond do
          Map.has_key?(config, key) ->
            {:halt, {:error, {:duplicate_option, key}}}

          key in required or is_map_key(@defaults, key) ->
            case option_value(key, value) do
              {:ok, normalized} -> {:cont, {:ok, Map.put(config, key, normalized)}}
              :error -> {:halt, {:error, {:invalid_option, key}}}
            end

          true ->
            {:halt, {:error, {:unknown_option, key}}}
        end
      end)

    with {:ok, config} <- parsed do
      case Enum.find(required, &(not Map.has_key?(config, &1))) do
        nil -> {:ok, Map.merge(@defaults, config)}
        key -> {:error, {:missing_option, key}}
      end
    end
  end

  @spec option_value(atom(), term()) :: {:ok, float() | pos_integer() | atom()} | :error
  defp option_value(key, value)
  defp option_value(:direction, value) when value in [:direct, :reverse], do: {:ok, value}
  defp option_value(:direction, _), do: :error
  defp option_value(:method, value) when value in [:step, :relay], do: {:ok, value}
  defp option_value(:max_samples, value) when is_integer(value) and value > 0, do: {:ok, value}
  defp option_value(:max_samples, _), do: :error

  defp option_value(key, value) do
    case normalize_number(value) do
      {:ok, number} when key not in @positive_options or number > 0.0 -> {:ok, number}
      _ -> :error
    end
  end

  @spec input_number(:measurement | :elapsed_seconds, term()) ::
          {:ok, float()} | {:error, error()}
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

  @spec validate_configuration(map()) :: :ok | {:error, error()}
  defp validate_configuration(config) do
    cond do
      config.out_min >= config.out_max ->
        {:error, :invalid_output_limits}

      config.measurement_min >= config.measurement_max ->
        {:error, :invalid_measurement_limits}

      not within_output_bounds?(config, config.exit_output) ->
        {:error, {:invalid_option, :exit_output}}

      true ->
        validate_method(config)
    end
  end

  @spec validate_method(map()) :: :ok | {:error, error()}
  defp validate_method(config)

  defp validate_method(%{method: :step} = config) do
    cond do
      not within_output_bounds?(config, config.baseline_output) ->
        {:error, {:invalid_option, :baseline_output}}

      not within_output_bounds?(config, config.step_output) or
          config.step_output == config.baseline_output ->
        {:error, {:invalid_option, :step_output}}

      config.settle_tolerance >= config.min_response ->
        {:error, {:invalid_option, :settle_tolerance}}

      true ->
        :ok
    end
  end

  defp validate_method(%{method: :relay} = config) do
    lower = config.target - config.hysteresis
    upper = config.target + config.hysteresis

    cond do
      not within_output_bounds?(config, config.low_output) ->
        {:error, {:invalid_option, :low_output}}

      not within_output_bounds?(config, config.high_output) or
          config.low_output >= config.high_output ->
        {:error, {:invalid_option, :high_output}}

      lower <= config.measurement_min or upper >= config.measurement_max or
        lower >= config.target or upper <= config.target ->
        {:error, {:invalid_option, :hysteresis}}

      true ->
        :ok
    end
  end

  @spec within_output_bounds?(map(), float()) :: boolean()
  defp within_output_bounds?(config, output),
    do: output >= config.out_min and output <= config.out_max

  @spec check_measurement(map(), float()) :: :ok | {:error, :measurement_limit}
  defp check_measurement(config, measurement) do
    if measurement > config.measurement_min and measurement < config.measurement_max do
      :ok
    else
      {:error, :measurement_limit}
    end
  end

  @spec check_interval(map(), float()) :: :ok | {:error, :sample_gap}
  defp check_interval(config, interval) do
    if interval <= config.max_sample_interval_seconds, do: :ok, else: {:error, :sample_gap}
  end

  @spec check_progress(t(), float()) :: :ok | {:error, error()}
  defp check_progress(tuner, elapsed) do
    cond do
      elapsed <= tuner.elapsed -> {:error, :numeric_overflow}
      elapsed >= tuner.config.timeout_seconds -> {:error, :timeout}
      tuner.sample_count >= tuner.config.max_samples -> {:error, :sample_limit}
      true -> :ok
    end
  end

  @spec initialize(map(), float()) :: {float(), experiment()}
  defp initialize(config, measurement)

  defp initialize(%{method: :step} = config, measurement) do
    {config.baseline_output, %{phase: :baseline, window: new_window(0.0, measurement)}}
  end

  defp initialize(%{method: :relay} = config, measurement) do
    deviation = relay_deviation(config, measurement)
    high? = deviation <= 0.0

    {relay_output(config, high?),
     %{
       phase: :relay,
       high?: high?,
       extreme: {0.0, deviation},
       half_samples: 0,
       valley: nil,
       peak: nil,
       warmed_up?: false,
       cycles: []
     }}
  end

  @spec advance(t(), float()) :: progress()
  defp advance(tuner, measurement)

  defp advance(%{experiment: %{phase: :baseline} = experiment} = tuner, measurement) do
    config = tuner.config
    window = update_window(experiment.window, tuner.elapsed, measurement, config.settle_tolerance)

    if settled?(window, tuner.elapsed, config.settle_seconds) do
      response = %{
        phase: :response,
        baseline: window.mean,
        started_at: tuner.elapsed,
        samples: [{0.0, measurement}],
        window: nil
      }

      {:running, config.step_output, %{tuner | experiment: response}}
    else
      {:running, config.baseline_output, %{tuner | experiment: %{experiment | window: window}}}
    end
  end

  defp advance(%{experiment: %{phase: :response} = experiment} = tuner, measurement) do
    config = tuner.config
    time = tuner.elapsed - experiment.started_at
    samples = [{time, measurement} | experiment.samples]

    window =
      if abs(measurement - experiment.baseline) >= config.min_response do
        update_window(experiment.window, time, measurement, config.settle_tolerance)
      end

    tuner = %{tuner | experiment: %{experiment | samples: samples, window: window}}

    if window && settled?(window, time, config.settle_seconds) do
      finish_response(tuner, time, window.mean)
    else
      {:running, config.step_output, tuner}
    end
  end

  defp advance(%{experiment: %{phase: :relay} = experiment} = tuner, measurement) do
    deviation = relay_deviation(tuner.config, measurement)
    sample = {tuner.elapsed, deviation}
    extreme = relay_extreme(experiment.extreme, sample, experiment.high?)
    experiment = %{experiment | extreme: extreme, half_samples: experiment.half_samples + 1}
    tuner = %{tuner | experiment: experiment}
    hysteresis = tuner.config.hysteresis

    cond do
      experiment.high? and deviation >= hysteresis ->
        finish_relay_half_cycle(tuner, sample)

      not experiment.high? and deviation <= -hysteresis ->
        peak = {extreme, experiment.half_samples}
        switch_relay(tuner, %{experiment | peak: peak}, sample)

      true ->
        {:running, relay_output(tuner.config, experiment.high?), tuner}
    end
  end

  @spec new_window(float(), float()) :: window()
  defp new_window(time, value) do
    %{since: time, min: value, max: value, mean: value, count: 1}
  end

  @spec update_window(window() | nil, float(), float(), float()) :: window()
  defp update_window(window, time, value, tolerance)
  defp update_window(nil, time, value, _), do: new_window(time, value)

  defp update_window(window, time, value, tolerance) do
    low = min(window.min, value)
    high = max(window.max, value)

    if high - low > tolerance do
      new_window(time, value)
    else
      count = window.count + 1

      %{
        window
        | min: low,
          max: high,
          count: count,
          mean: window.mean + (value - window.mean) / count
      }
    end
  end

  @spec settled?(window(), float(), float()) :: boolean()
  defp settled?(window, time, duration), do: window.count >= 3 and time - window.since >= duration

  @spec finish_response(t(), float(), float()) :: progress()
  defp finish_response(%{experiment: experiment, config: config} = tuner, time, endpoint) do
    response = endpoint - experiment.baseline
    output_step = config.step_output - config.baseline_output
    gain = response / output_step

    with :ok <- check_direction(config.direction, gain),
         samples = Enum.reverse(experiment.samples),
         {:ok, tau, delay} <- estimate_response(samples, experiment.baseline, response) do
      cond do
        time < delay + 4.0 * tau ->
          {:running, config.step_output, tuner}

        response_fits?(samples, experiment.baseline, response, tau, delay) ->
          model = %{process_gain: gain, time_constant_seconds: tau, dead_time_seconds: delay}
          {:ok, config.exit_output, model}

        true ->
          {:error, :poor_fit, config.exit_output}
      end
    else
      {:error, reason} -> {:error, reason, config.exit_output}
    end
  end

  @spec check_direction(:direct | :reverse, float()) :: :ok | {:error, error()}
  defp check_direction(direction, gain)
  defp check_direction(:direct, gain) when gain > 0.0, do: :ok
  defp check_direction(:reverse, gain) when gain < 0.0, do: :ok
  defp check_direction(_, gain) when gain == 0.0, do: {:error, :unresolved_dynamics}
  defp check_direction(_, _), do: {:error, :direction_mismatch}

  @spec estimate_response([sample()], float(), float()) ::
          {:ok, float(), float()} | {:error, error()}
  defp estimate_response(samples, baseline, response) do
    normalized = Enum.map(samples, fn {time, value} -> {time, (value - baseline) / response} end)

    with {early, early_interval} <- crossing(normalized, @early_fraction),
         {late, late_interval} <- crossing(normalized, @late_fraction) do
      tau = 1.5 * (late - early)
      delay = late - tau

      if tau > 0.0 and delay >= max(early_interval, late_interval) do
        {:ok, tau, delay}
      else
        {:error, :unresolved_dynamics}
      end
    else
      nil -> {:error, :unresolved_dynamics}
    end
  end

  @spec crossing([sample()], float()) :: sample() | nil
  defp crossing(samples, fraction)

  defp crossing([{before_time, before}, {after_time, after_value} | _], fraction)
       when before < fraction and after_value >= fraction do
    interval = after_time - before_time
    time = before_time + interval * ((fraction - before) / (after_value - before))
    {time, interval}
  end

  defp crossing([_, next | rest], fraction), do: crossing([next | rest], fraction)
  defp crossing(_, _), do: nil

  @spec response_fits?([sample()], float(), float(), float(), float()) :: boolean()
  defp response_fits?(samples, baseline, response, tau, delay) do
    Enum.all?(samples, fn {time, measurement} ->
      observed = (measurement - baseline) / response
      predicted = 1.0 - :math.exp(-max(0.0, time - delay) / tau)
      abs(observed - predicted) <= 0.1
    end)
  end

  @spec relay_deviation(map(), float()) :: float()
  defp relay_deviation(config, measurement) do
    case config.direction do
      :direct -> measurement - config.target
      :reverse -> config.target - measurement
    end
  end

  @spec relay_output(map(), boolean()) :: float()
  defp relay_output(config, high?)
  defp relay_output(config, true), do: config.high_output
  defp relay_output(config, false), do: config.low_output

  @spec relay_extreme(sample(), sample(), boolean()) :: sample()
  defp relay_extreme(previous, current, high?)

  defp relay_extreme({_, previous}, {_, current} = sample, true) when current < previous,
    do: sample

  defp relay_extreme({_, previous}, {_, current} = sample, false) when current > previous,
    do: sample

  defp relay_extreme(previous, _, _), do: previous

  @spec switch_relay(t(), map(), sample()) :: progress()
  defp switch_relay(tuner, experiment, sample) do
    high? = not experiment.high?
    experiment = %{experiment | high?: high?, extreme: sample, half_samples: 0}
    {:running, relay_output(tuner.config, high?), %{tuner | experiment: experiment}}
  end

  @spec finish_relay_half_cycle(t(), sample()) :: progress()
  defp finish_relay_half_cycle(%{experiment: experiment} = tuner, sample) do
    valley = experiment.extreme

    case record_relay_cycle(experiment, valley) do
      {:ok, cycles, warmed_up?} when length(cycles) < @relay_cycles ->
        experiment = %{
          experiment
          | valley: valley,
            peak: nil,
            cycles: cycles,
            warmed_up?: warmed_up?
        }

        switch_relay(tuner, experiment, sample)

      {:ok, cycles, _} ->
        finish_relay(tuner.config, cycles)

      {:error, reason} ->
        {:error, reason, tuner.config.exit_output}
    end
  end

  @spec record_relay_cycle(map(), sample()) ::
          {:ok, [{float(), float()}], boolean()} | {:error, error()}
  defp record_relay_cycle(experiment, valley)

  defp record_relay_cycle(%{valley: nil, cycles: cycles}, _), do: {:ok, cycles, false}

  defp record_relay_cycle(%{warmed_up?: false, cycles: cycles}, _), do: {:ok, cycles, true}

  defp record_relay_cycle(
         %{
           valley: {previous_time, previous_valley},
           peak: {{_, peak}, peak_samples},
           half_samples: valley_samples,
           cycles: cycles
         },
         {time, valley}
       )
       when peak_samples >= 3 and valley_samples >= 3 do
    amplitude = ((peak - previous_valley) / 2.0 + (peak - valley) / 2.0) / 2.0
    period = time - previous_time

    if amplitude > 0.0 and period > 0.0 do
      {:ok, [{amplitude, period} | cycles], true}
    else
      {:error, :irregular_oscillation}
    end
  end

  defp record_relay_cycle(_, _), do: {:error, :undersampled_oscillation}

  @spec finish_relay(map(), [{float(), float()}]) :: progress()
  defp finish_relay(config, cycles) do
    {amplitudes, periods} = cycles |> Enum.take(@relay_recent_cycles) |> Enum.unzip()
    amplitude = median(amplitudes)
    period = median(periods)

    if repeatable?(amplitudes, amplitude) and repeatable?(periods, period) do
      half_span = (config.high_output - config.low_output) / 2.0
      gain = 4.0 / :math.pi() * (half_span / amplitude)

      if gain > 0.0 do
        model = %{method: :relay, ultimate_gain: gain, ultimate_period_seconds: period}
        {:ok, config.exit_output, model}
      else
        {:error, :unresolved_dynamics, config.exit_output}
      end
    else
      {:error, :irregular_oscillation, config.exit_output}
    end
  end

  @spec median([float()]) :: float()
  defp median(values), do: values |> Enum.sort() |> Enum.at(div(@relay_recent_cycles, 2))

  @spec repeatable?([float()], float()) :: boolean()
  defp repeatable?(values, center), do: Enum.all?(values, &(abs(&1 - center) <= 0.2 * center))

  @spec calculate_gains(term(), form(), profile()) ::
          {:ok, [Controller.option()]} | {:error, error()}
  defp calculate_gains(model, form, profile)

  defp calculate_gains(%{method: :relay} = model, :pid, :relay_pid) do
    with %{ultimate_gain: gain, ultimate_period_seconds: period} <- model,
         {:ok, gain} when gain > 0.0 <- normalize_number(gain),
         {:ok, period} when period > 0.0 <- normalize_number(period) do
      gain_options(0.25 * gain, period, period / 8.0)
    else
      _ -> {:error, :invalid_model}
    end
  end

  defp calculate_gains(%{method: :relay}, _, _), do: {:error, :incompatible_profile}

  defp calculate_gains(
         %{process_gain: _, time_constant_seconds: _, dead_time_seconds: _},
         _,
         :relay_pid
       ),
       do: {:error, :incompatible_profile}

  defp calculate_gains(
         %{process_gain: gain, time_constant_seconds: tau, dead_time_seconds: delay},
         form,
         profile
       )
       when is_map_key(@step_profiles, profile) do
    with {:ok, gain} when gain != 0.0 <- normalize_number(gain),
         {:ok, tau} when tau > 0.0 <- normalize_number(tau),
         {:ok, delay} when delay > 0.0 <- normalize_number(delay) do
      proportional_gain = tau / delay / abs(gain)
      time = %{time_constant: tau, delay: delay}
      apply_profile(Map.fetch!(@step_profiles, profile), form, proportional_gain, time)
    else
      _ -> {:error, :invalid_model}
    end
  end

  defp calculate_gains(_, _, _), do: {:error, :invalid_model}

  @typep time_factor :: {float(), :time_constant | :delay}
  @typep step_profile ::
           {float(), {float(), time_factor()}, {float(), time_factor(), time_factor()}}

  @spec apply_profile(step_profile(), form(), float(), map()) ::
          {:ok, [Controller.option()]} | {:error, error()}
  defp apply_profile(profile, form, gain, time)

  defp apply_profile({p, _, _}, :p, gain, _), do: gain_options(p * gain, nil, nil)

  defp apply_profile({_, {p, {i, i_source}}, _}, :pi, gain, time) do
    gain_options(p * gain, i * Map.fetch!(time, i_source), nil)
  end

  defp apply_profile({_, _, {p, {i, i_source}, {d, d_source}}}, :pid, gain, time) do
    gain_options(p * gain, i * Map.fetch!(time, i_source), d * Map.fetch!(time, d_source))
  end

  @spec gain_options(float(), float() | nil, float() | nil) ::
          {:ok, [Controller.option()]} | {:error, error()}
  defp gain_options(kp, tn, td) do
    if kp > 0.0 and (is_nil(tn) or tn > 0.0) and (is_nil(td) or td > 0.0) do
      {:ok, [kp: kp, p_enabled: true, tn: tn, td: td]}
    else
      {:error, :invalid_gains}
    end
  end
end
