defmodule HvacPidAutotunerTest do
  alias HvacPid.Autotuner
  alias HvacPid.Controller

  use ExUnit.Case, async: true

  doctest Autotuner

  @step_options [
    out_min: 0,
    out_max: 100,
    baseline_output: 30,
    step_output: 50,
    exit_output: 25,
    measurement_min: 5,
    measurement_max: 35,
    min_response: 1,
    settle_tolerance: 0.03,
    settle_seconds: 2,
    timeout_seconds: 180,
    max_sample_interval_seconds: 2
  ]
  @relay_options [
    method: :relay,
    out_min: 0,
    out_max: 100,
    low_output: 30,
    high_output: 50,
    exit_output: 25,
    target: 20,
    hysteresis: 0.5,
    measurement_min: 10,
    measurement_max: 30,
    timeout_seconds: 180,
    max_sample_interval_seconds: 2
  ]

  describe "experiment options" do
    test "rejects missing, unknown, duplicate, and invalid options" do
      assert Autotuner.start(%{}, 20) == {:error, :invalid_options}
      assert Autotuner.start([], 20) == {:error, {:missing_option, :out_min}}

      assert Autotuner.start([out_min: 0, out_min: 1], 20) ==
               {:error, {:duplicate_option, :out_min}}

      assert Autotuner.start([low_output: 0], 20) == {:error, {:unknown_option, :low_output}}
      assert Autotuner.start([method: :other], 20) == {:error, {:invalid_option, :method}}

      for {key, value} <- [
            timeout_seconds: 0,
            max_sample_interval_seconds: -1,
            min_response: 0,
            settle_tolerance: 1,
            settle_seconds: nil,
            direction: :other,
            max_samples: 0,
            baseline_output: -1,
            step_output: 30,
            exit_output: 101,
            measurement_max: Integer.pow(10, 400)
          ] do
        assert Autotuner.start(step_opts([{key, value}]), 20) == {:error, {:invalid_option, key}}
      end
    end

    test "checks bounds and the initial sample before issuing a command" do
      assert Autotuner.start(step_opts(out_min: 100), 20) == {:error, :invalid_output_limits}

      assert Autotuner.start(step_opts(measurement_min: 35), 20) ==
               {:error, :invalid_measurement_limits}

      assert Autotuner.start(step_opts(), 35) == {:error, :measurement_limit}
      assert Autotuner.start(step_opts(), :missing) == {:error, {:invalid_input, :measurement}}

      assert {:ok, command, _tuner} =
               Autotuner.start(step_opts(baseline_output: 0, step_output: 100), 20)

      assert command == 0.0
    end

    test "validates relay commands and thresholds" do
      for {key, value} <- [low_output: -1, high_output: 30, hysteresis: 10] do
        assert Autotuner.start(relay_opts([{key, value}]), 20) == {:error, {:invalid_option, key}}
      end

      assert {:error, :numeric_overflow} =
               Autotuner.start(
                 relay_opts(target: 1.0e308, hysteresis: 1.0e308, measurement_max: 1.7e308),
                 20
               )
    end
  end

  describe "command lifecycle" do
    test "requires both the duration and sample count and resets an unsettled window" do
      assert {:ok, 30.0, tuner} = Autotuner.start(step_opts(), 20)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 20, 2)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 21, 1)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 21.01, 1)
      assert {:running, 50.0, tuner} = Autotuner.step(tuner, 21, 1)
      assert {:running, 50.0, _tuner} = Autotuner.step(tuner, 21, 1)
    end

    test "cancels or exits for bad samples, measurement limits, or missing time" do
      assert {:ok, 30.0, baseline} = Autotuner.start(step_opts(), 20)
      response = begin_step(step_opts())

      for tuner <- [baseline, response] do
        assert Autotuner.cancel(tuner) == {:ok, 25.0}
        assert Autotuner.step(tuner, 35, 1) == {:error, :measurement_limit, 25.0}
        assert Autotuner.step(tuner, 5, 1) == {:error, :measurement_limit, 25.0}
        assert Autotuner.step(tuner, :lost, 1) == {:error, {:invalid_input, :measurement}, 25.0}
        assert Autotuner.step(tuner, 20, 0) == {:error, {:invalid_input, :elapsed_seconds}, 25.0}
        assert Autotuner.step(tuner, 20, 2.01) == {:error, :sample_gap, 25.0}
      end
    end

    test "counts the initial sample and both experiment phases against run limits" do
      assert {:ok, 30.0, tuner} = Autotuner.start(step_opts(max_samples: 1), 20)
      assert Autotuner.step(tuner, 20, 1) == {:error, :sample_limit, 25.0}

      tuner = begin_step(step_opts(max_samples: 4, timeout_seconds: 4))
      assert {:running, 50.0, tuner} = Autotuner.step(tuner, 20, 1)
      assert Autotuner.step(tuner, 20, 0.5) == {:error, :sample_limit, 25.0}
      assert Autotuner.step(tuner, 20, 1) == {:error, :timeout, 25.0}
    end

    test "returns the exit command on overflow or when elapsed time loses resolution" do
      opts = step_opts(measurement_min: -1.7e308, measurement_max: 1.7e308)
      assert {:ok, 30.0, tuner} = Autotuner.start(opts, -1.0e308)
      assert Autotuner.step(tuner, 1.0e308, 1) == {:error, :numeric_overflow, 25.0}

      opts = step_opts(timeout_seconds: 1.7e308, max_sample_interval_seconds: 1.7e308)
      assert {:ok, 30.0, tuner} = Autotuner.start(opts, 20)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 20, 1.0e308)
      assert Autotuner.step(tuner, 20, 1) == {:error, :numeric_overflow, 25.0}
    end
  end

  describe "step-response models" do
    test "fits both directions and both signs of actuator step with irregular samples" do
      for {direction, gain} <- [direct: 0.4, reverse: -0.4], command <- [10, 50] do
        opts = step_opts(direction: direction, step_output: command, settle_seconds: 8)
        response = fn t -> first_order_response(t, gain * (command - 30), 3) end
        assert {:ok, 25.0, model} = identify(opts, response)
        assert_in_delta model.process_gain, gain, 0.005
        assert_in_delta model.time_constant_seconds, 12.0, 0.2
        assert_in_delta model.dead_time_seconds, 3.0, 0.1

        assert {:ok, gains} = Autotuner.gains(model, :pi, :chr_aperiodic_setpoint)
        assert_closed_loop(gains, direction, gain)
      end
    end

    test "waits for observation through the model horizon" do
      opts = step_opts(settle_tolerance: 0.99)
      assert {:ok, 25.0, model} = identify(opts, &first_order_response(&1, 8, 3))
      assert model.time_constant_seconds > 9.0
    end

    test "rejects a wrong direction, unresolved delay, and a poor fit" do
      assert identify(step_opts(), &first_order_response(&1, -8, 3)) ==
               {:error, :direction_mismatch, 25.0}

      assert identify(step_opts(), &first_order_response(&1, 8, 0)) ==
               {:error, :unresolved_dynamics, 25.0}

      bumped = fn t ->
        if t < 1, do: 18, else: first_order_response(t, 8, 3)
      end

      assert identify(step_opts(), bumped) == {:error, :poor_fit, 25.0}
    end

    test "times out when the response is too small or never settles" do
      opts = step_opts(timeout_seconds: 12)
      assert identify(opts, fn _time -> 20 end) == {:error, :timeout, 25.0}
      assert identify(opts, &first_order_response(&1, 0.5, 3)) == {:error, :timeout, 25.0}
      assert identify(opts, fn t -> 20 + t / 2 end) == {:error, :timeout, 25.0}
    end
  end

  describe "gain calculations" do
    test "calculates all 15 step-response profiles and returns applicable options" do
      for {profile, results} <- [
            ziegler_nichols: [{2, nil, nil}, {1.8, 16.65, nil}, {2.4, 10, 2.5}],
            chr_aperiodic_disturbance: [{0.6, nil, nil}, {1.2, 20, nil}, {1.9, 12, 2.1}],
            chr_aperiodic_setpoint: [{0.6, nil, nil}, {0.7, 24, nil}, {1.2, 20, 2.5}],
            chr_20_disturbance: [{1.4, nil, nil}, {1.4, 11.5, nil}, {2.4, 10, 2.1}],
            chr_20_setpoint: [{1.4, nil, nil}, {1.2, 20, nil}, {1.9, 27, 2.35}]
          ],
          {form, {kp, tn, td}} <- Enum.zip([:p, :pi, :pid], results) do
        assert {:ok, opts} = Autotuner.gains(step_model(-2, 20, 5), form, profile)
        assert opts[:p_enabled]
        assert_in_delta opts[:kp], kp, 1.0e-10
        assert_time(opts[:tn], tn)
        assert_time(opts[:td], td)

        assert {:ok, pid} =
                 Controller.new(out_min: 0, out_max: 100, p_enabled: false, tn: 1, td: 1)

        assert {:ok, _pid} = Controller.configure(pid, opts)
      end
    end

    test "rejects invalid and incompatible gain requests and unrepresentable gains" do
      step = step_model(2, 20, 5)
      relay = %{method: :relay, ultimate_gain: 4, ultimate_period_seconds: 8}
      assert Autotuner.gains(step, :id, :ziegler_nichols) == {:error, :invalid_form}
      assert Autotuner.gains(step, :p, :unknown) == {:error, :invalid_profile}
      assert Autotuner.gains(step, :pid, :relay_pid) == {:error, :incompatible_profile}
      assert Autotuner.gains(relay, :p, :relay_pid) == {:error, :incompatible_profile}
      assert Autotuner.gains(relay, :pid, :ziegler_nichols) == {:error, :incompatible_profile}
      assert Autotuner.gains(%{}, :pid, :relay_pid) == {:error, :invalid_model}

      assert Autotuner.gains(%{relay | ultimate_period_seconds: 0}, :pid, :relay_pid) ==
               {:error, :invalid_model}

      for model <- [step_model(0, 1, 1), step_model(1, 0, 1), step_model(1, 1, 0)] do
        assert Autotuner.gains(model, :p, :ziegler_nichols) == {:error, :invalid_model}
      end

      assert Autotuner.gains(step_model(1, 1.0e308, 1.0e-308), :p, :ziegler_nichols) ==
               {:error, :numeric_overflow}

      assert Autotuner.gains(step_model(1, 1.0e-308, 1.0e308), :p, :ziegler_nichols) ==
               {:error, :invalid_gains}
    end
  end

  describe "relay identification" do
    test "switches at inclusive thresholds and holds the command inside the band" do
      assert {:ok, 30.0, tuner} = Autotuner.start(relay_opts(), 21)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 20, 1)
      assert {:running, 50.0, tuner} = Autotuner.step(tuner, 19.5, 1)
      assert {:running, 50.0, tuner} = Autotuner.step(tuner, 20, 1)
      assert {:running, 30.0, tuner} = Autotuner.step(tuner, 20.5, 1)
      assert Autotuner.cancel(tuner) == {:ok, 25.0}
    end

    test "measures amplitude and period with nonzero low output and irregular samples" do
      for {direction, sign, gain} <- [{:direct, 1, 0.4}, {:reverse, -1, -0.4}] do
        opts = relay_opts(direction: direction)
        assert {:ok, 25.0, model} = relay_trace(opts, sign, List.duplicate(2.0, 11))
        assert model.method == :relay
        assert_in_delta model.ultimate_gain, 20.0 / :math.pi(), 1.0e-10
        assert_in_delta model.ultimate_period_seconds, 8.0, 1.0e-10

        assert {:ok, gains} = Autotuner.gains(model, :pid, :relay_pid)
        assert gains[:p_enabled]
        assert_in_delta gains[:kp], 5.0 / :math.pi(), 1.0e-10
        assert_in_delta gains[:tn], 8.0, 1.0e-10
        assert_in_delta gains[:td], 1.0, 1.0e-10
        assert_closed_loop(gains, direction, gain)
      end
    end

    test "rejects undersampled and drifting oscillations and times out without both crossings" do
      opts = relay_opts()
      assert {:ok, 50.0, tuner} = Autotuner.start(opts, 20)

      undersampled =
        Enum.reduce_while([20.6, 19.4, 20.6, 19.4, 20.6], tuner, fn measurement, tuner ->
          case Autotuner.step(tuner, measurement, 1) do
            {:running, _command, next} -> {:cont, next}
            result -> {:halt, result}
          end
        end)

      assert undersampled == {:error, :undersampled_oscillation, 25.0}

      amplitudes = List.duplicate(2.0, 9) ++ [4.0, 4.0]
      assert relay_trace(opts, 1, amplitudes) == {:error, :irregular_oscillation, 25.0}

      assert {:ok, 50.0, tuner} = Autotuner.start(relay_opts(timeout_seconds: 2), 20)
      assert {:running, 50.0, tuner} = Autotuner.step(tuner, 20, 1)
      assert Autotuner.step(tuner, 20, 1) == {:error, :timeout, 25.0}
    end
  end

  defp step_opts(overrides \\ []), do: Keyword.merge(@step_options, overrides)

  defp relay_opts(overrides \\ []), do: Keyword.merge(@relay_options, overrides)

  defp step_model(gain, tau, delay) do
    %{process_gain: gain, time_constant_seconds: tau, dead_time_seconds: delay}
  end

  defp begin_step(opts) do
    baseline = opts[:baseline_output] * 1.0
    step = opts[:step_output] * 1.0
    assert {:ok, ^baseline, tuner} = Autotuner.start(opts, 20)

    [1.0]
    |> Stream.cycle()
    |> Enum.take(200)
    |> Enum.reduce_while(tuner, fn elapsed, tuner ->
      case Autotuner.step(tuner, 20, elapsed) do
        {:running, ^baseline, next} -> {:cont, next}
        {:running, ^step, next} -> {:halt, next}
      end
    end)
  end

  defp identify(opts, response) do
    command = opts[:step_output] * 1.0

    [0.4, 0.6, 0.5]
    |> Stream.cycle()
    |> Enum.take(600)
    |> Enum.reduce_while({begin_step(opts), 0.0}, fn elapsed, {tuner, time} ->
      time = time + elapsed

      case Autotuner.step(tuner, response.(time), elapsed) do
        {:running, ^command, next} -> {:cont, {next, time}}
        result -> {:halt, result}
      end
    end)
  end

  defp first_order_response(time, change, delay) do
    20 + change * (1 - :math.exp(-max(0.0, time - delay) / 12.0))
  end

  defp assert_time(nil, nil), do: :ok
  defp assert_time(actual, expected), do: assert_in_delta(actual, expected, 1.0e-10)

  defp relay_trace(opts, sign, amplitudes) do
    assert {:ok, 50.0, tuner} = Autotuner.start(opts, 20)

    # Each cycle ends at the high-to-low switch. Its valley and peak
    # occur inside the half-cycles, before the next threshold crossing.
    deviations =
      [0.6] ++
        Enum.flat_map(amplitudes, fn amplitude ->
          [1, amplitude, 0, -0.6, -1, -amplitude, 0, 0.6]
        end)

    deviations
    |> Enum.zip(Stream.cycle([0.75, 1.25]))
    |> Enum.reduce_while({tuner, 50.0}, fn {deviation, elapsed}, {tuner, previous} ->
      case Autotuner.step(tuner, 20 + sign * deviation, elapsed) do
        {:running, command, next} ->
          expected =
            cond do
              deviation <= -0.5 -> 50.0
              deviation >= 0.5 -> 30.0
              true -> previous
            end

          assert command == expected
          {:cont, {next, command}}

        result ->
          {:halt, result}
      end
    end)
  end

  defp assert_closed_loop(gains, direction, process_gain) do
    effective_output = 27.0
    target = 22.0
    {:ok, pid} = Controller.new(out_min: 0, out_max: 100, direction: direction)
    {:ok, pid} = Controller.set_mode(pid, {:manual, 50})
    {:ok, pid} = Controller.configure(pid, gains)
    {:ok, pid} = Controller.set_mode(pid, :automatic)
    {:ok, pid} = Controller.track_output(pid, effective_output)
    assert {:ok, ^effective_output, pid} = Controller.step(pid, target, 20, 0.5)

    # Exact zero-order-hold update for the first-order plant. The queue
    # delays the command by six samples (three seconds).
    commands = :queue.from_list(List.duplicate(effective_output, 6))

    {measurement, _pid, _commands} =
      Enum.reduce(1..300, {20.0, pid, commands}, fn _tick, {measurement, pid, commands} ->
        {{:value, delayed}, commands} = :queue.out(commands)
        equilibrium = 20 + process_gain * (delayed - 30)
        measurement = equilibrium + (measurement - equilibrium) * :math.exp(-0.5 / 12.0)
        assert {:ok, command, pid} = Controller.step(pid, target, measurement, 0.5)
        assert command >= 0 and command <= 100
        {measurement, pid, :queue.in(command, commands)}
      end)

    assert_in_delta measurement, target, 0.05
  end
end
