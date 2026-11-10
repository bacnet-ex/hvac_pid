defmodule HvacPidTest do
  alias HvacPid.Controller, as: PID

  use ExUnit.Case, async: true

  doctest PID

  describe "configuration" do
    test "requires both bounds and accepts equal bounds" do
      assert PID.new([]) == {:error, {:missing_option, :out_min}}
      assert PID.new(out_min: 5) == {:error, {:missing_option, :out_max}}
      assert PID.new(out_min: 5, out_max: 4) == {:error, :invalid_output_limits}

      pid = automatic(out_min: 4, out_max: 4, bias: -20, tn: 1, td: 2, start_value: 8)
      assert {:ok, 4.0, pid} = PID.step(pid, 100, 0, 1)
      assert {:ok, 4.0, pid} = PID.step(pid, 0, 100, 1)
      assert {:ok, 4.0, _pid} = PID.step(pid, 100, 0, 1)
    end

    test "rejects malformed, unknown, duplicate, and invalid settings" do
      for opts <- [%{out_min: 0, out_max: 100}, [:kp], [{1, 2}], [kp: 1] ++ :improper] do
        assert PID.new(opts) == {:error, :invalid_options}
      end

      assert PID.new(ki: 2) == {:error, {:unknown_option, :ki}}
      assert PID.new(out_min: 0, out_min: 1) == {:error, {:duplicate_option, :out_min}}

      for {key, value} <- [
            out_min: nil,
            out_max: :infinity,
            kp: -1,
            kp: "2",
            kp: Integer.pow(10, 400),
            tn: 0,
            tn: "10",
            td: -1,
            p_enabled: 1,
            bias: true,
            derivative_on: :setpoint,
            direction: :heating,
            deadzone: -0.1,
            start_value: :default,
            stop_value: []
          ] do
        assert PID.new([{key, value}]) == {:error, {:invalid_option, key}}
      end
    end

    test "applies both bounds together and rejects a bad batch without retuning" do
      pid = automatic(kp: 2, tn: 1)
      assert {:ok, 4.0, pid} = PID.step(pid, 2, 0, 1)

      assert PID.configure(pid, out_min: 150, kp: 5) == {:error, :invalid_output_limits}
      assert PID.configure(pid, kp: 5, td: 0) == {:error, {:invalid_option, :td}}
      assert {:ok, 8.0, _pid} = PID.step(pid, 2, 0, 1)

      assert {:ok, moved} = PID.configure(pid, out_min: 150, out_max: 200)
      assert PID.output(moved) === 150.0
      assert {:ok, 150.0, _pid} = PID.step(moved, 2, 0, 1)
    end

    test "empty and equivalent tuning batches preserve sample history" do
      pid = automatic(tn: 1)
      assert {:ok, 3.0, pid} = PID.step(pid, 3, 0, 1)
      assert {:ok, ^pid} = PID.configure(pid, [])
      assert {:ok, ^pid} = PID.configure(pid, tn: 1.0, kp: 1)
      assert {:ok, 6.0, _pid} = PID.step(pid, 3, 0, 1)
    end
  end

  describe "control law" do
    test "produces hand-calculated P, PI, PD, and PID outputs" do
      for {terms, second_output} <- [
            {[], 11.0},
            {[tn: 2], 13.25},
            {[td: 0.5], 8.0},
            {[tn: 2, td: 0.5], 10.25}
          ] do
        pid = automatic([kp: 3, bias: 2] ++ terms)

        assert {:ok, 14.0, pid} = PID.step(pid, 10, 6, 1)
        assert {:ok, ^second_output, pid} = PID.step(pid, 10, 7, 0.5)
        assert PID.output(pid) === second_output
      end
    end

    test "keeps the shared gain with P disabled" do
      for {terms, second_output} <- [
            {[], 2.0},
            {[tn: 2], 4.25},
            {[td: 0.5], -1.0},
            {[tn: 2, td: 0.5], 1.25}
          ] do
        pid = automatic([kp: 3, bias: 2, p_enabled: false] ++ terms)

        assert {:ok, 2.0, pid} = PID.step(pid, 10, 6, 1)
        assert {:ok, ^second_output, _pid} = PID.step(pid, 10, 7, 0.5)
      end
    end

    test "zero gain keeps all terms at zero even with extreme time constants" do
      for gain <- [0, -0.0] do
        pid = automatic(kp: gain, bias: 5, tn: 5.0e-324, td: 1.0e308)
        assert {:ok, 5.0, pid} = PID.step(pid, 10, 4, 1)
        assert {:ok, 5.0, _pid} = PID.step(pid, 10, 2, 5.0e-324)
        assert {:ok, 5.0, _pid} = PID.step(pid, 10, 2, 100)
      end
    end

    test "direct action responds to a falling measurement and reverse to a rising measurement" do
      for {direction, measurement} <- [direct: 18, reverse: 22] do
        pid = automatic(kp: 5, direction: direction, bias: 15)
        assert {:ok, 15.0, pid} = PID.step(pid, 20, 20, 1)
        assert {:ok, 25.0, _pid} = PID.step(pid, 20, measurement, 1)
      end
    end

    test "derivative sources agree at a fixed setpoint and differ when only the setpoint changes" do
      for {direction, signed_output} <- [direct: 2.0, reverse: -2.0],
          source <- [:measurement, :error] do
        pid =
          automatic(
            kp: 2,
            p_enabled: false,
            td: 0.5,
            direction: direction,
            derivative_on: source
          )

        assert {:ok, command, pid} = PID.step(pid, 22, 20, 0.5)
        assert command == 0.0
        assert {:ok, ^signed_output, pid} = PID.step(pid, 22, 19, 0.5)
        expected = if source == :error, do: signed_output, else: 0.0
        assert {:ok, ^expected, _pid} = PID.step(pid, 23, 19, 0.5)
      end
    end

    test "scales I and D at one millisecond and at irregular intervals" do
      pid = automatic(kp: 2, p_enabled: false, tn: 1, td: 0.5, out_min: -1_000)

      assert {:ok, command, pid} = PID.step(pid, 1, 0, 0.001)
      assert command == 0.0
      assert {:ok, command, pid} = PID.step(pid, 1, 0.5, 0.001)
      assert_in_delta command, -499.999, 1.0e-10
      assert {:ok, command, pid} = PID.step(pid, 1, 0.25, 0.25)
      assert_in_delta command, 1.376, 1.0e-10
      assert {:ok, command, _pid} = PID.step(pid, 1, 0.625, 0.75)
      assert_in_delta command, 0.4385, 1.0e-10
    end
  end

  describe "output limits and anti-windup" do
    test "integrates up to the upper bound and recovers without stored excess" do
      pid = automatic(out_min: -3, out_max: 12, kp: 2, tn: 1, bias: 4)

      assert {:ok, 8.0, pid} = PID.step(pid, 2, 0, 1)
      assert {:ok, 12.0, pid} = PID.step(pid, 2, 0, 2)
      assert {:ok, 12.0, pid} = PID.step(pid, 20, 0, 20)
      assert {:ok, 4.0, _pid} = PID.step(pid, -1, 0, 1)
    end

    test "integrates down to the lower bound and recovers without stored excess" do
      pid = automatic(out_min: -3, out_max: 12, kp: 2, tn: 1, bias: 4)

      assert {:ok, command, pid} = PID.step(pid, -2, 0, 1)
      assert command == 0.0
      assert {:ok, -3.0, pid} = PID.step(pid, -2, 0, 2)
      assert {:ok, -3.0, pid} = PID.step(pid, -20, 0, 20)
      assert {:ok, 5.0, _pid} = PID.step(pid, 1, 0, 1)
    end

    test "integrates back toward the range even while the output is saturated" do
      for {bias, error, bound, recovered} <- [{20, -3, 10.0, 8.0}, {-10, 3, 0.0, 2.0}] do
        pid = automatic(out_min: 0, out_max: 10, bias: bias, p_enabled: false, tn: 1)

        assert {:ok, ^bound, pid} = PID.step(pid, error, 0, 2)
        assert {:ok, ^bound, pid} = PID.step(pid, error, 0, 2)
        assert {:ok, ^recovered, _pid} = PID.step(pid, error, 0, 2)
      end
    end

    test "includes the derivative term when calculating integral headroom" do
      pid = automatic(out_min: 0, out_max: 10, p_enabled: false, bias: 5, tn: 1, td: 1)

      assert {:ok, 5.0, pid} = PID.step(pid, 3, 0, 1)
      assert {:ok, 10.0, pid} = PID.step(pid, 3, -2, 1)
      assert {:ok, 8.0, _pid} = PID.step(pid, -2, -2, 1)
    end
  end

  describe "modes and tracking" do
    test "disabled emits its stop command and clears active history" do
      assert {:ok, disabled} = PID.new(out_min: -10, out_max: 40)
      assert PID.output(disabled) === -10.0
      assert {:ok, -10.0, ^disabled} = PID.step(disabled, 20, 10, 1)
      assert {:ok, ^disabled} = PID.track_output(disabled, 30)
      assert {:ok, ^disabled} = PID.set_mode(disabled, :disabled)

      assert {:ok, pid} = PID.configure(disabled, stop_value: 80, tn: 1, td: 2)
      assert PID.output(pid) === 40.0
      assert {:ok, pid} = PID.set_mode(pid, :automatic)
      assert {:ok, 10.0, pid} = PID.step(pid, 20, 10, 1)
      assert {:ok, 40.0, pid} = PID.step(pid, 20, 5, 1)
      assert {:ok, pid} = PID.track_output(pid, 23)
      assert {:ok, pid} = PID.set_mode(pid, :disabled)
      assert PID.output(pid) === 40.0
      assert {:ok, pid} = PID.configure(pid, stop_value: nil)
      assert PID.output(pid) === -10.0
      assert {:ok, pid} = PID.set_mode(pid, :automatic)
      assert {:ok, 3.0, _pid} = PID.step(pid, 20, 17, 20)
    end

    test "clamps the startup output and seeds history even inside the deadzone" do
      pid =
        automatic(
          out_min: 0,
          out_max: 100,
          start_value: -5,
          bias: 10,
          kp: 2,
          tn: 2,
          td: 1,
          deadzone: 1
        )

      assert {:ok, command, pid} = PID.step(pid, 20, 19, 10)
      assert command == 0.0
      assert {:ok, 11.0, _pid} = PID.step(pid, 20, 17, 1)
    end

    test "pending startup uses new settings and a repeated automatic request is idempotent" do
      pid = automatic(out_min: 0, out_max: 100, kp: 2, start_value: 90, tn: 1)
      assert {:ok, ^pid} = PID.set_mode(pid, :automatic)
      assert {:ok, pid} = PID.configure(pid, start_value: 20, kp: 4)
      assert {:ok, 20.0, pid} = PID.step(pid, 10, 9, 1)
      assert {:ok, ^pid} = PID.set_mode(pid, :automatic)
      assert {:ok, 24.0, _pid} = PID.step(pid, 10, 9, 1)
    end

    test "transfers from manual without a manual sample and clamps a manual command immediately" do
      pid = automatic(out_min: 0, out_max: 100, bias: 10, kp: 2, td: 1)
      assert {:ok, 14.0, pid} = PID.step(pid, 10, 8, 1)
      assert {:ok, pid} = PID.set_mode(pid, {:manual, 150})
      assert PID.output(pid) === 100.0
      assert {:ok, pid} = PID.configure(pid, out_max: 60)
      assert PID.output(pid) === 60.0
      assert {:ok, pid} = PID.configure(pid, out_max: 100)
      assert PID.output(pid) === 60.0
      assert {:ok, pid} = PID.set_mode(pid, :automatic)
      assert {:ok, 60.0, pid} = PID.step(pid, 10, 1, 10)
      assert {:ok, 56.0, _pid} = PID.step(pid, 10, 2, 1)
    end

    test "tracking preserves a manual command and reapplying it preserves the tracked target" do
      pid = automatic(out_min: 0, out_max: 100, kp: 2, tn: 2, td: 1)
      assert {:ok, pid} = PID.set_mode(pid, {:manual, 35})
      assert {:ok, pid} = PID.track_output(pid, 42)
      assert {:ok, 35.0, pid} = PID.step(pid, 20, 0, 1)
      assert {:ok, ^pid} = PID.set_mode(pid, {:manual, 35.0})
      assert {:ok, pid} = PID.set_mode(pid, :automatic)
      assert {:ok, 42.0, pid} = PID.step(pid, 20, 18, 5)
      assert {:ok, 39.0, _pid} = PID.step(pid, 20, 19, 1)
    end

    test "a different manual command clears an earlier tracked target" do
      pid = automatic(out_min: 0, out_max: 100)
      assert {:ok, pid} = PID.set_mode(pid, {:manual, 40})
      assert {:ok, pid} = PID.track_output(pid, 55)
      assert {:ok, pid} = PID.set_mode(pid, {:manual, -5})
      assert PID.output(pid) === 0.0
      assert {:ok, pid} = PID.set_mode(pid, :automatic)
      assert {:ok, command, _pid} = PID.step(pid, 21, 20, 1)
      assert command == 0.0
    end

    test "tracking supersedes startup and the latest effective output is clamped" do
      pid = automatic(out_min: 0, out_max: 100, start_value: 20)
      assert {:ok, pid} = PID.track_output(pid, 40)
      assert {:ok, pid} = PID.track_output(pid, 200)
      assert PID.output(pid) === 0.0
      assert {:ok, 100.0, pid} = PID.step(pid, 20, 18, 1)
      assert {:ok, 99.0, _pid} = PID.step(pid, 20, 19, 1)
    end

    test "tracking after a pause resets I and D and uses the latest sample" do
      pid = automatic(kp: 2, tn: 1, td: 2, deadzone: 1)
      assert {:ok, 10.0, pid} = PID.step(pid, 10, 5, 1)
      assert {:ok, 28.0, pid} = PID.step(pid, 10, 4, 1)
      assert {:ok, pid} = PID.track_output(pid, 27)
      assert PID.output(pid) === 28.0
      assert {:ok, 27.0, pid} = PID.step(pid, 10, 10, 3_600)
      assert {:ok, 43.0, _pid} = PID.step(pid, 10, 8, 1)
    end
  end

  describe "retuning" do
    test "reanchors for each change and uses the new law from the following sample" do
      pid = automatic(kp: 2, tn: 2, td: 1, bias: 4)
      assert {:ok, 14.0, pid} = PID.step(pid, 10, 5, 1)
      assert {:ok, 14.0, pid} = PID.step(pid, 10, 6, 1)

      for {opts, target, next} <- [
            {[kp: 4], 14.0, 26.0},
            {[tn: 4], 14.0, 17.0},
            {[tn: nil], 14.0, 14.0},
            {[td: 2], 14.0, 18.0},
            {[td: nil], 14.0, 22.0},
            {[p_enabled: false], 14.0, 18.0},
            {[derivative_on: :error], 14.0, 24.0},
            {[direction: :reverse], 14.0, 8.0},
            {[bias: 8], 14.0, 20.0},
            {[out_min: 16], 16.0, 22.0},
            {[out_max: 12], 12.0, 12.0}
          ] do
        assert {:ok, retuned} = PID.configure(pid, opts)
        assert PID.output(retuned) === target
        assert {:ok, ^target, retuned} = PID.step(retuned, 12, 7, 0.001)
        assert {:ok, ^next, _pid} = PID.step(retuned, 14, 8, 1)
      end
    end

    test "successive changes preserve the tracked target within the new limits" do
      pid = automatic(out_min: 0, out_max: 100, kp: 5)
      assert {:ok, 10.0, pid} = PID.step(pid, 20, 18, 1)
      assert {:ok, pid} = PID.configure(pid, kp: 8)
      assert {:ok, pid} = PID.track_output(pid, 70)
      assert {:ok, pid} = PID.configure(pid, bias: 10, out_max: 50)
      assert {:ok, pid} = PID.configure(pid, direction: :reverse, out_max: 100)
      assert PID.output(pid) === 10.0
      assert {:ok, 50.0, pid} = PID.step(pid, 20, 24, 1)
      assert {:ok, 42.0, _pid} = PID.step(pid, 20, 23, 1)
    end
  end

  describe "deadzone" do
    test "holds at the boundary, freezes I and tracking, and updates both derivative sources" do
      for {source, expected_exit} <- [measurement: 28.0, error: 31.0] do
        pid =
          automatic(
            kp: 2,
            tn: 2,
            td: 1,
            bias: 10,
            start_value: 21,
            deadzone: 1,
            derivative_on: source
          )

        assert {:ok, 21.0, pid} = PID.step(pid, 10, 7, 1)
        assert {:ok, 19.0, pid} = PID.step(pid, 10, 8, 1)
        assert {:ok, 19.0, pid} = PID.step(pid, 11, 10, 10)
        assert {:ok, 19.0, pid} = PID.step(pid, 10.5, 10, 10)
        assert {:ok, ^expected_exit, _pid} = PID.step(pid, 12, 9, 1)
      end
    end

    test "disabling the deadzone keeps history and allows changes at zero error" do
      pid = automatic(deadzone: 1, td: 2, derivative_on: :measurement, bias: 20)
      assert {:ok, 23.0, pid} = PID.step(pid, 10, 7, 1)
      assert {:ok, 23.0, pid} = PID.step(pid, 10, 9, 1)
      assert {:ok, pid} = PID.configure(pid, deadzone: 0, start_value: 80, stop_value: 30)
      assert {:ok, 18.0, _pid} = PID.step(pid, 10, 10, 1)
    end
  end

  describe "errors and retries" do
    test "validates each argument even while manual or disabled" do
      assert {:ok, disabled} = PID.new(out_min: 0, out_max: 100)
      assert {:ok, manual} = PID.set_mode(disabled, {:manual, 20})
      assert {:ok, auto} = PID.set_mode(disabled, :automatic)

      for pid <- [disabled, manual, auto] do
        assert PID.step(pid, "21", 20, 1) == {:error, {:invalid_input, :setpoint}}
        assert PID.step(pid, 21, nil, 1) == {:error, {:invalid_input, :measurement}}

        for interval <- [0, -0.01, "1", Integer.pow(10, 400)] do
          assert PID.step(pid, 21, 20, interval) == {:error, {:invalid_input, :elapsed_seconds}}
        end

        assert PID.set_mode(pid, :manual) == {:error, :invalid_mode}

        assert PID.set_mode(pid, {:manual, :infinity}) ==
                 {:error, {:invalid_input, :manual_output}}

        assert PID.track_output(pid, nil) == {:error, {:invalid_input, :applied_output}}
      end
    end

    test "returns numeric overflow for signed error, startup P, and bias plus P" do
      pid = automatic(kp: 1.0e308, bias: 1.0e308)
      assert PID.step(pid, 1.0e308, -1.0e308, 1) == {:error, :numeric_overflow}
      assert PID.step(pid, 2, 0, 1) == {:error, :numeric_overflow}
      assert PID.step(pid, 1, 0, 1) == {:error, :numeric_overflow}
      assert {:ok, 100.0, _pid} = PID.step(pid, 0, 0, 1)
    end

    test "returns numeric overflow for an unrepresentable tracking adjustment" do
      pid = automatic(out_min: -1.0e308, kp: 1.0e308, stop_value: 0)
      assert {:ok, pid} = PID.track_output(pid, -1.0e308)
      assert PID.step(pid, 1, 0, 1) == {:error, :numeric_overflow}
      assert PID.output(pid) === 0.0
      assert {:ok, -1.0e308, _pid} = PID.step(pid, 0, 0, 1)
    end

    test "returns numeric overflow for D at a very short interval and permits a retry" do
      pid = automatic(td: 1)
      assert {:ok, 10.0, pid} = PID.step(pid, 10, 0, 5.0e-324)
      assert PID.step(pid, 10, 1, 5.0e-324) == {:error, :numeric_overflow}
      assert PID.output(pid) === 10.0
      assert {:ok, 8.0, _pid} = PID.step(pid, 10, 1, 1)
    end

    test "returns numeric overflow for I and skips the division at zero error" do
      pid = automatic(tn: 5.0e-324)
      assert {:ok, 1.0, pid} = PID.step(pid, 1, 0, 1)
      assert PID.step(pid, 1, 0, 1) == {:error, :numeric_overflow}
      assert {:ok, command, _pid} = PID.step(pid, 1, 1, 1)
      assert command == 0.0
    end
  end

  defp automatic(opts) do
    {:ok, pid} = PID.new(Keyword.merge([out_min: -100, out_max: 100], opts))
    {:ok, pid} = PID.set_mode(pid, :automatic)
    pid
  end
end
