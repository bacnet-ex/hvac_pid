defmodule HvacPidPlantTest do
  alias HvacPid.Controller, as: PID

  use ExUnit.Case, async: true

  test "heats a room toward the setpoint with irregular sampling" do
    assert_tracks_setpoint(:direct, 14.0, 1.0)
  end

  test "cools a room toward the setpoint with irregular sampling" do
    assert_tracks_setpoint(:reverse, 32.0, -1.0)
  end

  defp assert_tracks_setpoint(direction, ambient, action) do
    setpoint = 21.0
    {:ok, pid} = PID.new(out_min: 0, out_max: 100, direction: direction, kp: 8, tn: 60)
    {:ok, pid} = PID.set_mode(pid, :automatic)

    {temperature, pid} =
      [0.25, 0.75, 1.5, 2.0]
      |> Stream.cycle()
      |> Enum.take(480)
      |> Enum.reduce({ambient, pid}, fn elapsed, {temperature, pid} ->
        assert {:ok, command, pid} = PID.step(pid, setpoint, temperature, elapsed)
        assert command >= 0.0 and command <= 100.0

        # The room leaks heat to its surroundings with a 60-second time constant.
        # At full output the actuator contributes one degree per second.
        rate = (ambient - temperature) / 60 + action * command / 100
        {temperature + elapsed * rate, pid}
      end)

    assert_in_delta temperature, setpoint, 0.01
    assert_in_delta PID.output(pid), abs(setpoint - ambient) * 100 / 60, 0.05
  end
end
