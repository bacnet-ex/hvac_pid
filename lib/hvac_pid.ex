defmodule HvacPid do
  @readme "README.md"
  @external_resource @readme

  @moduledoc @readme
             |> File.read!()
             |> String.split("\n", parts: 2)
             |> List.last()
             |> String.trim()
end
