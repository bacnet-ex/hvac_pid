defmodule HvacPid.MixProject do
  use Mix.Project

  @version "0.1.0"
  @source_url "https://github.com/bacnet-ex/hvac_pid"

  @spec project() :: keyword()
  def project do
    [
      app: :hvac_pid,
      name: "HVAC PID",
      description: "A general-purpose PID controller for discrete control loops.",
      version: @version,
      elixir: "~> 1.15",
      elixirc_paths: elixirc_paths(Mix.env()),
      elixirc_options: [],
      source_url: @source_url,
      aliases: aliases(),
      deps: deps(),
      package: package(),
      dialyzer: [
        # Flags could be ["-Wno_opaque"]
        flags: [],
        ignore_warnings: "dialyzer.ignore-warnings.exs",
        plt_add_apps: [:mix]
      ],
      source_url: @source_url,
      docs: docs(),
      test_coverage: [
        ignore_modules: [
          ~r"Inspect\..+",
          ~r"HvacPid\.Test\..+",
          ~r"Mix\.Tasks\..+"
        ],
        summary: true,
        tool: Mix.Tasks.Test.Coverage
      ]
    ]
  end

  def cli() do
    [preferred_envs: [precommit: :test]]
  end

  defp aliases() do
    [
      precommit: [
        "format --force",
        "compile --force",
        "credo --all --strict --ignore todo",
        # credo task is disabled after the run, so we need to enable it again
        fn _args -> Mix.Task.reenable("credo") end,
        "credo --all --strict -C test",
        "doctor --failed",
        "dialyzer",
        "test"
      ]
    ]
  end

  # Specifies which paths to compile per environment
  defp elixirc_paths(:test), do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]

  @spec deps() :: [{atom(), String.t(), keyword()}]
  defp deps do
    [
      {:credo, "~> 1.4", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:doctor, "~> 0.21", only: [:dev, :test], runtime: false},
      {:ex_doc, "~> 0.40.0", only: [:dev, :test], runtime: false}
    ]
  end

  @spec package() :: keyword()
  defp package do
    [
      files: ~w(lib .formatter.exs mix.exs README.md LICENSE),
      licenses: ["Apache-2.0"],
      links: %{"GitHub" => @source_url}
    ]
  end

  defp docs() do
    [
      main: "HvacPid",
      source_ref: "master",
      groups_for_docs: [Guards: & &1[:guard]],
      groups_for_modules: [],
      nest_modules_by_prefix: []
      # before_closing_head_tag: &docs_before_closing_head_tag/1,
      # before_closing_body_tag: &docs_before_closing_body_tag/1
    ]
  end
end
