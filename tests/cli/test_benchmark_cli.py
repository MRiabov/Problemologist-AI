from typer.testing import CliRunner
from controller.cli.benchmark import app

runner = CliRunner()

def test_benchmark_cli_help():
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "Benchmark Generation CLI" in result.stdout

def test_benchmark_cli_import():
    """Verify that the module can be imported and dependencies are resolved."""
    from controller.cli import benchmark
    assert benchmark.app is not None
