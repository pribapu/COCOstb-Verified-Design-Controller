"""
runner.py -- build + run the SPI master regression with cocotb's Python runner.

Elaborates the DUT at two word widths (8 and 16) to exercise the DATA_WIDTH
parameter, runs the full self-checking suite against each, plus one build of
the APB register/FIFO/IRQ wrapper, aggregates functional coverage into one
JSON, and exits non-zero if coverage is below 100%.

    python3 runner.py            # regression + coverage (no waves)
    python3 runner.py --waves    # extra single-width traced run -> tb/dump.vcd

Set SIM=verilator or SIM=icarus (default: verilator).
"""

import os
import sys

from cocotb_tools.runner import get_runner

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RTL = os.path.join(ROOT, "rtl", "spi_master.sv")
RTL_APB = os.path.join(ROOT, "rtl", "spi_apb_wrapper.sv")
COV_FILE = os.path.join(HERE, "spi_cov.json")
SIM = os.environ.get("SIM", "verilator")


def run_width(width):
    runner = get_runner(SIM)
    runner.build(
        sources=[RTL],
        hdl_toplevel="spi_master",
        parameters={"DATA_WIDTH": width},
        build_dir=os.path.join(HERE, f"sim_build_w{width}"),
        always=True,
    )
    runner.test(
        hdl_toplevel="spi_master",
        test_module="test_spi_master",
        test_dir=HERE,
        extra_env={"SPI_COV_FILE": COV_FILE, "PYTHONPATH": HERE},
    )


def run_apb():
    runner = get_runner(SIM)
    runner.build(
        sources=[RTL, RTL_APB],
        hdl_toplevel="spi_apb_wrapper",
        build_dir=os.path.join(HERE, "sim_build_apb"),
        always=True,
    )
    runner.test(
        hdl_toplevel="spi_apb_wrapper",
        test_module="test_spi_apb",
        test_dir=HERE,
        extra_env={"SPI_COV_FILE": COV_FILE, "PYTHONPATH": HERE},
    )


def run_waves():
    runner = get_runner(SIM)
    runner.build(
        sources=[RTL], hdl_toplevel="spi_master", parameters={"DATA_WIDTH": 8},
        build_dir=os.path.join(HERE, "wave_build"), waves=True, always=True,
    )
    runner.test(
        hdl_toplevel="spi_master", test_module="test_waves", test_dir=HERE,
        extra_env={"PYTHONPATH": HERE}, waves=True,
    )
    print("Waveform written to tb/dump.vcd  (open with: gtkwave tb/dump.vcd)")


def main():
    if "--waves" in sys.argv:
        run_waves()
        return
    if os.path.exists(COV_FILE):
        os.remove(COV_FILE)
    for width in (8, 16):
        run_width(width)
    run_apb()

    sys.path.insert(0, HERE)
    from spi_coverage import Coverage
    final = Coverage()
    final.merge_file(COV_FILE)
    report, overall = final.report()
    print(report)
    if overall < 100.0:
        print(f"\nERROR: functional coverage {overall:.1f}% < 100%")
        sys.exit(1)
    print(f"\nAll functional-coverage goals met: {overall:.1f}%")


if __name__ == "__main__":
    main()
