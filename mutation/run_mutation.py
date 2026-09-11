"""
run_mutation.py -- mutation testing of the verification environment.

Each mutant below is a realistic, hand-written RTL bug (one targeted text
edit). For every mutant, in an isolated copy of the repo, this script runs

  * the full cocotb simulation regression (tb/runner.py: W=8, W=16, APB), and
  * the formal flow: the unbounded proof plus a bounded check *without* the
    helper invariants (bmc_ext), so a formal kill is attributed to the
    externally meaningful property the bug violates,

and reports which method killed it. A mutant that survives both would mean a
hole in the verification; the script exits non-zero if any does.

    python3 run_mutation.py                 # all mutants, parallel
    python3 run_mutation.py -k C1 W6        # just these
    python3 run_mutation.py --jobs 4

Environment: SIM (default icarus), SBY_FLAGS (extra arguments for `sby`,
e.g. tool paths on Windows), PYTHON (interpreter for the cocotb runner).
"""

import argparse
import concurrent.futures as cf
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

CORE = "spi_master.sv"
WRAP = "spi_apb_wrapper.sv"

# (id, file, find, replace, description). `find` must match exactly once.
MUTANTS = [
    ("C1", CORE, "                    sclk <= cpol;\n                    if (start) begin",
     "                    if (start) begin",
     "SCLK doesn't track CPOL while idle (the original bug)"),
    ("C2", CORE, "if (edge_cnt == $clog2(NEDGES+1)'(NEDGES-1))",
     "if (edge_cnt == $clog2(NEDGES+1)'(NEDGES-2))",
     "Transfer ends one SCLK edge early"),
    ("C3", CORE, "                                sh_rx[cur_index(bit_i)] <= miso;         // sample\n"
                 "                            else",
     "                                sh_rx[cur_index(bit_i)] <= 1'b0;         // sample\n"
     "                            else",
     "CPHA=0 samples a constant instead of MISO"),
    ("C4", CORE, "cur_index = lsb_l ?", "cur_index = !lsb_l ?",
     "Bit order inverted"),
    ("C5", CORE, "cs_n <= 1'b0;    // assert chip-select", "cs_n <= 1'b1;    // assert chip-select",
     "Chip-select never asserted"),
    ("C6", CORE, "sclk <= cpol;    // idle at CPOL", "sclk <= ~cpol;   // idle at CPOL",
     "SCLK starts the frame at the wrong polarity"),
    ("C7", CORE, "state <= S_SETUP;", "state <= S_XFER;",
     "CS-to-first-SCLK setup half-period skipped"),
    ("C8", CORE, "                        div_cnt  <= div_l;",
     "                        div_cnt  <= clk_div;",
     "Divider re-read live mid-word instead of latched"),
    ("C9", CORE, "if (cpha_l == 1'b0) begin", "if (cpha == 1'b0) begin",
     "CPHA re-read live mid-word instead of latched"),
    ("C10", CORE, ": tx_data[DATA_WIDTH-1]));", ": tx_data[0]));",
     "MSB-first CPHA=0 drives the wrong first bit"),

    ("W1", WRAP, "((tx_count < CNTW'(FIFO_DEPTH)) || tx_pop)",
     "((tx_count < CNTW'(FIFO_DEPTH - 1)) || tx_pop)",
     "TX FIFO reports full one entry early"),
    ("W2", WRAP, "(irq_done_sticky && !(irqstatus_wr && pwdata[0]))",
     "(irq_done_sticky && !(irqstatus_wr && pwdata[1]))",
     "IRQ_STATUS.DONE cleared by the wrong W1C bit"),
    ("W3", WRAP, "assign irq = |(irq_status & irq_en);", "assign irq = |irq_status;",
     "Interrupt enables ignored"),
    ("W4", WRAP, "|| rx_overrun_event;", "|| 1'b0;",
     "RX overrun never flagged"),
    ("W5", WRAP, "        end else if (tx_flush) begin\n            tx_wptr  <= '0;\n"
                 "            tx_rptr  <= '0;\n            tx_count <= '0;",
     "        end else if (tx_flush) begin\n            tx_wptr  <= '0;\n"
     "            tx_rptr  <= '0;",
     "TX_FLUSH resets the pointers but not the count"),
    ("W6", WRAP, "wire spi_idle = !spi_busy && !spi_done;", "wire spi_idle = !spi_busy;",
     "Engine may start a word on the core's done cycle"),
    ("W7", WRAP, "&& !mode_load && !mode_upd_q;", "&& !mode_load;",
     "Engine not held off for the cycle after a mode change"),
    ("W8", WRAP, "wire rx_pop            = rx_apb_pop_req && (rx_count != 0);",
     "wire rx_pop            = rx_apb_pop_req;",
     "Reading RXDATA when empty underflows the RX FIFO"),
    ("W9", WRAP, "wire irq_txe_event = tx_empty && !tx_empty_d;",
     "wire irq_txe_event = !tx_empty && tx_empty_d;",
     "TX_EMPTY interrupt fires on the wrong FIFO edge"),
    ("W10", WRAP, "wire       reg_valid = (paddr[APB_AW-1:5] == '0);",
     "wire       reg_valid = 1'b1;",
     "Unmapped addresses decoded as registers (no PSLVERR)"),
    ("W11", WRAP, "wire frame_idle = !frame_req && !cs_pin_q;", "wire frame_idle = !spi_busy;",
     "New mode applied inside a CS-held frame"),
    ("W12", WRAP, "wire ctrl_wr   = apb_write && reg_valid && (reg_sel == REG_CTRL);",
     "wire ctrl_wr   = apb_write && (reg_sel == REG_CTRL);",
     "Errored write aliases onto CTRL (the original decode bug)"),
]

FORMAL = {
    CORE: ("spi_master.sby", ["prove_w8", "bmc_ext"]),
    WRAP: ("spi_apb_wrapper.sby", ["prove", "bmc_ext"]),
}


def make_tree(dst):
    shutil.copytree(os.path.join(ROOT, "rtl"), os.path.join(dst, "rtl"))
    ign = shutil.ignore_patterns("sim_build*", "wave_build", "__pycache__", "*.xml",
                                 "*.json", "*.vcd", "*.fst")
    shutil.copytree(os.path.join(ROOT, "tb"), os.path.join(dst, "tb"), ignore=ign)
    os.makedirs(os.path.join(dst, "formal"))
    for f in os.listdir(os.path.join(ROOT, "formal")):
        if f.endswith((".sv", ".sby")):
            shutil.copy(os.path.join(ROOT, "formal", f), os.path.join(dst, "formal", f))


def apply(dst, fname, find, repl):
    path = os.path.join(dst, "rtl", fname)
    with open(path, newline="") as f:
        src = f.read().replace("\r\n", "\n")
    n = src.count(find)
    if n != 1:
        raise RuntimeError(f"mutation anchor found {n} times in {fname}: {find!r}")
    with open(path, "w", newline="\n") as f:
        f.write(src.replace(find, repl))


def run_sim(dst):
    env = dict(os.environ, SIM=os.environ.get("SIM", "icarus"))
    py = os.environ.get("PYTHON", sys.executable)
    try:
        p = subprocess.run([py, "runner.py"], cwd=os.path.join(dst, "tb"), env=env,
                           capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return {"status": "killed", "tests": ["(regression hung)"]}
    out = p.stdout + p.stderr
    if "TESTS=" not in out:
        return {"status": "error", "detail": out[-400:]}
    failed = sorted(set(re.findall(r"\*\*\s+test_\w+\.(\w+)\s+FAIL", out)))
    return {"status": "killed" if failed else "survived", "tests": failed}


def run_sby(dst, sby, task):
    flags = shlex.split(os.environ.get("SBY_FLAGS", ""))
    try:
        p = subprocess.run(["sby", *flags, "-f", sby, task], cwd=os.path.join(dst, "formal"),
                           capture_output=True, text=True, timeout=1800)
    except subprocess.TimeoutExpired:
        return {"status": "error", "detail": "sby timed out"}
    out = p.stdout + p.stderr
    props = re.findall(r"failed assertion \S+?\\?([\w.]+) at", out)
    props = sorted({x.split(".")[-1] for x in props})
    if "DONE (PASS" in out:
        return {"status": "pass"}
    if "DONE (FAIL" in out:
        return {"status": "cex", "props": props}
    if "DONE (UNKNOWN" in out:
        return {"status": "unproven", "props": props}
    return {"status": "error", "detail": out[-400:]}


def evaluate(m):
    mid, fname, find, repl, desc = m
    dst = tempfile.mkdtemp(prefix=f"mut_{mid}_")
    t0 = time.time()
    sby, tasks = FORMAL[fname]
    try:
        make_tree(dst)
        apply(dst, fname, find, repl)
        sim = run_sim(dst)
        formal = {t: run_sby(dst, sby, t) for t in tasks}
    except Exception as e:  # record, don't take the whole campaign down
        sim = {"status": "error", "detail": repr(e)}
        formal = {t: {"status": "error"} for t in tasks}
    finally:
        shutil.rmtree(dst, ignore_errors=True)
    return {"id": mid, "file": fname, "desc": desc, "sim": sim, "formal": formal,
            "secs": time.time() - t0}


def formal_verdict(r):
    prove, bmc = r["formal"].values()
    if bmc["status"] == "cex":
        return "killed", bmc["props"]
    if prove["status"] == "cex":
        return "killed", prove["props"]
    if prove["status"] == "unproven":
        return "proof broken", prove["props"]
    if prove["status"] == "pass" and bmc["status"] == "pass":
        return "survived", []
    return "error", []


def fmt_list(xs, n=3):
    xs = [f"`{x}`" for x in xs]
    return ", ".join(xs[:n]) + (f" +{len(xs) - n}" if len(xs) > n else "")


def report(results):
    lines = [
        "# Mutation testing report",
        "",
        "Generated by `mutation/run_mutation.py`. Each row is a deliberate RTL bug; "
        "**Simulation** lists the cocotb tests that failed, **Formal** the property "
        "violated in the bounded check without helper invariants (`bmc_ext`), or in "
        "the unbounded proof.",
        "",
        "| # | Injected bug | Simulation | Formal |",
        "|---|---|---|---|",
    ]
    tally = {"both": 0, "sim_only": 0, "formal_only": 0, "neither": 0}
    for r in results:
        s = r["sim"]
        sim_k = s["status"] == "killed"
        s_txt = ("killed: " + fmt_list(s["tests"])) if sim_k else s["status"]
        fv, props = formal_verdict(r)
        f_k = fv in ("killed", "proof broken")
        f_txt = f"{fv}: {fmt_list(props)}" if props else fv
        key = ("both" if sim_k and f_k else "sim_only" if sim_k else
               "formal_only" if f_k else "neither")
        tally[key] += 1
        lines.append(f"| {r['id']} | {r['desc']} | {s_txt} | {f_txt} |")
    n = len(results)
    lines += [
        "",
        f"**{n} mutants: {n - tally['neither']} killed** "
        f"({tally['both']} by both, {tally['sim_only']} by simulation only, "
        f"{tally['formal_only']} by formal only), {tally['neither']} survived.",
    ]
    return "\n".join(lines) + "\n", tally


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", nargs="*", help="only these mutant ids")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "mutation_report.md"))
    args = ap.parse_args()

    todo = [m for m in MUTANTS if not args.k or m[0] in args.k]
    results = []
    with cf.ThreadPoolExecutor(args.jobs) as ex:
        for r in ex.map(evaluate, todo):
            fv, _ = formal_verdict(r)
            print(f"{r['id']:>4}  sim={r['sim']['status']:<9} formal={fv:<13} "
                  f"({r['secs']:.0f}s)  {r['desc']}", flush=True)
            results.append(r)

    text, tally = report(results)
    if not args.k:
        with open(args.out, "w") as f:
            f.write(text)
    print()
    print(text)
    sys.exit(1 if tally["neither"] else 0)


if __name__ == "__main__":
    main()
