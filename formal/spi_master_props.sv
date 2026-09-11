// -----------------------------------------------------------------------------
// spi_master_props.sv
//
// Formal property set for spi_master, attached with `bind` (see
// spi_master_bind.sv) so the RTL carries no verification code. Proven with
// SymbiYosys (see spi_master.sby) for *every* mode, bit order, divider value,
// data word and MISO pattern -- including inputs changing mid-frame -- not
// just the ones a simulation happened to pick.
//
// Style: the checker keeps registered copies of last cycle's pins (p_*) and
// derives "what happened at the previous clock edge" (f_start, f_tog, ...)
// combinationally from them. Its reference model state (e_*) is therefore
// aligned to exactly the same point in time as the DUT registers it is
// compared against.
//
// Property groups
//   p_*   external protocol / functional properties (what a user cares about)
//   inv_* internal invariants linking the reference model to the RTL state.
//         These are what make the k-induction proof go through (they are also
//         checked, so a wrong invariant fails loudly rather than hiding bugs).
//   c_*   cover points: prove the interesting behaviours are *reachable*, so
//         the assertions can't pass vacuously.
//
// Environment contract (ASSUME_INPUTS=1: assumed; 0: asserted -- used when the
// core sits inside spi_apb_wrapper, turning the wrapper's side of the contract
// into a proof obligation):
//   `start` issued while idle must come >= 1 cycle after reset, not on the
//   `done` cycle, and with `cpol` unchanged from the previous cycle.
// -----------------------------------------------------------------------------
`default_nettype none

module spi_master_props #(
    parameter int DATA_WIDTH    = 8,
    parameter int DIV_WIDTH     = 16,
    parameter bit ASSUME_INPUTS = 1,
    // 1 when the core sits inside spi_apb_wrapper: additionally *assert* the
    // wrapper's guarantee that the mode inputs never change mid-word.
    parameter bit ENV_MODE_FROZEN = 0
) (
    input wire                          clk,
    input wire                          rst_n,
    input wire                          cpol,
    input wire                          cpha,
    input wire                          lsb_first,
    input wire [DIV_WIDTH-1:0]          clk_div,
    input wire                          start,
    input wire [DATA_WIDTH-1:0]         tx_data,
    input wire                          busy,
    input wire                          done,
    input wire [DATA_WIDTH-1:0]         rx_data,
    input wire                          sclk,
    input wire                          mosi,
    input wire                          miso,
    input wire                          cs_n,
    // ---- RTL internals (only used by the inv_* strengthening invariants) ----
    input wire [1:0]                    state,
    input wire                          cpha_l,
    input wire                          lsb_l,
    input wire [DIV_WIDTH-1:0]          div_l,
    input wire [DIV_WIDTH-1:0]          div_cnt,
    input wire [DATA_WIDTH-1:0]         sh_tx,
    input wire [DATA_WIDTH-1:0]         sh_rx,
    input wire [$clog2(2*DATA_WIDTH+1)-1:0] edge_cnt,
    input wire [$clog2(DATA_WIDTH+1)-1:0]   bit_i
);
    localparam int W    = DATA_WIDTH;
    localparam int NE   = 2 * W;
    localparam int ECW  = $clog2(NE + 1) + 1;           // headroom: overflow can't wrap
    localparam int NSW  = $clog2(W + 1) + 1;
    localparam int CYW  = DIV_WIDTH + $clog2(NE + 2) + 2;

    localparam logic [1:0] S_IDLE = 2'd0, S_SETUP = 2'd1, S_XFER = 2'd2, S_DONE = 2'd3;

    // Physical bit index of the n-th bit on the wire.
    function automatic int pos(input logic lsb, input int n);
        return lsb ? n : (W - 1 - n);
    endfunction

    // ---------------------------------------------------------------- history
    reg f_past_valid = 1'b0;
    reg p_rst_n, p_cs_n, p_sclk, p_mosi, p_miso, p_cpol, p_cpha, p_lsb, p_busy, p_done, pp_done;
    reg [DIV_WIDTH-1:0]  p_div;
    reg [DATA_WIDTH-1:0] p_tx, p_rx_data;

    always @(posedge clk) begin
        f_past_valid <= 1'b1;
        p_rst_n <= rst_n;   p_cs_n  <= cs_n;   p_sclk <= sclk;  p_mosi <= mosi;
        p_miso  <= miso;    p_cpol  <= cpol;   p_cpha <= cpha;  p_lsb  <= lsb_first;
        p_busy  <= busy;    p_done  <= done;   pp_done <= p_done;
        p_div   <= clk_div; p_tx    <= tx_data; p_rx_data <= rx_data;
    end

    // ---------------------------------------- what happened at the last edge
    wire f_live  = f_past_valid && rst_n && p_rst_n;        // no reset involved
    wire f_start = f_live &&  !cs_n &&  p_cs_n;             // frame began
    wire f_end   = f_live &&   cs_n && !p_cs_n;             // frame ended
    wire f_tog   = f_live &&  !cs_n && !p_cs_n && (sclk != p_sclk);

    // --------------------------------------------- reference model (e_ = now)
    reg                  q_in, q_cpol, q_cpha, q_lsb;
    reg [DIV_WIDTH-1:0]  q_div;
    reg [DATA_WIDTH-1:0] q_tx, q_rx;
    reg [ECW-1:0]        q_edges;
    reg [NSW-1:0]        q_ns;
    reg [CYW-1:0]        q_cyc, q_gap;

    // Frame configuration: what the DUT sampled on the start edge.
    wire                  e_cpol = f_start ? p_cpol : q_cpol;
    wire                  e_cpha = f_start ? p_cpha : q_cpha;
    wire                  e_lsb  = f_start ? p_lsb  : q_lsb;
    wire [DIV_WIDTH-1:0]  e_div  = f_start ? p_div  : q_div;
    wire [DATA_WIDTH-1:0] e_tx   = f_start ? p_tx   : q_tx;
    wire [CYW-1:0]        e_half = CYW'(e_div) + 1'b1;      // SCLK half-period, clks

    // Edge classification (after a toggle, SCLK at the active level => the
    // toggle was a leading edge).
    wire f_samp  = f_tog && (e_cpha ? (sclk == e_cpol) : (sclk != e_cpol));

    wire [ECW-1:0] e_edges = f_start ? '0 : q_edges + ECW'(f_tog);
    wire [NSW-1:0] e_ns    = f_start ? '0 : q_ns + NSW'(f_samp);
    wire [CYW-1:0] e_cyc   = f_start ? '0 : q_cyc + 1'b1;
    wire [CYW-1:0] e_gap   = (f_start || f_tog) ? '0 : q_gap + 1'b1;
    wire           e_in    = f_start ? 1'b1 : (f_end ? 1'b0 : q_in);

    // MISO bit the DUT captured on this sampling edge goes to position q_ns.
    wire [DATA_WIDTH-1:0] samp_mask = DATA_WIDTH'(1) << pos(e_lsb, int'(q_ns));
    wire [DATA_WIDTH-1:0] e_rx = f_start ? '0
                               : f_samp  ? ((q_rx & ~samp_mask) | (p_miso ? samp_mask : '0))
                                         : q_rx;

    // Cycle-exact latency from the start edge to the `done` edge.
    wire [CYW-1:0] LATENCY = e_half * CYW'(NE + 1) + 1'b1;

    always @(posedge clk) begin
        if (!rst_n) begin
            q_in <= 1'b0; q_edges <= '0; q_ns <= '0; q_cyc <= '0; q_gap <= '0; q_rx <= '0;
        end else begin
            q_in <= e_in; q_edges <= e_edges; q_ns <= e_ns; q_cyc <= e_cyc; q_gap <= e_gap;
            q_rx <= e_rx;
        end
        q_cpol <= e_cpol; q_cpha <= e_cpha; q_lsb <= e_lsb; q_div <= e_div; q_tx <= e_tx;
    end

    // ------------------------------------------------------ environment contract
    wire contract_ok = !(f_past_valid && rst_n && start && !busy)
                     || (p_rst_n && !done && (cpol == p_cpol));

    always @(posedge clk) begin
        if (!f_past_valid) assume (!rst_n);                 // start in reset
        if (ASSUME_INPUTS) a_core_contract: assume (contract_ok);
        else               p_core_contract: assert (contract_ok);
    end

    // ================================================================ PROPERTIES
    always @(posedge clk) if (rst_n) begin
        // ---- handshake -----------------------------------------------------
        p_busy_is_cs:      assert (busy == !cs_n);
        if (ENV_MODE_FROZEN && f_past_valid && state != S_IDLE) begin
            p_env_mode_frozen: assert (cpol == e_cpol && cpha == e_cpha && lsb_first == e_lsb);
        end
        p_done_idle:       assert (!done || (!busy && cs_n));
        if (f_past_valid) begin
            p_done_pulse:  assert (!(done && p_done));
        end
        if (f_live) begin
            p_done_is_end: assert (done == f_end);
            p_rx_stable:   assert (done || rx_data == p_rx_data);

            // ---- SCLK / CS relationship (checker rules SPI-1/2/4) ----------
            if (f_start || f_end) begin
                p_sclk_quiet_on_cs_edge: assert (sclk == p_sclk);
                p_sclk_idle_at_cs_edge:  assert (sclk == e_cpol);
            end
            if (cs_n && p_cs_n && sclk != p_sclk) begin
                p_sclk_tracks_cpol:      assert (sclk == p_cpol);
            end

            // ---- SCLK timing -----------------------------------------------
            if (f_tog) begin
                p_half_period: assert (q_gap + 1'b1 == ((q_edges == 0) ? (e_half << 1) : e_half));
            end
            p_edge_bound:      assert (e_edges <= ECW'(NE));
            p_latency_bound:   assert (!e_in || e_cyc < LATENCY);
            if (f_end) begin
                p_edge_count:  assert (e_edges == ECW'(NE));
                p_sample_count:assert (e_ns == NSW'(W));
            end
            if (done) begin
                p_latency:     assert (e_cyc == LATENCY);
            end

            // ---- data integrity --------------------------------------------
            if (f_samp) begin
                p_mosi_stable_on_sample: assert (mosi == p_mosi);                  // SPI-3
                p_mosi_data:             assert (mosi == e_tx[pos(e_lsb, int'(q_ns))]);
            end
            if (done) begin
                p_rx_correct:            assert (rx_data == e_rx);
            end
        end
    end

    // ================================================ STRENGTHENING INVARIANTS
    // Compiled out for the `bmc_ext` task, so that a bug is attributed to the
    // externally meaningful p_* property it violates, not to an invariant.
`ifndef SPI_PROPS_NO_INV
    wire [DATA_WIDTH-1:0] tx_bit_i  = sh_tx >> pos(lsb_l, int'(bit_i));
    wire [DATA_WIDTH-1:0] tx_bit_im = sh_tx >> pos(lsb_l, int'(bit_i) - 1);
    wire [CYW-1:0]        div_done  = CYW'(div_l) - CYW'(div_cnt);   // clks into this half-period

    always @(posedge clk) if (rst_n && f_past_valid) begin
        inv_in_frame:  assert (e_in == (state != S_IDLE));
        inv_busy:      assert (busy == (state != S_IDLE));
        if (!busy && !p_busy && f_live) begin
            inv_idle_sclk: assert (sclk == p_cpol);
        end
        if (state != S_IDLE) begin
            inv_cfg:     assert (cpha_l == e_cpha && lsb_l == e_lsb && div_l == e_div && sh_tx == e_tx);
            inv_div:     assert (div_cnt <= div_l);
            inv_rx:      assert (sh_rx == e_rx);
            if (cpha_l == 1'b0) begin
                inv_bit0:  assert (bit_i == ((edge_cnt >> 1) < W-1 ? (edge_cnt >> 1) : W-1));
                inv_ns0:   assert (e_ns == NSW'((edge_cnt + 1) >> 1));
                inv_mosi0: assert (mosi == tx_bit_i[0]);
            end else begin
                inv_bit1:  assert (bit_i == (edge_cnt >> 1));
                inv_ns1:   assert (e_ns == NSW'(edge_cnt >> 1));
                inv_mosi1: assert (edge_cnt == 0 ? (mosi == 1'b0)
                                 : edge_cnt[0] ? (mosi == tx_bit_i[0]) : (mosi == tx_bit_im[0]));
            end
        end
        case (state)
            S_SETUP: begin
                inv_setup: assert (edge_cnt == 0 && e_edges == 0 && sclk == e_cpol
                                   && e_gap == div_done && e_cyc == div_done);
            end
            S_XFER: begin
                inv_xfer:  assert (edge_cnt < NE && e_edges == ECW'(edge_cnt)
                                   && sclk == (e_cpol ^ edge_cnt[0]));
                inv_xgap:  assert (e_gap == ((edge_cnt == 0) ? e_half : '0) + div_done);
                inv_xcyc:  assert (e_cyc == e_half * CYW'(edge_cnt + 1) + div_done);
            end
            S_DONE: begin
                inv_done:  assert (edge_cnt == NE && e_edges == ECW'(NE) && sclk == e_cpol
                                   && e_cyc == e_half * CYW'(NE + 1));
            end
            default: ;
        endcase
    end
`endif

    // ==================================================================== COVER
    always @(posedge clk) if (rst_n && f_live) begin
        c_mode0:         cover (done && e_cpol == 0 && e_cpha == 0);
        c_mode1:         cover (done && e_cpol == 0 && e_cpha == 1);
        c_mode2:         cover (done && e_cpol == 1 && e_cpha == 0);
        c_mode3:         cover (done && e_cpol == 1 && e_cpha == 1);
        c_lsb_slow:      cover (done && e_lsb && e_div == 1);
        c_back_to_back:  cover (f_start && pp_done);
        c_cpol_switch:   cover (f_start && e_cpol != q_cpol);   // the bug scenario
    end

endmodule

`default_nettype wire
