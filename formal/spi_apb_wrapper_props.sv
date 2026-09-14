// -----------------------------------------------------------------------------
// spi_apb_wrapper_props.sv
//
// Formal property set for spi_apb_wrapper, attached with `bind`
// (spi_apb_wrapper_bind.sv). Proven with SymbiYosys (spi_apb_wrapper.sby)
// against *any* legal APB4 master: arbitrary register writes, FIFO traffic,
// flushes, CS-hold frames and mode changes at any cycle, plus any MISO data.
//
// The core's own property set (spi_master_props.sv) is bound into the
// instantiated core at the same time with ASSUME_INPUTS=0, so:
//   * every spi_master property is re-proven *in context*, and
//   * spi_master's input contract, which the standalone core proof assumes,
//     becomes an assertion here -- the wrapper must prove it honours it
//     (assume-guarantee).
//
//   p_*   properties        inv_* induction-strengthening invariants
//   c_*   cover points
// -----------------------------------------------------------------------------
`default_nettype none

module spi_apb_wrapper_props #(
    parameter int DATA_WIDTH = 8,
    parameter int DIV_WIDTH  = 16,
    parameter int FIFO_DEPTH = 8,
    parameter int NUM_CS     = 4,
    parameter int APB_AW     = 8
) (
    input wire                     clk,
    input wire                     rst_n,
    input wire                     psel,
    input wire                     penable,
    input wire                     pwrite,
    input wire [APB_AW-1:0]        paddr,
    input wire [31:0]              pwdata,
    input wire [31:0]              prdata,
    input wire                     pready,
    input wire                     pslverr,
    input wire                     irq,
    input wire                     sclk,
    input wire                     mosi,
    input wire                     miso,
    input wire [NUM_CS-1:0]        cs_n,
    // ---- internals ----
    input wire                     reg_valid,
    input wire                     apb_access,
    input wire                     en_r,
    input wire [6:0]               cfg_r,
    input wire [DIV_WIDTH-1:0]     div_r,
    input wire [$clog2(NUM_CS)-1:0] cs_sel_r,
    input wire                     cs_hold_r,
    input wire [2:0]               mode_q,
    input wire [$clog2(NUM_CS)-1:0] sel_q,
    input wire                     mode_upd_q,
    input wire                     mode_load,
    input wire                     cs_pin_q,
    input wire                     frame_req,
    input wire                     spi_busy,
    input wire                     spi_done,
    input wire                     spi_cs_n,
    input wire                     spi_start,
    input wire [DATA_WIDTH-1:0]    spi_tx_data,
    input wire [DATA_WIDTH-1:0]    spi_rx_data,
    input wire [$clog2(FIFO_DEPTH)-1:0]   tx_wptr, tx_rptr, rx_wptr, rx_rptr,
    input wire [$clog2(FIFO_DEPTH+1)-1:0] tx_count, rx_count,
    input wire [DATA_WIDTH-1:0]    tx_mem [0:FIFO_DEPTH-1],
    input wire [DATA_WIDTH-1:0]    rx_mem [0:FIFO_DEPTH-1],
    input wire                     tx_push_ok, tx_pop, rx_push_ok, rx_pop,
    input wire                     tx_flush, rx_flush, tx_apb_push_req,
    input wire                     tx_empty_d, rx_full_d,
    input wire                     tx_dropped_sticky,
    input wire                     irq_done_sticky, irq_txe_sticky, irq_rxf_sticky, irq_ov_sticky
);
    localparam int W    = DATA_WIDTH;
    localparam int PTRW = $clog2(FIFO_DEPTH);
    localparam int CNTW = $clog2(FIFO_DEPTH + 1);
    localparam logic [CNTW-1:0] DEPTH = CNTW'(FIFO_DEPTH);
    localparam logic [NUM_CS-1:0] CS_IDLE = '1;

    // ---------------------------------------------------------------- history
    reg f_past_valid = 1'b0;
    reg p_rst_n = 1'b0, pp_rst_n = 1'b0;
    reg p_psel, p_penable, p_pwrite, p_sclk, p_spi_cs_n, p_spi_done, p_frame_req;
    reg p_cs_pin_q, p_cs_hold_r, p_mode_load, p_err_access, p_tx_pop, p_rx_pop;
    reg [APB_AW-1:0]   p_paddr;
    reg [31:0]         p_pwdata;
    reg [NUM_CS-1:0]   p_cs_n;
    reg [2:0]          p_mode_q;
    reg [$clog2(NUM_CS)-1:0] p_sel_q;
    reg [CNTW-1:0]     p_tx_count, pp_tx_count, p_rx_count, pp_rx_count;
    reg                p_en_r, p_tx_dropped;
    reg [6:0]          p_cfg_r;
    reg [DIV_WIDTH-1:0] p_div_r;
    reg [$clog2(NUM_CS)-1:0] p_cs_sel_r;
    reg p_w1c_done, p_w1c_txe, p_w1c_rxf, p_w1c_ov, p_txd_push_full, p_ov_event;
    reg p_done_irq_d, p_txe_d, p_rxf_d;

    // Address decode *from the spec* (register map in the RTL header), never
    // from the RTL's own decode wires -- a decode bug must not be able to
    // agree with itself.
    wire       spec_mapped = (paddr >> 5) == 0;
    wire       spec_acc    = psel && penable;
    function automatic logic spec_wr(input logic [2:0] idx);
        return spec_acc && pwrite && spec_mapped && paddr[4:2] == idx;
    endfunction
    wire irqstatus_wr = spec_wr(3'd6);
    wire txdata_wr    = spec_wr(3'd4);
    wire tx_flush_wr  = spec_wr(3'd0) && pwdata[1];
    wire rx_flush_wr  = spec_wr(3'd0) && pwdata[2];
    reg  p_txdata_wr, p_tx_flush_wr, p_rx_flush_wr;

    always @(posedge clk) begin
        f_past_valid <= 1'b1;
        p_rst_n <= rst_n;           pp_rst_n <= p_rst_n;
        p_psel <= psel;             p_penable <= penable;     p_pwrite <= pwrite;
        p_paddr <= paddr;           p_pwdata <= pwdata;
        p_sclk <= sclk;             p_cs_n <= cs_n;           p_spi_cs_n <= spi_cs_n;
        p_spi_done <= spi_done;     p_frame_req <= frame_req; p_cs_pin_q <= cs_pin_q;
        p_cs_hold_r <= cs_hold_r;   p_mode_q <= mode_q;       p_sel_q <= sel_q;
        p_mode_load <= mode_load;   p_tx_pop <= tx_pop;       p_rx_pop <= rx_pop;
        p_tx_count <= tx_count;     pp_tx_count <= p_tx_count;
        p_rx_count <= rx_count;     pp_rx_count <= p_rx_count;
        p_en_r <= en_r;  p_cfg_r <= cfg_r;  p_div_r <= div_r;  p_cs_sel_r <= cs_sel_r;
        p_tx_dropped <= tx_dropped_sticky;
        p_err_access <= spec_acc && !spec_mapped;
        p_txdata_wr  <= txdata_wr;
        p_tx_flush_wr <= tx_flush_wr;
        p_rx_flush_wr <= rx_flush_wr;
        p_w1c_done <= irqstatus_wr && pwdata[0];
        p_w1c_txe  <= irqstatus_wr && pwdata[1];
        p_w1c_rxf  <= irqstatus_wr && pwdata[2];
        p_w1c_ov   <= irqstatus_wr && pwdata[3];
        p_txd_push_full <= tx_apb_push_req && tx_count == DEPTH && !tx_pop && !tx_flush;
        p_ov_event <= spi_done && rx_count == DEPTH && !rx_pop && !rx_flush;
        p_done_irq_d <= irq_done_sticky;
        p_txe_d      <= irq_txe_sticky;
        p_rxf_d      <= irq_rxf_sticky;
    end

    wire f_live  = f_past_valid && rst_n && p_rst_n;
    wire f_live2 = f_live && pp_rst_n;

    // ------------------------------------------------- legal APB4 master (env)
    always @(posedge clk) begin
        if (!f_past_valid) assume (!rst_n);
        assume (!penable || psel);
        if (f_past_valid && p_rst_n) begin
            // ACCESS only straight after SETUP; SETUP always followed by
            // ACCESS (PREADY is tied high); address/controls/data held.
            if (penable)                    assume (p_psel && !p_penable);
            if (p_psel && !p_penable)       assume (psel && penable);
            if (penable)                    assume (paddr == p_paddr && pwrite == p_pwrite
                                                    && pwdata == p_pwdata);
        end else begin
            assume (!penable);
        end
    end

    // ------------------------------- marked-word FIFO data-integrity trackers
    // A free (anyseq) bit picks an arbitrary word pushed into each FIFO. The
    // solver may pick *any* word, so proving the marked word pops out intact,
    // in order, proves it for every word.
    (* anyseq *) wire f_tx_mark;
    (* anyseq *) wire f_rx_mark;
    reg              f_tx_armed = 1'b0, f_rx_armed = 1'b0;
    reg [W-1:0]      f_tx_val, f_rx_val;
    reg [CNTW-1:0]   f_tx_ahead, f_rx_ahead;

    always @(posedge clk) begin
        if (!rst_n || tx_flush) f_tx_armed <= 1'b0;
        else if (f_tx_armed) begin
            if (tx_pop) begin
                if (f_tx_ahead == 0) f_tx_armed <= 1'b0;
                else                 f_tx_ahead <= f_tx_ahead - 1'b1;
            end
        end else if (tx_push_ok && f_tx_mark) begin
            f_tx_armed <= 1'b1;
            f_tx_val   <= pwdata[W-1:0];
            f_tx_ahead <= tx_count - CNTW'(tx_pop);
        end

        if (!rst_n || rx_flush) f_rx_armed <= 1'b0;
        else if (f_rx_armed) begin
            if (rx_pop) begin
                if (f_rx_ahead == 0) f_rx_armed <= 1'b0;
                else                 f_rx_ahead <= f_rx_ahead - 1'b1;
            end
        end else if (rx_push_ok && f_rx_mark) begin
            f_rx_armed <= 1'b1;
            f_rx_val   <= spi_rx_data;
            f_rx_ahead <= rx_count - CNTW'(rx_pop);
        end
    end

    // ================================================================ PROPERTIES
    wire cs_any = (cs_n != CS_IDLE);
    wire [NUM_CS-1:0] cs_asserted = ~cs_n;

    always @(posedge clk) if (rst_n) begin
        p_pready:             assert (pready);
        p_pslverr:            assert (pslverr == (spec_acc && !spec_mapped));
        p_fifo_bounds:        assert (tx_count <= DEPTH && rx_count <= DEPTH);
        p_start_only_enabled: assert (!spi_start || (en_r && tx_count != 0));
        p_irq:                assert (irq == |({irq_ov_sticky, irq_rxf_sticky, irq_txe_sticky,
                                                irq_done_sticky} & cfg_r[6:3]));

        // ---- chip-select pins ------------------------------------------
        p_cs_onehot:          assert ((cs_asserted & (cs_asserted - 1'b1)) == '0);
        p_cs_is_selected:     assert (!cs_any || cs_asserted == (NUM_CS'(1) << sel_q));
        p_cs_active_status:   assert (cs_any == cs_pin_q);

        // ---- FIFO data integrity -----------------------------------------
        if (f_tx_armed && tx_pop && f_tx_ahead == 0) begin
            p_tx_fifo_order:  assert (spi_tx_data == f_tx_val);
        end
        if (f_rx_armed && rx_pop && f_rx_ahead == 0) begin
            p_rx_fifo_order:  assert (prdata[W-1:0] == f_rx_val);
        end

        if (f_live) begin
            // ---- an errored access has no side effects -------------------
            if (p_err_access) begin
                p_err_no_reg_change: assert (en_r == p_en_r && cfg_r == p_cfg_r && div_r == p_div_r
                                             && cs_sel_r == p_cs_sel_r && cs_hold_r == p_cs_hold_r);
                p_err_no_push:       assert (tx_count <= p_tx_count && !(tx_dropped_sticky && !p_tx_dropped));
                p_err_no_pop:        assert (rx_count >= p_rx_count);
            end

            // ---- SPI pins: the controller as seen from the bus ------------
            if (cs_n != p_cs_n) begin
                p_pin_sclk_quiet_on_cs_edge: assert (sclk == p_sclk);
            end
            if (!cs_any && p_cs_n == CS_IDLE && sclk != p_sclk) begin
                p_pin_sclk_moves_to_cpol:    assert (sclk == p_mode_q[0]);
            end
            if (cs_any && p_cs_n != CS_IDLE) begin
                p_pin_frame_mode_frozen:     assert (mode_q == p_mode_q && sel_q == p_sel_q);
            end
            if (!spi_cs_n && !p_spi_cs_n) begin
                p_pin_cs_covers_word:        assert (cs_pin_q);
            end
            if (cs_hold_r && p_cs_hold_r && p_cs_pin_q) begin
                p_pin_hold_keeps_cs:         assert (cs_pin_q);
            end
            if (!p_frame_req) begin
                p_pin_release:               assert (!cs_pin_q);
            end

            // ---- FIFO bookkeeping --------------------------------------
            if (p_txdata_wr && p_tx_count < DEPTH) begin
                p_tx_accept:   assert (tx_count == p_tx_count + 1'b1 - CNTW'(p_tx_pop));
            end
            if (p_spi_done && p_rx_count < DEPTH && !p_rx_flush_wr) begin
                p_rx_accept:   assert (rx_count == p_rx_count + 1'b1 - CNTW'(p_rx_pop));
            end
            if (p_tx_flush_wr) begin
                p_tx_flush_empties: assert (tx_count == 0);
            end
            if (p_rx_flush_wr) begin
                p_rx_flush_empties: assert (rx_count == 0);
            end

            // ---- sticky status / interrupt sources -----------------------
            p_drop_flag:     assert (!p_txd_push_full || (tx_dropped_sticky && tx_count == p_tx_count));
            p_overrun_flag:  assert (!p_ov_event || (irq_ov_sticky && rx_count == p_rx_count));
            p_done_irq_set:  assert (!p_spi_done || irq_done_sticky);
            p_done_irq_only: assert (!(irq_done_sticky && !p_done_irq_d) || p_spi_done);
            p_done_irq_w1c:  assert (!(p_w1c_done && !p_spi_done) || !irq_done_sticky);
            if (f_live2) begin
                // TX_EMPTY fires exactly when the TX FIFO *becomes* empty,
                // RX_FULL exactly when the RX FIFO *becomes* full.
                p_txe_irq_only:  assert (!(irq_txe_sticky && !p_txe_d) || (p_tx_count == 0 && pp_tx_count != 0));
                p_txe_irq_set:   assert (!(p_tx_count == 0 && pp_tx_count != 0 && !p_w1c_txe) || irq_txe_sticky);
                p_rxf_irq_only:  assert (!(irq_rxf_sticky && !p_rxf_d) || (p_rx_count == DEPTH && pp_rx_count != DEPTH));
                p_rxf_irq_set:   assert (!(p_rx_count == DEPTH && pp_rx_count != DEPTH && !p_w1c_rxf) || irq_rxf_sticky);
            end
        end
    end

    // ================================================ STRENGTHENING INVARIANTS
`ifndef SPI_PROPS_NO_INV
    always @(posedge clk) if (rst_n) begin
        inv_tx_ptrs:   assert (tx_wptr == PTRW'(tx_rptr + tx_count));
        inv_rx_ptrs:   assert (rx_wptr == PTRW'(rx_rptr + rx_count));
        if (f_tx_armed) begin
            inv_tx_mark: assert (f_tx_ahead < tx_count && tx_mem[PTRW'(tx_rptr + f_tx_ahead)] == f_tx_val);
        end
        if (f_rx_armed) begin
            inv_rx_mark: assert (f_rx_ahead < rx_count && rx_mem[PTRW'(rx_rptr + f_rx_ahead)] == f_rx_val);
        end
        inv_cs_regs:   assert (cs_n == (cs_pin_q ? ~(NUM_CS'(1) << sel_q) : CS_IDLE));
        if (f_past_valid && p_rst_n) begin
            inv_edge_d:   assert (tx_empty_d == (p_tx_count == 0) && rx_full_d == (p_rx_count == DEPTH));
            inv_upd:      assert (mode_upd_q == p_mode_load);
        end
    end
`endif

    // ==================================================================== COVER
    reg [2:0] f_words_in_frame;
    reg       f_had_frame, f_last_cpol;
    wire      f_frame_start = f_live && cs_any && p_cs_n == CS_IDLE;
    always @(posedge clk) begin
        if (!rst_n || !cs_pin_q) f_words_in_frame <= '0;
        else if (spi_done && f_words_in_frame != '1) f_words_in_frame <= f_words_in_frame + 1'b1;
        if (!rst_n) f_had_frame <= 1'b0;
        else if (f_frame_start) begin
            f_had_frame <= 1'b1;
            f_last_cpol <= mode_q[0];
        end
    end

    always @(posedge clk) if (rst_n && f_live) begin
        c_hold_two_words:   cover (f_words_in_frame == 2 && cs_pin_q);
        c_tx_full:          cover (tx_count == DEPTH);
        c_rx_overrun:       cover (irq_ov_sticky);
        c_cpol_switch:      cover (f_frame_start && f_had_frame && mode_q[0] != f_last_cpol);
        c_cs3_word:         cover (spi_done && !cs_n[3]);
        c_tx_order_checked: cover (f_tx_armed && tx_pop && f_tx_ahead == 0 && tx_count > 1);
        c_rx_order_checked: cover (f_rx_armed && rx_pop && f_rx_ahead == 0);
    end

endmodule

`default_nettype wire
