// -----------------------------------------------------------------------------
// spi_apb_wrapper.sv
//
// APB4 register-mapped wrapper around `spi_master`: adds TX/RX FIFOs, a
// hardware auto-transfer engine that drains the TX FIFO whenever the core is
// idle, a maskable interrupt, NUM_CS chip-select lines, and a CS-hold mode for
// multi-word frames (flash, ADCs, ... anything whose command spans more than
// one word). The `spi_master` core itself is instantiated unmodified.
//
// Register map (byte offsets, 32-bit registers, APB_AW-bit byte address):
//   0x00 CTRL       RW   [0] EN            auto-transfer engine enable
//                        [1] TX_FLUSH      W1P: pulse to clear TX FIFO
//                        [2] RX_FLUSH      W1P: pulse to clear RX FIFO
//   0x04 CONFIG     RW   [0] CPOL  [1] CPHA  [2] LSB_FIRST
//                        [3] IRQ_EN_DONE [4] IRQ_EN_TX_EMPTY
//                        [5] IRQ_EN_RX_FULL [6] IRQ_EN_OVERRUN
//                        CPOL/CPHA/LSB_FIRST take effect at the next frame
//                        boundary (never inside a CS-held frame).
//   0x08 DIVIDER    RW   [DIV_WIDTH-1:0] -> spi_master.clk_div (per word)
//   0x0C STATUS     RO/W1C(bit7)
//                        [0] BUSY      [1] CS_ACTIVE (a CS pin is asserted)
//                        [2] TX_EMPTY  [3] TX_FULL
//                        [4] RX_EMPTY  [5] RX_FULL   [6] RX_OVERRUN (mirror)
//                        [7] TX_DROPPED (sticky, write 1 to clear)
//                        [15:8] TX_COUNT  [23:16] RX_COUNT
//   0x10 TXDATA     WO   write pushes a word into the TX FIFO (dropped + sets
//                        TX_DROPPED if full)
//   0x14 RXDATA     RO   read pops the oldest word from the RX FIFO (returns 0
//                        and does not underflow if empty)
//   0x18 IRQ_STATUS RO/W1C
//                        [0] DONE  [1] TX_EMPTY  [2] RX_FULL  [3] OVERRUN
//                        irq = |(IRQ_STATUS[3:0] & CONFIG[6:3])
//   0x1C CS_CTRL    RW   [CSW-1:0] CS_SEL  chip-select line used by the engine
//                        [8]       CS_HOLD keep CS_SEL asserted between words;
//                                  clear it to end the frame. Takes effect at
//                                  the next frame boundary, like CS_SEL.
//
// Frame / pin timing (all CS pins are registered, glitch-free):
//   * A frame is: CS_HOLD set, or a word in flight. The CS pin asserts one clk
//     after the frame starts and deasserts one clk after it ends.
//   * The mode (CPOL/CPHA/LSB) and CS_SEL applied to the bus are captured only
//     while no frame is active, so SCLK can never move inside a frame and the
//     selected line can never change under an asserted CS.
//   * The engine never starts a word on the cycle the applied mode changes, or
//     the cycle after -- spi_master tracks CPOL while idle and needs one cycle
//     to settle SCLK before `start` (its documented input contract, which the
//     formal proof checks as an assertion on this wrapper).
// -----------------------------------------------------------------------------
`default_nettype none

module spi_apb_wrapper #(
    parameter int DATA_WIDTH = 8,
    parameter int DIV_WIDTH  = 16,
    parameter int FIFO_DEPTH = 8,      // must be a power of 2
    parameter int NUM_CS     = 4,      // must be a power of 2, >= 2
    parameter int APB_AW     = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- APB4 slave ----
    input  wire                 psel,
    input  wire                 penable,
    input  wire                 pwrite,
    // Word-aligned 32-bit registers: paddr[1:0] and the pwdata bits no
    // register implements are legitimately unused.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [APB_AW-1:0]    paddr,
    input  wire [31:0]          pwdata,
    /* verilator lint_on UNUSEDSIGNAL */
    output reg  [31:0]          prdata,
    output wire                 pready,
    output reg                  pslverr,

    output wire                 irq,

    // ---- SPI bus ----
    output wire                 sclk,
    output wire                 mosi,
    input  wire                 miso,
    output reg  [NUM_CS-1:0]    cs_n
);

    localparam int PTRW  = $clog2(FIFO_DEPTH);
    localparam int CNTW  = $clog2(FIFO_DEPTH + 1);
    localparam int CSW   = $clog2(NUM_CS);

    localparam [2:0] REG_CTRL      = 3'd0;  // 0x00
    localparam [2:0] REG_CONFIG    = 3'd1;  // 0x04
    localparam [2:0] REG_DIVIDER   = 3'd2;  // 0x08
    localparam [2:0] REG_STATUS    = 3'd3;  // 0x0C
    localparam [2:0] REG_TXDATA    = 3'd4;  // 0x10
    localparam [2:0] REG_RXDATA    = 3'd5;  // 0x14
    localparam [2:0] REG_IRQSTATUS = 3'd6;  // 0x18
    localparam [2:0] REG_CSCTRL    = 3'd7;  // 0x1C

    wire [2:0] reg_sel   = paddr[4:2];
    wire       reg_valid = (paddr[APB_AW-1:5] == '0);

    wire apb_access = psel && penable;
    wire apb_write  = apb_access && pwrite;
    wire apb_read   = apb_access && !pwrite;

    assign pready = 1'b1;

    // ---- stored registers ----
    reg              en_r;
    reg [6:0]        cfg_r;      // [0]cpol [1]cpha [2]lsb [6:3]irq_en
    reg [DIV_WIDTH-1:0] div_r;
    reg [CSW-1:0]    cs_sel_r;
    reg              cs_hold_r;

    wire ctrl_wr   = apb_write && reg_valid && (reg_sel == REG_CTRL);
    wire config_wr = apb_write && reg_valid && (reg_sel == REG_CONFIG);
    wire div_wr    = apb_write && reg_valid && (reg_sel == REG_DIVIDER);
    wire csctrl_wr = apb_write && reg_valid && (reg_sel == REG_CSCTRL);
    wire tx_flush  = ctrl_wr && pwdata[1];
    wire rx_flush  = ctrl_wr && pwdata[2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_r      <= 1'b0;
            cfg_r     <= 7'b0;
            div_r     <= '0;
            cs_sel_r  <= '0;
            cs_hold_r <= 1'b0;
        end else begin
            if (ctrl_wr)   en_r  <= pwdata[0];
            if (config_wr) cfg_r <= pwdata[6:0];
            if (div_wr)    div_r <= pwdata[DIV_WIDTH-1:0];
            if (csctrl_wr) begin
                cs_sel_r  <= pwdata[CSW-1:0];
                cs_hold_r <= pwdata[8];
            end
        end
    end

    wire [3:0] irq_en = cfg_r[6:3];   // {ov,rxf,txe,done}

    // ---- spi_master core ----
    wire                  spi_busy, spi_done, spi_cs_n;
    wire [DATA_WIDTH-1:0] spi_rx_data;
    wire                  spi_start;
    wire [DATA_WIDTH-1:0] spi_tx_data;
    reg  [2:0]            mode_q;     // applied {lsb, cpha, cpol}

    spi_master #(
        .DATA_WIDTH (DATA_WIDTH),
        .DIV_WIDTH  (DIV_WIDTH)
    ) u_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpol       (mode_q[0]),
        .cpha       (mode_q[1]),
        .lsb_first  (mode_q[2]),
        .clk_div    (div_r),
        .start      (spi_start),
        .tx_data    (spi_tx_data),
        .busy       (spi_busy),
        .done       (spi_done),
        .rx_data    (spi_rx_data),
        .sclk       (sclk),
        .mosi       (mosi),
        .miso       (miso),
        .cs_n       (spi_cs_n)
    );

    // `!spi_busy` alone means the core FSM is in S_IDLE. Also excluding
    // `spi_done` is *required*, not just conservative: on the done cycle the
    // core has not yet had an idle cycle to re-settle SCLK to CPOL, so firing
    // `start` there would break spi_master's input contract.
    wire spi_idle = !spi_busy && !spi_done;

    // ---- frame tracking, applied mode / chip-select ----
    reg              cs_pin_q;        // registered "a CS pin is asserted"
    reg [CSW-1:0]    sel_q;           // applied chip-select index
    reg              mode_upd_q;      // mode_q/sel_q changed at the last edge

    wire frame_req  = cs_hold_r || !spi_cs_n;           // a frame should be active
    wire frame_idle = !frame_req && !cs_pin_q;          // nothing on the bus at all
    wire mode_load  = frame_idle && ({cfg_r[2:0], cs_sel_r} != {mode_q, sel_q});

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_q     <= 3'b0;
            sel_q      <= '0;
            mode_upd_q <= 1'b0;
            cs_pin_q   <= 1'b0;
            cs_n       <= '1;
        end else begin
            mode_upd_q <= mode_load;
            if (mode_load) begin
                mode_q <= cfg_r[2:0];
                sel_q  <= cs_sel_r;
            end
            // Assert only once the applied mode has had a cycle to reach SCLK
            // (so SCLK can't move on the CS edge); deassert immediately.
            cs_pin_q <= frame_req && (cs_pin_q || !mode_upd_q);
            for (int i = 0; i < NUM_CS; i++)
                cs_n[i] <= !(frame_req && (cs_pin_q || !mode_upd_q) && (sel_q == CSW'(i)));
        end
    end

    // ---- TX FIFO ----
    reg [DATA_WIDTH-1:0] tx_mem [0:FIFO_DEPTH-1];
    reg [PTRW-1:0]       tx_wptr, tx_rptr;
    reg [CNTW-1:0]       tx_count;

    wire tx_apb_push_req = apb_write && reg_valid && (reg_sel == REG_TXDATA);
    wire eng_fire         = en_r && (tx_count != 0) && spi_idle && !mode_load && !mode_upd_q;
    wire tx_pop           = eng_fire;
    wire tx_push_ok        = tx_apb_push_req && ((tx_count < CNTW'(FIFO_DEPTH)) || tx_pop);
    wire tx_dropped_event  = tx_apb_push_req && !tx_push_ok;

    assign spi_start   = eng_fire;
    assign spi_tx_data = tx_mem[tx_rptr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_wptr  <= '0;
            tx_rptr  <= '0;
            tx_count <= '0;
        end else if (tx_flush) begin
            tx_wptr  <= '0;
            tx_rptr  <= '0;
            tx_count <= '0;
        end else begin
            if (tx_push_ok) tx_wptr <= tx_wptr + 1'b1;
            if (tx_pop)     tx_rptr <= tx_rptr + 1'b1;
            tx_count <= tx_count + (tx_push_ok ? CNTW'(1) : CNTW'(0))
                                  - (tx_pop     ? CNTW'(1) : CNTW'(0));
        end
    end

    // FIFO storage is plain RAM: no reset, kept out of the async-reset block
    // so synthesis can map it to distributed / block RAM.
    always_ff @(posedge clk) begin
        if (tx_push_ok && !tx_flush) tx_mem[tx_wptr] <= pwdata[DATA_WIDTH-1:0];
    end

    // ---- RX FIFO ----
    reg [DATA_WIDTH-1:0] rx_mem [0:FIFO_DEPTH-1];
    reg [PTRW-1:0]       rx_wptr, rx_rptr;
    reg [CNTW-1:0]       rx_count;

    wire rx_apb_pop_req   = apb_read && reg_valid && (reg_sel == REG_RXDATA);
    wire rx_pop            = rx_apb_pop_req && (rx_count != 0);
    wire rx_push_req       = spi_done;
    wire rx_push_ok         = rx_push_req && ((rx_count < CNTW'(FIFO_DEPTH)) || rx_pop);
    wire rx_overrun_event   = rx_push_req && !rx_push_ok;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wptr  <= '0;
            rx_rptr  <= '0;
            rx_count <= '0;
        end else if (rx_flush) begin
            rx_wptr  <= '0;
            rx_rptr  <= '0;
            rx_count <= '0;
        end else begin
            if (rx_push_ok) rx_wptr <= rx_wptr + 1'b1;
            if (rx_pop)     rx_rptr <= rx_rptr + 1'b1;
            rx_count <= rx_count + (rx_push_ok ? CNTW'(1) : CNTW'(0))
                                  - (rx_pop      ? CNTW'(1) : CNTW'(0));
        end
    end

    always_ff @(posedge clk) begin
        if (rx_push_ok && !rx_flush) rx_mem[rx_wptr] <= spi_rx_data;
    end

    wire tx_empty = (tx_count == 0);
    wire tx_full  = (tx_count == CNTW'(FIFO_DEPTH));
    wire rx_empty = (rx_count == 0);
    wire rx_full  = (rx_count == CNTW'(FIFO_DEPTH));

    // The RX-FIFO push triggered by `spi_done` commits one cycle after
    // `spi_busy` first reads 0 (both are registered off the same event, but
    // the FIFO write is behind an extra flop stage). Extending BUSY through
    // the `done` cycle means software never observes "not busy" before the
    // just-completed transfer's byte (and any resulting overrun) is already
    // visible in STATUS/RXDATA.
    wire status_busy = spi_busy || spi_done;

    // ---- sticky event bits (TX_DROPPED, and the four IRQ_STATUS sources) ----
    reg tx_dropped_sticky;
    reg irq_done_sticky, irq_txe_sticky, irq_rxf_sticky, irq_ov_sticky;
    reg tx_empty_d, rx_full_d;

    wire status_wr      = apb_write && reg_valid && (reg_sel == REG_STATUS);
    wire irqstatus_wr   = apb_write && reg_valid && (reg_sel == REG_IRQSTATUS);

    wire irq_txe_event = tx_empty && !tx_empty_d;   // rising edge -> just drained
    wire irq_rxf_event = rx_full  && !rx_full_d;    // rising edge -> just filled

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_dropped_sticky <= 1'b0;
            irq_done_sticky   <= 1'b0;
            irq_txe_sticky    <= 1'b0;
            irq_rxf_sticky    <= 1'b0;
            irq_ov_sticky     <= 1'b0;
            tx_empty_d        <= 1'b1;
            rx_full_d         <= 1'b0;
        end else begin
            tx_empty_d <= tx_empty;
            rx_full_d  <= rx_full;

            tx_dropped_sticky <= (tx_dropped_sticky && !(status_wr && pwdata[7]))
                                  || tx_dropped_event;
            irq_done_sticky <= (irq_done_sticky && !(irqstatus_wr && pwdata[0]))
                                || spi_done;
            irq_txe_sticky <= (irq_txe_sticky && !(irqstatus_wr && pwdata[1]))
                                || irq_txe_event;
            irq_rxf_sticky <= (irq_rxf_sticky && !(irqstatus_wr && pwdata[2]))
                                || irq_rxf_event;
            irq_ov_sticky <= (irq_ov_sticky && !(irqstatus_wr && pwdata[3]))
                                || rx_overrun_event;
        end
    end

    wire [3:0] irq_status = {irq_ov_sticky, irq_rxf_sticky, irq_txe_sticky, irq_done_sticky};
    assign irq = |(irq_status & irq_en);

    // ---- read mux ----
    always_comb begin
        prdata  = 32'h0;
        pslverr = apb_access && !reg_valid;
        case (reg_sel)
            REG_CTRL:      prdata = {31'b0, en_r};
            REG_CONFIG:    prdata = {25'b0, cfg_r};
            REG_DIVIDER:   prdata = {{(32 - DIV_WIDTH){1'b0}}, div_r};
            REG_STATUS:    prdata = {8'b0,
                                      {{(8 - CNTW){1'b0}}, rx_count},   // [23:16] RX_COUNT
                                      {{(8 - CNTW){1'b0}}, tx_count},   // [15:8]  TX_COUNT
                                      tx_dropped_sticky,                // [7]
                                      irq_ov_sticky,                    // [6]
                                      rx_full, rx_empty,                // [5:4]
                                      tx_full, tx_empty,                // [3:2]
                                      cs_pin_q, status_busy};           // [1:0]
            REG_TXDATA:    prdata = 32'h0;
            // rx_mem is never reset (it's plain RAM); guard against reading
            // an unwritten (X in simulation) slot when the FIFO is empty.
            REG_RXDATA:    prdata = rx_empty ? 32'h0
                                     : {{(32 - DATA_WIDTH){1'b0}}, rx_mem[rx_rptr]};
            REG_IRQSTATUS: prdata = {28'b0, irq_status};
            REG_CSCTRL:    prdata = {23'b0, cs_hold_r, {(8 - CSW){1'b0}}, cs_sel_r};
            default:       prdata = 32'h0;
        endcase
        if (!reg_valid) prdata = 32'h0;
    end

endmodule

`default_nettype wire
