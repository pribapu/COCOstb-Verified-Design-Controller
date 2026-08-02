// -----------------------------------------------------------------------------
// spi_apb_wrapper.sv
//
// APB4 register-mapped wrapper around `spi_master`: adds TX/RX FIFOs, a
// hardware auto-transfer engine that drains the TX FIFO whenever the core is
// idle, and a maskable interrupt. The `spi_master` core itself is instantiated
// unmodified.
//
// Register map (byte offsets, 32-bit registers, APB_AW-bit byte address):
//   0x00 CTRL       RW   [0] EN            auto-transfer engine enable
//                        [1] TX_FLUSH      W1P: pulse to clear TX FIFO
//                        [2] RX_FLUSH      W1P: pulse to clear RX FIFO
//   0x04 CONFIG     RW   [0] CPOL  [1] CPHA  [2] LSB_FIRST
//                        [3] IRQ_EN_DONE [4] IRQ_EN_TX_EMPTY
//                        [5] IRQ_EN_RX_FULL [6] IRQ_EN_OVERRUN
//                        Only takes effect on the *next* transfer, matching
//                        spi_master's own start-time config latch.
//   0x08 DIVIDER    RW   [DIV_WIDTH-1:0] -> spi_master.clk_div
//   0x0C STATUS     RO/W1C(bit7)
//                        [0] BUSY      [2] TX_EMPTY  [3] TX_FULL
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
// -----------------------------------------------------------------------------
`default_nettype none

module spi_apb_wrapper #(
    parameter int DATA_WIDTH = 8,
    parameter int DIV_WIDTH  = 16,
    parameter int FIFO_DEPTH = 8,      // must be a power of 2
    parameter int APB_AW     = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- APB4 slave ----
    input  wire                 psel,
    input  wire                 penable,
    input  wire                 pwrite,
    input  wire [APB_AW-1:0]    paddr,
    input  wire [31:0]          pwdata,
    output reg  [31:0]          prdata,
    output wire                 pready,
    output reg                  pslverr,

    output wire                 irq,

    // ---- SPI bus (pass-through to the instantiated spi_master) ----
    output wire                 sclk,
    output wire                 mosi,
    input  wire                 miso,
    output wire                 cs_n
);

    localparam int PTRW  = $clog2(FIFO_DEPTH);
    localparam int CNTW  = $clog2(FIFO_DEPTH + 1);

    localparam [2:0] REG_CTRL      = 3'd0;  // 0x00
    localparam [2:0] REG_CONFIG    = 3'd1;  // 0x04
    localparam [2:0] REG_DIVIDER   = 3'd2;  // 0x08
    localparam [2:0] REG_STATUS    = 3'd3;  // 0x0C
    localparam [2:0] REG_TXDATA    = 3'd4;  // 0x10
    localparam [2:0] REG_RXDATA    = 3'd5;  // 0x14
    localparam [2:0] REG_IRQSTATUS = 3'd6;  // 0x18

    wire [2:0] reg_sel   = paddr[4:2];
    wire       reg_valid = (paddr[APB_AW-1:5] == '0) && (reg_sel <= REG_IRQSTATUS);

    wire apb_access = psel && penable;
    wire apb_write  = apb_access && pwrite;
    wire apb_read   = apb_access && !pwrite;

    assign pready = 1'b1;

    // ---- stored registers ----
    reg              en_r;
    reg [6:0]        cfg_r;      // [0]cpol [1]cpha [2]lsb [6:3]irq_en
    reg [DIV_WIDTH-1:0] div_r;

    wire ctrl_wr   = apb_write && (reg_sel == REG_CTRL);
    wire config_wr = apb_write && (reg_sel == REG_CONFIG);
    wire div_wr    = apb_write && (reg_sel == REG_DIVIDER);
    wire tx_flush  = ctrl_wr && pwdata[1];
    wire rx_flush  = ctrl_wr && pwdata[2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_r  <= 1'b0;
            cfg_r <= 7'b0;
            div_r <= '0;
        end else begin
            if (ctrl_wr)   en_r  <= pwdata[0];
            if (config_wr) cfg_r <= pwdata[6:0];
            if (div_wr)    div_r <= pwdata[DIV_WIDTH-1:0];
        end
    end

    wire cpol      = cfg_r[0];
    wire cpha      = cfg_r[1];
    wire lsb_first = cfg_r[2];
    wire [3:0] irq_en = cfg_r[6:3];   // {ov,rxf,txe,done}

    // ---- spi_master core ----
    wire                  spi_busy, spi_done;
    wire [DATA_WIDTH-1:0] spi_rx_data;
    wire                  spi_start;
    wire [DATA_WIDTH-1:0] spi_tx_data;

    spi_master #(
        .DATA_WIDTH (DATA_WIDTH),
        .DIV_WIDTH  (DIV_WIDTH)
    ) u_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpol       (cpol),
        .cpha       (cpha),
        .lsb_first  (lsb_first),
        .clk_div    (div_r),
        .start      (spi_start),
        .tx_data    (spi_tx_data),
        .busy       (spi_busy),
        .done       (spi_done),
        .rx_data    (spi_rx_data),
        .sclk       (sclk),
        .mosi       (mosi),
        .miso       (miso),
        .cs_n       (cs_n)
    );

    // spi_master's `busy` is high through S_SETUP/S_XFER/S_DONE and low only
    // in S_IDLE, so `!spi_busy` alone already means "FSM really is in
    // S_IDLE" (true even on the cycle `done` pulses, since `done` and the
    // return to S_IDLE become visible on the same edge). Also excluding
    // `spi_done` costs one extra idle cycle between back-to-back transfers
    // but keeps the "is it safe to fire `start`" condition obviously
    // conservative without having to rely on that same-edge timing.
    wire spi_idle = !spi_busy && !spi_done;

    // ---- TX FIFO ----
    reg [DATA_WIDTH-1:0] tx_mem [0:FIFO_DEPTH-1];
    reg [PTRW-1:0]       tx_wptr, tx_rptr;
    reg [CNTW-1:0]       tx_count;

    wire tx_apb_push_req = apb_write && (reg_sel == REG_TXDATA);
    wire eng_fire         = en_r && (tx_count != 0) && spi_idle;
    wire tx_pop           = eng_fire;
    wire tx_push_ok        = tx_apb_push_req && ((tx_count < FIFO_DEPTH) || tx_pop);
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
            if (tx_push_ok) begin
                tx_mem[tx_wptr] <= pwdata[DATA_WIDTH-1:0];
                tx_wptr <= tx_wptr + 1'b1;
            end
            if (tx_pop) tx_rptr <= tx_rptr + 1'b1;
            tx_count <= tx_count + (tx_push_ok ? 1'b1 : 1'b0)
                                  - (tx_pop     ? 1'b1 : 1'b0);
        end
    end

    // ---- RX FIFO ----
    reg [DATA_WIDTH-1:0] rx_mem [0:FIFO_DEPTH-1];
    reg [PTRW-1:0]       rx_wptr, rx_rptr;
    reg [CNTW-1:0]       rx_count;

    wire rx_apb_pop_req   = apb_read && (reg_sel == REG_RXDATA);
    wire rx_pop            = rx_apb_pop_req && (rx_count != 0);
    wire rx_push_req       = spi_done;
    wire rx_push_ok         = rx_push_req && ((rx_count < FIFO_DEPTH) || rx_pop);
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
            if (rx_push_ok) begin
                rx_mem[rx_wptr] <= spi_rx_data;
                rx_wptr <= rx_wptr + 1'b1;
            end
            if (rx_pop) rx_rptr <= rx_rptr + 1'b1;
            rx_count <= rx_count + (rx_push_ok ? 1'b1 : 1'b0)
                                  - (rx_pop      ? 1'b1 : 1'b0);
        end
    end

    wire tx_empty = (tx_count == 0);
    wire tx_full  = (tx_count == FIFO_DEPTH);
    wire rx_empty = (rx_count == 0);
    wire rx_full  = (rx_count == FIFO_DEPTH);

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

    wire status_wr      = apb_write && (reg_sel == REG_STATUS);
    wire irqstatus_wr   = apb_write && (reg_sel == REG_IRQSTATUS);

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
                                      1'b0, status_busy};               // [1:0]
            REG_TXDATA:    prdata = 32'h0;
            REG_RXDATA:    prdata = {{(32 - DATA_WIDTH){1'b0}}, rx_mem[rx_rptr]};
            REG_IRQSTATUS: prdata = {28'b0, irq_status};
            default:       prdata = 32'h0;
        endcase
    end

endmodule

`default_nettype wire
