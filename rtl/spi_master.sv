// -----------------------------------------------------------------------------
// spi_master.sv
//
// Parameterizable SPI master controller supporting all four SPI modes
// (CPOL/CPHA = 0..3), configurable word width, configurable SCLK divider, and
// MSB- or LSB-first bit ordering. Mode / ordering / divider are *runtime*
// inputs latched at the start of each transfer, so a single elaboration
// exercises the entire configuration space -- convenient for constrained-random
// verification and functional coverage.
//
// CPU-side handshake:
//   - Pulse `start` for one clk cycle with `tx_data` and the config inputs valid.
//   - `busy` is high for the duration of the transfer.
//   - `done` pulses for one clk cycle when the transfer completes; `rx_data`
//     is valid on that cycle and held until the next transfer.
//
// SPI timing (standard Motorola SPI):
//   - SCLK idles at CPOL.
//   - CPHA=0: data sampled on the leading edge, shifted on the trailing edge.
//   - CPHA=1: data shifted on the leading edge, sampled on the trailing edge.
//   - SCLK half-period = (clk_div + 1) system-clock cycles.
// -----------------------------------------------------------------------------
`default_nettype none

module spi_master #(
    parameter int DATA_WIDTH = 8,
    parameter int DIV_WIDTH  = 16
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- configuration, latched when `start` is asserted ----
    input  wire                    cpol,       // clock idle polarity
    input  wire                    cpha,       // clock phase
    input  wire                    lsb_first,  // 1 = LSB first, 0 = MSB first
    input  wire [DIV_WIDTH-1:0]    clk_div,    // sclk half-period = clk_div+1 clks

    // ---- CPU-side control / data ----
    input  wire                    start,      // 1-cycle pulse to begin a transfer
    input  wire [DATA_WIDTH-1:0]   tx_data,    // word to transmit
    output reg                     busy,
    output reg                     done,       // 1-cycle pulse at end of transfer
    output reg  [DATA_WIDTH-1:0]   rx_data,    // received word (valid at `done`)

    // ---- SPI bus ----
    output reg                     sclk,
    output reg                     mosi,
    input  wire                    miso,
    output reg                     cs_n
);

    // Total number of SCLK edges (toggles) in one transfer.
    localparam int NEDGES = 2 * DATA_WIDTH;

    typedef enum logic [1:0] {S_IDLE, S_SETUP, S_XFER, S_DONE} state_t;
    state_t state;

    // Latched configuration for the in-flight transfer.
    reg                    cpha_l, lsb_l;
    reg [DIV_WIDTH-1:0]    div_l;

    reg [DATA_WIDTH-1:0]   sh_tx;    // transmit source (indexed by bit_i)
    reg [DATA_WIDTH-1:0]   sh_rx;    // receive accumulator
    reg [DIV_WIDTH-1:0]    div_cnt;  // half-period countdown
    reg [$clog2(NEDGES+1)-1:0] edge_cnt; // 0..NEDGES

    // Current bit position within the word (0 = first bit sent).
    reg [$clog2(DATA_WIDTH+1)-1:0] bit_i;

    // Physical bit index: LSB-first counts up from 0, MSB-first counts down.
    function automatic [$clog2(DATA_WIDTH)-1:0] cur_index
        (input [$clog2(DATA_WIDTH+1)-1:0] bi);
        cur_index = lsb_l ? bi[$clog2(DATA_WIDTH)-1:0]
                          : ($clog2(DATA_WIDTH))'(DATA_WIDTH - 1 - bi);
    endfunction

    wire tick = (div_cnt == '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            busy     <= 1'b0;
            done     <= 1'b0;
            rx_data  <= '0;
            sclk     <= 1'b0;
            mosi     <= 1'b0;
            cs_n     <= 1'b1;
            div_cnt  <= '0;
            edge_cnt <= '0;
            bit_i    <= '0;
            sh_tx    <= '0;
            sh_rx    <= '0;
            cpha_l   <= 1'b0;
            lsb_l    <= 1'b0;
            div_l    <= '0;
        end else begin
            done <= 1'b0; // default: single-cycle pulse

            case (state)
                // -------------------------------------------------------------
                S_IDLE: begin
                    cs_n <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        // Latch configuration for this transfer.
                        cpha_l <= cpha;
                        lsb_l  <= lsb_first;
                        div_l  <= clk_div;

                        sh_tx  <= tx_data;
                        sh_rx  <= '0;
                        bit_i  <= '0;
                        edge_cnt <= '0;
                        div_cnt  <= clk_div;

                        sclk <= cpol;    // idle at CPOL
                        cs_n <= 1'b0;    // assert chip-select
                        busy <= 1'b1;

                        // CPHA=0 needs the first bit valid before the first edge.
                        mosi <= (cpha ? 1'b0
                                      : (lsb_first ? tx_data[0]
                                                   : tx_data[DATA_WIDTH-1]));
                        state <= S_SETUP;
                    end
                end

                // One half-period of CS setup before the first SCLK edge.
                // -------------------------------------------------------------
                S_SETUP: begin
                    if (tick) begin
                        div_cnt <= div_l;
                        state   <= S_XFER;
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                // -------------------------------------------------------------
                S_XFER: begin
                    if (tick) begin
                        div_cnt  <= div_l;
                        sclk     <= ~sclk;          // toggle SCLK
                        edge_cnt <= edge_cnt + 1'b1;

                        // Even edge_cnt (0,2,..) = leading (idle->active).
                        // Odd  edge_cnt (1,3,..) = trailing (active->idle).
                        if (edge_cnt[0] == 1'b0) begin
                            // ---- leading edge ----
                            if (cpha_l == 1'b0)
                                sh_rx[cur_index(bit_i)] <= miso;         // sample
                            else
                                mosi <= sh_tx[cur_index(bit_i)];         // shift out
                        end else begin
                            // ---- trailing edge ----
                            if (cpha_l == 1'b0) begin
                                if (bit_i < $clog2(DATA_WIDTH+1)'(DATA_WIDTH-1)) begin  // shift out
                                    bit_i <= bit_i + 1'b1;
                                    mosi  <= sh_tx[cur_index(bit_i + 1'b1)];
                                end
                            end else begin
                                sh_rx[cur_index(bit_i)] <= miso;         // sample
                                bit_i <= bit_i + 1'b1;
                            end
                        end

                        // Last toggle returns SCLK to idle -> finish.
                        if (edge_cnt == $clog2(NEDGES+1)'(NEDGES-1))
                            state <= S_DONE;
                    end else begin
                        div_cnt <= div_cnt - 1'b1;
                    end
                end

                // -------------------------------------------------------------
                S_DONE: begin
                    cs_n    <= 1'b1;
                    busy    <= 1'b0;
                    rx_data <= sh_rx;
                    done    <= 1'b1;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
