// Attach the wrapper property checker. `.*` reaches the wrapper's ports and
// internal signals (including the FIFO memories) by name.
bind spi_apb_wrapper spi_apb_wrapper_props #(
    .DATA_WIDTH (DATA_WIDTH),
    .DIV_WIDTH  (DIV_WIDTH),
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS),
    .APB_AW     (APB_AW)
) u_wprops (.*);
