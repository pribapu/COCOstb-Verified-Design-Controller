// Attach the property checker to every spi_master instance. `.*` connects the
// checker's ports to the same-named ports *and internal signals* of the core
// (`state` is named explicitly: it's an enum in the RTL, a plain vector here).
//
// SPI_CORE_ASSUME_INPUTS=1: standalone core proof, the environment contract
// is assumed. =0: the core is inside spi_apb_wrapper and the contract becomes
// an assertion on the wrapper (assume-guarantee).
bind spi_master spi_master_props #(
    .DATA_WIDTH    (DATA_WIDTH),
    .DIV_WIDTH     (DIV_WIDTH),
    .ASSUME_INPUTS   (`SPI_CORE_ASSUME_INPUTS),
    .ENV_MODE_FROZEN (!`SPI_CORE_ASSUME_INPUTS)
) u_props (.*, .state(state));
