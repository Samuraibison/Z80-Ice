/*
 * Interface to CLC driver
 *
 * Copyright Debug Innovations Ltd. 2021-2024
 */

// Call these 4 functions in the order below.
// The implementations on each CPU type have further information in the
// function headers about where to find the information in the datasheets.

// Connect a pin to a CLC input channel
// @param[in]  clc       CLC block to use
// @param[in]  channel   CLC input mux channel (0-3)
// @param[in]  source    CLC input mux setting
// @param[in]  gpio_num  Input pin (GPIO_NOT_CONNECTED if source isn't a pin)
void clc_input(uint8_t clc, uint8_t channel, uint8_t source, uint16_t gpio_num);


// Gate the CLC input channels to produce a control signal, there are
// 4 gates on each CLC, each gate producing one control signal.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  gate_num  CLC data gate number (0-3)
// @param[in]  gate_fn   CLC data gate function
// @param[in]  inv_out   If true, the control signal will be inverted
void clc_gate(uint8_t clc, uint8_t gate_num, uint8_t gate_fn, bool inv_out);


// Combine the control signals with the chosen logic cell function.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  logic_fn  CLC data gate function (0-7) + control bits
// @param[in]  inv_out   If true, the final output signal will be inverted
void clc_function(uint8_t clc, uint8_t logic_fn, bool inv_out);


// Connect a CLC output to a pin
// This will also enable the CLC block.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  gpio_num  Output pin (GPIO_NOT_CONNECTED if used internally)
void clc_output(uint8_t clc, uint16_t gpio_num);

