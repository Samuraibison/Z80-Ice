/**
 * @brief  Interface to PIC18 interrupt driver
 *
 * Copyright Debug Innovations Ltd. 2018-2021
 */


typedef enum picint_t
{
    INT_TIMER0,
    INT_TIMER5,
    INT_GPIO_PIN,

    // UART RX must be in one block in number order
    INT_UART0_RX,
    INT_UART1_RX,

    // UART TX must be in one block in number order
    INT_UART0_TX,
    INT_UART1_TX,

    // Must be last
    NUM_INT_TYPES
}
picint_t;


// Callback function type
typedef void (*picint_fn_t)(void);


/*
 * @brief Initialise the PIC interrupts
 *        This does not enable interrupts
 */
void picint_init(void);


/*
 * @brief Register a callback for a specific interrupt
 * @param[IN]  source     Hardware interrupt source
 * @param[IN]  callback   Interrupt service routine to call
 */
void picint_register(picint_t source, picint_fn_t callback);

