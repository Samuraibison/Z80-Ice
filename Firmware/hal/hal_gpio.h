/*
 * Embedded code hardware abstraction layer API
 *    - GPIO and ADC/DAC access functions
 *
 * To maintain compatibility across all MCU types, a standard numbering scheme
 * is used to describe GPIO pins.  On chips with numbered ports/pins we use a
 * 16-bit value with the top byte being the port and the bottom byte is the pin
 * e.g. 0x0102 is port 1, pin 2.  On chips with lettered ports, A is port 0
 * e.g. 0x0102 is port B, pin 2.  The same scheme is used for analog pins so
 * there is no need to know how the ADC channels map to GPIO pins, it is the
 * responsibility of the HAL code to deal with it.
 *
 * Functions are also provided to set/get/configure whole (digital) ports.
 * These are designed to be used for parallel interfaces and should not be used
 * to mop up multiple GPIO pin assignments as that defeats portability between
 * CPUs with different sized ports.  More complex functions such as interrupts
 * are only available for individual pins.
 *
 * A project should have a hal_config.h file that defines the pins used for
 * each purpose and an emul_config.h that defines what range of ports/pins
 * the target MCU has.  These 2 files should be contained within the project
 * folder and included in the project path so that the generic HAL code picks
 * up the correct file for each different project.  For example if you write
 * #define LED  0x0102  in hal_config.h, then when you want the LED pin to
 * go high, you can say gpio_set(LED, HIGH); and if you change the pin or
 * have two boards with different pins, the only definintion of the GPIO
 * pins are contained within hal_config.h.  Porting to a different MCU just
 * requires a new hal_config.h and building the code with the HAL files for
 * the new MCU.
 *
 * Copyright Debug Innovations Ltd. 2016-2022
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


/* This value is used to indicate to the gpio functions that a pin
 * isn't to be used.  In other words, if you pass this value to
 * a gpio function, it will do nothing.  In hal_config.h we can use
 * this value to indicate that, for a particular PCB, a certain
 * function is not available and this means we don't need #ifdef's
 * for each board type scattered throughout the code.
 */
#define GPIO_NOT_CONNECTED   (0xFFFF)

/* HIGH/LOW definition for use with GPIO set/get */
#define LOW                  (0)
#define HIGH                 (1)

/* GPIO pin modes for gpio_configure_pin()
 * OR these together to describe the required pin properties
 */
/* Pin direction */
#define PIN_DIR_IN           (0x00)
#define PIN_DIR_OUT          (0x01)
#define PIN_OPEN_DRAIN       (0x02)
#define PIN_DIR_MASK         (0x01)

/* Analog or digital mode */
#define PIN_MODE_DIGITAL     (0x00)
#define PIN_MODE_ANALOG      (0x10)

/* Initial value for a digital output pin, ignored for inputs and analog pins
 * This will be applied before the pin becomes an output to avoid glitches
 */
#define PIN_SET_LOW          (0x00)
#define PIN_SET_HIGH         (0x08)

/* Pullup/downs */
#define PIN_FLOATING         (0x00)
#define PIN_PULL_UP          (0x40)
#define PIN_PULL_DOWN        (0x80)

/* Output drive strength/speed (only applies to digital outputs).
 * Not all MCUs will support this and different drivers will interpret
 * this request differently.  For default output drive, omit this bit.
 */
#define PIN_HIGH_CURRENT     (0x04)
#define PIN_HIGH_SPEED       (0x04)

/* Input threshold or hysteresis (only applies to digital inputs).
 * Not all MCUs will support this and different drivers will interpret
 * this request differently.  For default input levels, omit this bit.
 */
#define PIN_TTL_INPUT_LEVEL  (0x04)
#define PIN_SCHMITT_INPUT    (0x04)

/* Hybrid definitions for common use cases */
#define PIN_DIGITAL_IN       (PIN_MODE_DIGITAL | PIN_DIR_IN)
#define PIN_DIGITAL_OUT      (PIN_MODE_DIGITAL | PIN_DIR_OUT)
#define PIN_ANALOG_IN        (PIN_MODE_ANALOG | PIN_DIR_IN)
#define PIN_ANALOG_OUT       (PIN_MODE_ANALOG | PIN_DIR_OUT)

/* Analog channel specifier flag - see analog_input_get() */
#define ADC_CHANNEL_NUM      (0x8000)

/* This value is OR'ed with 1/0 for the pin_level parameter when
 * software calls a GPIO interrupt handler.  Normal hardware
 * interrupts simply use 1 and 0.
 */
#define GPIO_SW_INT          (0x80)


/* GPIO interrupt setup settings */
enum gpio_int_setting_type
{
    // All possible interrupt conditions.
    // Chips that don't support a specific condition
    // will use the closest substitute setting.
    INT_NEVER = 0,
    INT_WHEN_HIGH,
    INT_WHEN_LOW,
    INT_ON_RISING_EDGE,
    INT_ON_FALLING_EDGE,
    INT_ON_BOTH_EDGES,

    // Number of conditions above -
    // this should always be last in the list
    INT_NUM_SETTINGS,

    // Special qualifier to be OR'ed with the above
    WAKE_ON_INT = 0x80
};
typedef enum gpio_int_setting_type  gpio_int_setting_t;

/* GPIO interrupt control settings */
enum gpio_int_control_type
{
    INT_DISABLE = 0,  // Disabled, pin changes ignored, no interrupts occur
    INT_ENABLE,       // Enabled, pin changes generate an interrupt
    INT_BLOCK         // Interrupt not generated but a pin change while blocked
                      // will generate an interrupt if enabled again
};
typedef enum gpio_int_control_type  gpio_int_control_t;


/* A GPIO interrupt handler must conform to this function signature.
 * When an interrupt occurs, gpio_num indicates which pin has caused
 * the interrupt and pin_level is set to 0 or 1.  If this function
 * is called by software, pin_level is OR'ed with GPIO_SW_INT.
 */
typedef void (*gpio_int_fn_t)(uint16_t gpio_num, uint8_t pin_level);


/**
 * @brief Initialisation for the GPIO as a whole, sets up clocks etc.
 *        Call this before using any of the gpio calls
 * @param[IN]  config - Driver-specific parameter.  For most implementations,
 *                      it is the number of pins on the MCU package.
 */
void gpio_init(int config);


/**
 * @brief Configure a GPIO pin
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @param[IN]  pin_mode - Pin type/direction etc. see GPIO pin modes above
 */
void gpio_configure_pin(uint16_t gpio_num, uint8_t pin_mode);


/**
 * @brief Configure a whole GPIO port
 * @param[IN]  port - Port number, for lettered ports 0 = Port A
 * @param[IN]  mode - Port type/direction etc. see GPIO pin modes above
 */
void gpio_configure_port(uint8_t port, uint8_t mode);


/**
 * @brief Set or clear a digital output pin
 * @param[IN]  gpio_num  - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @param[IN]  pin_level - New value for pin, 1/0
 */
void gpio_set(uint16_t gpio_num, uint8_t pin_level);


/**
 * @brief Set the value of a digital output port
 * @param[IN]  port  - Port number, for lettered ports 0 = Port A
 * @param[IN]  value - New value for port
 */
void gpio_set_port(uint8_t port, uint32_t value);


/**
 * @brief Get the current value of a digital input
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @return Input pin level - 1/0
 */
uint8_t gpio_get(uint16_t gpio_num);


/**
 * @brief Get the value of a digital input port
 * @param[IN]  port - Port number, for lettered ports 0 = Port A
 * @return Input port value
 */
uint32_t gpio_get_port(uint8_t port);


/**
 * @brief Setup interrupts on a digital input pin (but don't enable them).
 *        Use this function to associate an interrupt handler with a GPIO pin.
 *        The handler should be a regular C function, not marked 'interrupt'.
 *        May be called a second time to change the handler function or the
 *        pin_state setting.  This function disables the pin interrupt before
 *        applying the setting - use gpio_int_control() to enable the interrupt.
 * @param[IN]  gpio_num  - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @param[IN]  handler   - Interrupt handler to call, cannot be NULL
 * @param[IN]  pin_state - Edge or level to interrupt on
 */
void gpio_int_setup(uint16_t gpio_num, gpio_int_fn_t handler, gpio_int_setting_t pin_state);


/**
 * @brief Enable/disable pin interrupts on a digital input pin
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @param[IN]  setting  - Required interrupt behaviour
 */
void gpio_int_control(uint16_t gpio_num, gpio_int_control_t setting);


/**
 * @brief Returns the GPIO number of the last interrupt or -1 if there
 *        hasn't been an interrupt since this function was last called.
 *        Simply calling this function resets the 'had an interrupt'
 *        state so to decide whether there was an interrupt between two
 *        points in your code, call this first to reset the state then
 *        again to see if an interrupt has occurred since.
 */
int gpio_get_last_int(void);


/**
 * @brief Initialisation for the ADC subsystem, sets up clocks etc.
 *        Call this before using analog_input_get()
 * @param[OUT]  adc_bits  - ADC resolution in bits, NULL if not required
 */
void analog_input_init(uint8_t *adc_bits);


/**
 * @brief Read value of an analog input
 *        Pin must be configured as an analog input with gpio_configure_pin()
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port 1, pin 2.
 *                        If the ADC_CHANNEL_NUM flag is specified, gpio_num is
 *                        interpreted as the ADC channel, instead of the GPIO
 *                        pin number e.g. (ADC_CHANNEL_NUM | 17) = channel 17.
 *                        This is a device-specific feature and is not portable,
 *                        different drivers will interpret this request differently.
 * @return ADC value
 */
uint32_t analog_input_get(uint16_t gpio_num);


/**
 * @brief Initialisation for the DAC subsystem, sets up clocks etc.
 *        Call this before using analog_output_set()
 * @param[OUT]  dac_bits  - DAC resolution in bits, NULL if not required
 */
void analog_output_init(uint8_t *dac_bits);


/**
 * @brief Write value of an analog output
 *        Pin must be configured as an analog output with gpio_configure_pin()
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port 1, pin 2
 * @param[IN]  value    - DAC value to set
 */
void analog_output_set(uint16_t gpio_num, uint32_t value);


#ifdef __cplusplus
}
#endif

#endif

