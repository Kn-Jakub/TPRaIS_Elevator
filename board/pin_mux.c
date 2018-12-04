/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.1
processor: MKL25Z128xxx4
package_id: MKL25Z128VLK4
mcu_data: ksdk2_0
processor_version: 4.0.0
board: FRDM-KL25Z
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '28', peripheral: UART0, signal: TX, pin_signal: TSI0_CH3/PTA2/UART0_TX/TPM2_CH1}
  - {pin_num: '27', peripheral: UART0, signal: RX, pin_signal: TSI0_CH2/PTA1/UART0_RX/TPM2_CH0}
  - {pin_num: '24', peripheral: I2C0, signal: SCL, pin_signal: PTE24/TPM0_CH0/I2C0_SCL}
  - {pin_num: '25', peripheral: I2C0, signal: SDA, pin_signal: PTE25/TPM0_CH1/I2C0_SDA}
  - {pin_num: '74', peripheral: GPIOD, signal: 'GPIO, 1', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/TPM0_CH1, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '53', peripheral: GPIOB, signal: 'GPIO, 18', pin_signal: TSI0_CH11/PTB18/TPM2_CH0, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '54', peripheral: GPIOB, signal: 'GPIO, 19', pin_signal: TSI0_CH12/PTB19/TPM2_CH1, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '34', peripheral: GPIOA, signal: 'GPIO, 14', pin_signal: PTA14/SPI0_PCS0/UART0_TX, direction: INPUT, gpio_interrupt: kPORT_InterruptRisingEdge}
  - {pin_num: '35', peripheral: GPIOA, signal: 'GPIO, 15', pin_signal: PTA15/SPI0_SCK/UART0_RX, direction: INPUT, gpio_interrupt: kPORT_InterruptRisingEdge}
  - {pin_num: '42', peripheral: GPIOA, signal: 'GPIO, 20', pin_signal: PTA20/RESET_b, direction: INPUT, gpio_interrupt: kPORT_InterruptFallingEdge}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port D Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    gpio_pin_config_t ACCEL_INT1_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA14 (pin 34)  */
    GPIO_PinInit(BOARD_INITPINS_ACCEL_INT1_GPIO, BOARD_INITPINS_ACCEL_INT1_PIN, &ACCEL_INT1_config);

    gpio_pin_config_t ACCEL_INT2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA15 (pin 35)  */
    GPIO_PinInit(BOARD_INITPINS_ACCEL_INT2_GPIO, BOARD_INITPINS_ACCEL_INT2_PIN, &ACCEL_INT2_config);

    gpio_pin_config_t gpioa_pin42_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA20 (pin 42)  */
    GPIO_PinInit(GPIOA, 20U, &gpioa_pin42_config);

    gpio_pin_config_t LED_RED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTB18 (pin 53)  */
    GPIO_PinInit(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_PIN, &LED_RED_config);

    gpio_pin_config_t LED_GREEN_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTB19 (pin 54)  */
    GPIO_PinInit(BOARD_INITPINS_LED_GREEN_GPIO, BOARD_INITPINS_LED_GREEN_PIN, &LED_GREEN_config);

    gpio_pin_config_t LED_BLUE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTD1 (pin 74)  */
    GPIO_PinInit(BOARD_INITPINS_LED_BLUE_GPIO, BOARD_INITPINS_LED_BLUE_PIN, &LED_BLUE_config);

    /* PORTA1 (pin 27) is configured as UART0_RX */
    PORT_SetPinMux(BOARD_INITPINS_DEBUG_UART_RX_PORT, BOARD_INITPINS_DEBUG_UART_RX_PIN, kPORT_MuxAlt2);

    /* PORTA14 (pin 34) is configured as PTA14 */
    PORT_SetPinMux(BOARD_INITPINS_ACCEL_INT1_PORT, BOARD_INITPINS_ACCEL_INT1_PIN, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTA14 (pin 34): Interrupt on rising edge */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT1_PORT, BOARD_INITPINS_ACCEL_INT1_PIN, kPORT_InterruptRisingEdge);

    /* PORTA15 (pin 35) is configured as PTA15 */
    PORT_SetPinMux(BOARD_INITPINS_ACCEL_INT2_PORT, BOARD_INITPINS_ACCEL_INT2_PIN, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTA15 (pin 35): Interrupt on rising edge */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT2_PORT, BOARD_INITPINS_ACCEL_INT2_PIN, kPORT_InterruptRisingEdge);

    /* PORTA2 (pin 28) is configured as UART0_TX */
    PORT_SetPinMux(BOARD_INITPINS_DEBUG_UART_TX_PORT, BOARD_INITPINS_DEBUG_UART_TX_PIN, kPORT_MuxAlt2);

    /* PORTA20 (pin 42) is configured as PTA20 */
    PORT_SetPinMux(PORTA, 20U, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTA20 (pin 42): Interrupt on falling edge */
    PORT_SetPinInterruptConfig(PORTA, 20U, kPORT_InterruptFallingEdge);

    /* PORTB18 (pin 53) is configured as PTB18 */
    PORT_SetPinMux(BOARD_INITPINS_LED_RED_PORT, BOARD_INITPINS_LED_RED_PIN, kPORT_MuxAsGpio);

    /* PORTB19 (pin 54) is configured as PTB19 */
    PORT_SetPinMux(BOARD_INITPINS_LED_GREEN_PORT, BOARD_INITPINS_LED_GREEN_PIN, kPORT_MuxAsGpio);

    /* PORTD1 (pin 74) is configured as PTD1 */
    PORT_SetPinMux(BOARD_INITPINS_LED_BLUE_PORT, BOARD_INITPINS_LED_BLUE_PIN, kPORT_MuxAsGpio);

    /* PORTE24 (pin 24) is configured as I2C0_SCL */
    PORT_SetPinMux(BOARD_INITPINS_ACCEL_SCL_PORT, BOARD_INITPINS_ACCEL_SCL_PIN, kPORT_MuxAlt5);

    /* PORTE25 (pin 25) is configured as I2C0_SDA */
    PORT_SetPinMux(BOARD_INITPINS_ACCEL_SDA_PORT, BOARD_INITPINS_ACCEL_SDA_PIN, kPORT_MuxAlt5);

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK)))

                  /* UART0 transmit data source select: UART0_TX pin. */
                  | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)

                  /* UART0 receive data source select: UART0_RX pin. */
                  | SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
