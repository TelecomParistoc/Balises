#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for radioboard
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F3_DISCOVERY
#define BOARD_NAME                  "radioboard"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_LSE_BYPASS
//#define STM32_HSE_BYPASS

/*
 * MCU type as defined in the ST header.
 */
#define STM32F302x8

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_PIN2                  2U
#define GPIOA_VBAT_PROBE            3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_PIN9                  9U
#define GPIOA_USB_CONNECT           10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_DWM_IRQ               15U

#define GPIOB_PIN0                  0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_DWM_SCK               3U
#define GPIOB_DWM_MISO              4U
#define GPIOB_DWM_MOSI              5U
#define GPIOB_DWM_nCS               6U
#define GPIOB_DWM_RST               7U
#define GPIOB_RPI_SCL               8U
#define GPIOB_RPI_SDA               9U
#define GPIOB_RPI_TX                10U
#define GPIOB_RPI_RX                11U
#define GPIOB_MOT_nCS               12U
#define GPIOB_MOT_SCK               13U
#define GPIOB_MOT_MISO              14U
#define GPIOB_MOT_MOSI              15U

#define GPIOC_LED_BATT              13U
#define GPIOC_LED_RXTX              14U
#define GPIOC_LED_SYNC              15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U

/*
 * IO lines assignments.
 */
#define LINE_USB_CONNECT           PAL_LINE(GPIOA, GPIOA_USB_CONNECT)
#define LINE_DWM_IRQ               PAL_LINE(GPIOA, GPIOA_DWM_IRQ)

#define LINE_DWM_nCS               PAL_LINE(GPIOB, GPIOB_DWM_nCS)
#define LINE_DWM_RST               PAL_LINE(GPIOB, GPIOB_DWM_RST)
#define LINE_MOT_nCS               PAL_LINE(GPIOB, GPIOB_MOT_nCS)

#define LINE_LED_BATT              PAL_LINE(GPIOC, GPIOC_LED_BATT)
#define LINE_LED_RXTX              PAL_LINE(GPIOC, GPIOC_LED_RXTX)
#define LINE_LED_SYNC              PAL_LINE(GPIOC, GPIOC_LED_SYNC)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

// [MODE OTYPE OSPEED PUPDR]

/*
 * GPIOA setup:
 *
 * PA0  - PIN0                      (input floating).
 * PA1  - PIN1                      (input floating).
 * PA2  - PIN2                      (input floating).
 * PA3  - VBAT_PROBE                (analog).
 * PA4  - PIN4                      (input floating).
 * PA5  - PIN5                      (input floating).
 * PA6  - PIN6                      (input floating).
 * PA7  - PIN7                      (input floating).
 * PA8  - PIN8                      (input floating).
 * PA9  - PIN9                      (input floating).
 * PA10 - USB_CONNECT               (output push-pull).
 * PA11 - USB_DM                    (input floating).
 * PA12 - USB_DP                    (input floating).
 * PA13 - SWDIO                     (alternate 0 pull-up).
 * PA14 - SWCLK                     (alternate 0 pull-down).
 * PA15 - DWM_IRQ                   (input floating).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOA_VBAT_PROBE) |   \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN6) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN7) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN9) |          \
                                     PIN_MODE_OUTPUT(GPIOA_USB_CONNECT) |  \
                                     PIN_MODE_INPUT(GPIOA_USB_DM) |        \
                                     PIN_MODE_INPUT(GPIOA_USB_DP) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |     \
                                     PIN_MODE_INPUT(GPIOA_DWM_IRQ))
#define VAL_GPIOA_OTYPER    0x00000000
#define VAL_GPIOA_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_SWDIO)  |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK))
#define VAL_GPIOA_ODR       0x00000000
#define VAL_GPIOA_AFRL      0x00000000
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0U)   |       \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (input floating).
 * PB1  - PIN1                      (input floating).
 * PB2  - PIN2                      (input floating).
 * PB3  - DWM_SCK                   (alternate 6).
 * PB4  - DWM_MISO                  (alternate 6).
 * PB5  - DWM_MOSI                  (alternate 6).
 * PB6  - DWM_nCS                   (output push-pull).
 * PB7  - DWM_RST                   (output push-pull).
 * PB8  - RPI_SCL                   (alternate 4 open drain).
 * PB9  - RPI_SDA                   (alternate 4 open drain).
 * PB10 - RPI_TX                    (alternate 7).
 * PB11 - RPI_RX                    (alternate 7).
 * PB12 - MOT_nCS                   (output push-pull).
 * PB13 - MOT_SCK                   (alternate 5).
 * PB14 - MOT_MISO                  (alternate 5).
 * PB15 - MOT_MOSI                  (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_DWM_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_DWM_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_DWM_MOSI) |   \
                                     PIN_MODE_OUTPUT(GPIOB_DWM_nCS) |       \
                                     PIN_MODE_OUTPUT(GPIOB_DWM_RST) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_SDA) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_RX) |     \
                                     PIN_MODE_OUTPUT(GPIOB_MOT_nCS) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_MOT_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_MOT_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_MOT_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_RPI_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_RPI_SDA))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR     0x00000000
#define VAL_GPIOB_ODR       0x00000000
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_DWM_SCK, 6U) |       \
                                     PIN_AFIO_AF(GPIOB_DWM_MISO, 6U) |      \
                                     PIN_AFIO_AF(GPIOB_DWM_MOSI, 6U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_RPI_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_RPI_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_RPI_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOB_RPI_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOB_MOT_SCK, 5U) |       \
                                     PIN_AFIO_AF(GPIOB_MOT_MISO, 5U) |       \
                                     PIN_AFIO_AF(GPIOB_MOT_MOSI, 5U))

/*
 * GPIOC setup:
 *
 * PC13 - LED_BATT                  (output push-pull).
 * PC14 - LED_RXTX                  (output push-pull).
 * PC15 - LED_SYNC                  (output push-pull).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED_BATT) |      \
                                     PIN_MODE_OUTPUT(GPIOC_LED_RXTX) |      \
                                     PIN_MODE_OUTPUT(GPIOC_LED_SYNC))
#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     0x00000000
#define VAL_GPIOC_ODR       0x00000000
#define VAL_GPIOC_AFRL      0x00000000
#define VAL_GPIOC_AFRH      0x00000000

/*
 * GPIOD and GPIOE don't exist but seem to be required by ChibiOS
 */
#define VAL_GPIOD_MODER     0x00000000
#define VAL_GPIOD_OTYPER    0x00000000
#define VAL_GPIOD_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOD_PUPDR     0x00000000
#define VAL_GPIOD_ODR       0x00000000
#define VAL_GPIOD_AFRL      0x00000000
#define VAL_GPIOD_AFRH      0x00000000
#define VAL_GPIOE_MODER     0x00000000
#define VAL_GPIOE_OTYPER    0x00000000
#define VAL_GPIOE_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOE_PUPDR     0x00000000
#define VAL_GPIOE_ODR       0x00000000
#define VAL_GPIOE_AFRL      0x00000000
#define VAL_GPIOE_AFRH      0x00000000

/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (input floating).
 * PF1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT))
#define VAL_GPIOF_OTYPER    0x00000000
#define VAL_GPIOF_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOF_PUPDR     0x00000000
#define VAL_GPIOF_ODR       0x00000000
#define VAL_GPIOF_AFRL      0x00000000
#define VAL_GPIOF_AFRH      0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
