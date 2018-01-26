/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for Oresat Solar V1 STM32F042K6 board.
 * PART: STM32F042K6T6
 * PACKAGE: LQFP32
 */

/*
 * Board identifier.
 */
#define BOARD_ORSAT_SOLAR_V1
#define BOARD_NAME                  "Oresat solar v1"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

// #define STM32_HSECLK                8000000U
#define STM32_HSECLK                16000000U

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                0U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32F042x6

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0_NC                0U
#define GPIOA_PIN1_NC                1U
#define GPIOA_USART2_TX              2U
#define GPIOA_USART2_RX              3U
#define GPIOA_DEBUG                  4U
#define GPIOA_TP9                    5U
#define GPIOA_TP10                   6U
#define GPIOA_TP11                   7U
#define GPIOA_PIN8_NC                8U
#define GPIOA_CAN_LSTN               9U
#define GPIOA_CAN_EN                 10U
#define GPIOA_CAN_RX                 11U
#define GPIOA_CAN_TX                 12U
#define GPIOA_SWDIO                  13U
#define GPIOA_SWCLK                  14U
#define GPIOA_PIN15_NC               15U

#define GPIOB_PIN0_NC                0U
#define GPIOB_PIN1_NC                1U
#define GPIOB_PIN2_NA                2U
#define GPIOB_TP12                   3U
#define GPIOB_TP13                   4U
#define GPIOB_PIN5_NC                5U
#define GPIOB_I2C1_SCL               6U
#define GPIOB_I2C1_SDA               7U
#define GPIOB_PIN8_BOOT0             8U
#define GPIOB_PIN9                   9U
#define GPIOB_PIN10                  10U
#define GPIOB_PIN11                  11U
#define GPIOB_PIN12                  12U
#define GPIOB_PIN13                  13U
#define GPIOB_PIN14                  14U
#define GPIOB_PIN15                  15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U


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

/*
 * GPIOA setup:
 *
 * PA0  - NC                    (output pulldown).
 * PA1  - NC                    (output pulldown).
 * PA2  - USART2_TX             (alternate 1).
 * PA3  - USART2_RX             (alternate 1).
 * PA4  - DEBUG                 (output pullup).
 * PA5  - TP9                   (output pullup).
 * PA6  - TP10                  (output pullup).
 * PA7  - TP11                  (output pullup).
 * PA8  - NC                    (output pulldown).
 * PA9  - CAN_LSTN              (output pulldown).
 * PA10 - CAN_EN                (output pulldown).
 * PA11 - CAN_RX                (alternate 4).
 * PA12 - CAN_TX                (alternate 4).
 * PA13 - SWDIO                 (alternate 0).
 * PA14 - SWCLK                 (alternate 0).
 * PA15 - NC                    (output pulldown).
 */

#define VAL_GPIOA_MODER             (PIN_MODE_OUTPUT(GPIOA_PIN0_NC) |         \
                                     PIN_MODE_OUTPUT(GPIOA_PIN1_NC) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_RX) |         \
                                     PIN_MODE_OUTPUT(GPIOA_DEBUG) |         \
                                     PIN_MODE_OUTPUT(GPIOA_TP9) |         \
                                     PIN_MODE_OUTPUT(GPIOA_TP10) |         \
                                     PIN_MODE_OUTPUT(GPIOA_TP11) |         \
                                     PIN_MODE_OUTPUT(GPIOA_PIN8_NC) |         \
                                     PIN_MODE_OUTPUT(GPIOA_CAN_LSTN) |         \
                                     PIN_MODE_OUTPUT(GPIOA_CAN_EN) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_PIN15_NC))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOA_PIN0_NC) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN1_NC) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DEBUG) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TP9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TP10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TP11) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN8_NC) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_LSTN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_EN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN15_NC))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_PIN0_NC) |        \
                                     PIN_OSPEED_HIGH(GPIOA_PIN1_NC) |        \
                                     PIN_OSPEED_HIGH(GPIOA_USART2_TX) |         \
                                     PIN_OSPEED_HIGH(GPIOA_USART2_RX) |         \
                                     PIN_OSPEED_HIGH(GPIOA_DEBUG) |        \
                                     PIN_OSPEED_HIGH(GPIOA_TP9) |         \
                                     PIN_OSPEED_HIGH(GPIOA_TP10) |        \
                                     PIN_OSPEED_HIGH(GPIOA_TP11) |        \
                                     PIN_OSPEED_HIGH(GPIOA_PIN8_NC) |        \
                                     PIN_OSPEED_HIGH(GPIOA_CAN_LSTN) |        \
                                     PIN_OSPEED_HIGH(GPIOA_CAN_EN) |        \
                                     PIN_OSPEED_HIGH(GPIOA_CAN_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOA_CAN_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_PIN15_NC))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_PIN0_NC) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN1_NC) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_USART2_TX) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_USART2_RX) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_DEBUG) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_TP9) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_TP10) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_TP11) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN8_NC) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_CAN_LSTN) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_CAN_EN) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN_TX) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN15_NC))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0_NC) |           \
                                     PIN_ODR_HIGH(GPIOA_PIN1_NC) |           \
                                     PIN_ODR_HIGH(GPIOA_USART2_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_USART2_RX) |           \
                                     PIN_ODR_HIGH(GPIOA_DEBUG) |           \
                                     PIN_ODR_HIGH(GPIOA_TP9) |            \
                                     PIN_ODR_HIGH(GPIOA_TP10) |           \
                                     PIN_ODR_HIGH(GPIOA_TP11) |           \
                                     PIN_ODR_HIGH(GPIOA_PIN8_NC) |           \
                                     PIN_ODR_LOW(GPIOA_CAN_LSTN) |           \
                                     PIN_ODR_LOW(GPIOA_CAN_EN) |           \
                                     PIN_ODR_HIGH(GPIOA_CAN_RX) |          \
                                     PIN_ODR_HIGH(GPIOA_CAN_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15_NC))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0_NC, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_PIN1_NC, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_USART2_TX, 1U) |        \
                                     PIN_AFIO_AF(GPIOA_USART2_RX, 1U) |        \
                                     PIN_AFIO_AF(GPIOA_DEBUG, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_TP9, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_TP10, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_TP11, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8_NC, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_CAN_LSTN, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_CAN_EN, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_CAN_RX, 4U) |       \
                                     PIN_AFIO_AF(GPIOA_CAN_TX, 4U) |        \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15_NC, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - NC                    (output pulldown).
 * PB1  - NC                    (output pulldown).
 * NOT AVAIL ON LQFP32--- PB2  - PIN2                      (input pullup).
 * PB3  - TP12                  (output pullup)
 * PB4  - TP13                  (output pullup).
 * PB5  - NC                    (output pulldown).
 * PB6  - I2C1_SCL              (alternate 1)
 * PB7  - I2C1_SDA              (alternate 1).
 * PB8  - NC BOOT0              (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_PIN0_NC) |         \
                                     PIN_MODE_OUTPUT(GPIOB_PIN1_NC) |         \
                                     PIN_MODE_INPUT(GPIOB_PIN2_NA) |           \
                                     PIN_MODE_OUTPUT(GPIOB_TP12) |       \
                                     PIN_MODE_OUTPUT(GPIOB_TP13) |        \
                                     PIN_MODE_OUTPUT(GPIOB_PIN5_NC) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |         \
                                     PIN_MODE_INPUT(GPIOB_PIN8_BOOT0) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_PIN0_NC) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1_NC) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2_NA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_TP12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_TP13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5_NC) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8_BOOT0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_PIN0_NC) |        \
                                     PIN_OSPEED_HIGH(GPIOB_PIN1_NC) |        \
                                     PIN_OSPEED_HIGH(GPIOB_PIN2_NA) |          \
                                     PIN_OSPEED_HIGH(GPIOB_TP12) |       \
                                     PIN_OSPEED_HIGH(GPIOB_TP13) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN5_NC) |       \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |        \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |        \
                                     PIN_OSPEED_HIGH(GPIOB_PIN8_BOOT0) |          \
                                     PIN_OSPEED_HIGH(GPIOB_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOB_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_PIN0_NC) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN1_NC) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN2_NA) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_TP12) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_TP13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN5_NC) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN8_BOOT0) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0_NC) |           \
                                     PIN_ODR_HIGH(GPIOB_PIN1_NC) |           \
                                     PIN_ODR_HIGH(GPIOB_PIN2_NA) |             \
                                     PIN_ODR_HIGH(GPIOB_TP12) |           \
                                     PIN_ODR_HIGH(GPIOB_TP13) |          \
                                     PIN_ODR_HIGH(GPIOB_PIN5_NC) |          \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |           \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |           \
                                     PIN_ODR_HIGH(GPIOB_PIN8_BOOT0) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0_NC, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_PIN1_NC, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_PIN2_NA, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_TP12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_TP13, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN5_NC, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 1U) |        \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 1U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8_BOOT0, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (input pullup).
 * PC1  - PIN1                      (input pullup).
 * PC2  - PIN2                      (input pullup).
 * PC3  - PIN3                      (input pullup).
 * PC4  - PIN4                      (input pullup).
 * PC5  - PIN5                      (input pullup).
 * PC6  - PIN6                      (input pullup).
 * PC7  - PIN7                      (input pullup).
 * PC8  - PIN8                      (input pullup).
 * PC9  - PIN9                      (input pullup).
 * PC10 - PIN10                     (input pullup).
 * PC11 - PIN11                     (input pullup).
 * PC12 - PIN12                     (input pullup).
 * PC13 - PIN13                     (input pullup).
 * PC14 - PIN14                     (input pullup).
 * PC15 - PIN15                     (input pullup).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - PIN2                      (input pullup).
 * PD3  - PIN3                      (input pullup).
 * PD4  - PIN4                      (input pullup).
 * PD5  - PIN5                      (input pullup).
 * PD6  - PIN6                      (input pullup).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - PIN12                     (input pullup).
 * PD13 - PIN13                     (input pullup).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))
/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (input pullup).
 * PF1  - OSC_OUT                    (input pullup).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - PIN6                      (input pullup).
 * PF7  - PIN7                      (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - PIN9                      (input pullup).
 * PF10 - PIN10                     (input pullup).
 * PF11 - PIN11                     (input pullup).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT) |         \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOF_OSC_OUT) |        \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_OSC_IN) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_OSC_OUT) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOF_OSC_OUT) |           \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))


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
