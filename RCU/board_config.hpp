#ifndef BOARD_CONFIG_HPP
#define BOARD_CONFIG_HPP

#include "pico/stdlib.h"

/*
 * Pin map for the custom RCU PCB.
 *
 * Everything board-specific lives here so the drivers stay portable and there
 * is exactly one place to edit when the layout changes. Note that PICO_BOARD
 * in CMakeLists.txt is still "pico"; that only supplies defaults (LED on 25,
 * SPI0 on 16/17/18/19) which happen to match this board, but do not rely on
 * PICO_DEFAULT_* elsewhere in the firmware.
 */
namespace board {

/* Thruster ESC signal (RC servo pulse). */
constexpr uint kEscGpio = 28;

/* MCP2515 CAN controller on spi0. The SDK's SPI driver does not manage chip
 * select, so kCanCs is driven by hand around each transfer. */
constexpr uint kCanSck   = 18;  /* -> MCP2515 SCK */
constexpr uint kCanTx    = 19;  /* -> MCP2515 SI  (Pico MOSI) */
constexpr uint kCanRx    = 16;  /* <- MCP2515 SO  (Pico MISO) */
constexpr uint kCanCs    = 17;  /* -> MCP2515 CS,  active low */
constexpr uint kCanReset = 20;  /* -> MCP2515 RESET, active low */
constexpr uint kCanInt   = 21;  /* <- MCP2515 INT, active low, open drain */

constexpr uint32_t kCanXtalHz  = 8000000;
constexpr uint32_t kCanBitrate = 250000;

/* Status LED. Stock-Pico default; change if this PCB puts it elsewhere. */
constexpr uint kLedGpio = PICO_DEFAULT_LED_PIN;

}  // namespace board

#endif /* BOARD_CONFIG_HPP */
