#ifndef BOARD_CONFIG_HPP
#define BOARD_CONFIG_HPP

#include "pico/stdlib.h"

/*
 * Pin map for the LCU (LoRa Control Unit) board.
 *
 * ASSUMPTION: the CAN pins are the same as the RCU, because the two boards
 * were described as similar. VERIFY AGAINST THE SCHEMATIC before trusting a
 * bring-up failure -- if the MCP2515 does not answer, this is the first thing
 * to check.
 */
namespace board {

/* MCP2515 CAN controller on spi0. */
constexpr uint kCanSck   = 18;  /* -> MCP2515 SCK */
constexpr uint kCanTx    = 19;  /* -> MCP2515 SI  (Pico MOSI) */
constexpr uint kCanRx    = 16;  /* <- MCP2515 SO  (Pico MISO) */
constexpr uint kCanCs    = 17;  /* -> MCP2515 CS,  active low */
constexpr uint kCanReset = 20;  /* -> MCP2515 RESET, active low */
constexpr uint kCanInt   = 21;  /* <- MCP2515 INT, active low, open drain */

constexpr uint32_t kCanXtalHz  = 8000000;
constexpr uint32_t kCanBitrate = 250000;

/* Status LED. */
constexpr uint kLedGpio = PICO_DEFAULT_LED_PIN;

/*
 * TODO: LoRa module pins. Not filled in because the module is not chosen yet.
 *
 * An SX127x/RFM95 needs a second SPI bus (or a shared bus with its own CS),
 * plus DIO0 for RX-done and a RESET line. A UART module (E32, RYLR) needs only
 * TX/RX and possibly M0/M1 mode pins. The two need completely different
 * drivers, so nothing is assumed here.
 */

}  // namespace board

#endif /* BOARD_CONFIG_HPP */
