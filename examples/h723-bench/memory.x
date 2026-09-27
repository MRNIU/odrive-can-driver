/* Copyright The odrive-can-driver Contributors */
/* STM32H723VGT6 normal application area and AXI SRAM.
 * The final 256 KiB is intentionally left to the existing A/B configuration slots. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 768K
  RAM   : ORIGIN = 0x24000000, LENGTH = 320K
}
