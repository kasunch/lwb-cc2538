/**
 * \addtogroup cc2538-rf
 * @{
 *
 * \file
 * Implementation of the cc2538 RF driver
 */

#include <stdint.h>

#include "cc2538-rf.h"

#include "contiki.h"
#include "dev/radio.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/linkaddr.h"
#include "net/netstack.h"
#include "sys/energest.h"
#include "dev/rfcore.h"
#include "dev/sys-ctrl.h"
#include "dev/udma.h"
#include "reg.h"

#include <string.h>
/*---------------------------------------------------------------------------*/
#define CHECKSUM_LEN 2

/* uDMA channel control persistent flags */
#define UDMA_TX_FLAGS (UDMA_CHCTL_ARBSIZE_128 | UDMA_CHCTL_XFERMODE_AUTO \
    | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8 \
    | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_DSTINC_NONE)

#define UDMA_RX_FLAGS (UDMA_CHCTL_ARBSIZE_128 | UDMA_CHCTL_XFERMODE_AUTO \
    | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8 \
    | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTINC_8)

/*
 * uDMA transfer threshold. DMA will only be used to read an incoming frame
 * if its size is above this threshold
 */
#define UDMA_RX_SIZE_THRESHOLD 3
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Local RF Flags */
#define RF_MUST_RESET 0x40
#define RF_ON         0x01

/* Bit Masks for the last byte in the RX FIFO */
#define CRC_BIT_MASK 0x80
#define LQI_BIT_MASK 0x7F
/* RSSI Offset */
#define RSSI_OFFSET    73

/* 192 usec off -> on interval (RX Callib -> SFD Wait). We wait a bit more */
#define ONOFF_TIME                    RTIMER_ARCH_SECOND / 3125
/*---------------------------------------------------------------------------*/
#ifdef CC2538_RF_CONF_AUTOACK
#define CC2538_RF_AUTOACK CC2538_RF_CONF_AUTOACK
#else
#define CC2538_RF_AUTOACK 1
#endif
/*---------------------------------------------------------------------------
 * MAC timer
 *---------------------------------------------------------------------------*/
/* Timer conversion */
#define RADIO_TO_RTIMER(X) ((uint32_t)((uint64_t)(X) * RTIMER_ARCH_SECOND / SYS_CTRL_32MHZ))

#define CLOCK_STABLE() do {															\
			while ( !(REG(SYS_CTRL_CLOCK_STA) & (SYS_CTRL_CLOCK_STA_XOSC_STB)));	\
		} while(0)

/*---------------------------------------------------------------------------*/
/* TX Power dBm lookup table. Values from SmartRF Studio v1.16.0 */
typedef struct output_config {
  radio_value_t power;
  uint8_t txpower_val;
} output_config_t;

static const output_config_t output_power[] = {
  {  7, 0xFF },
  {  5, 0xED },
  {  3, 0xD5 },
  {  1, 0xC5 },
  {  0, 0xB6 },
  { -1, 0xB0 },
  { -3, 0xA1 },
  { -5, 0x91 },
  { -7, 0x88 },
  { -9, 0x72 },
  {-11, 0x62 },
  {-13, 0x58 },
  {-15, 0x42 },
  {-24, 0x00 },
};

#define OUTPUT_CONFIG_COUNT (sizeof(output_power) / sizeof(output_config_t))

/* Max and Min Output Power in dBm */
#define OUTPUT_POWER_MIN    (output_power[OUTPUT_CONFIG_COUNT - 1].power)
#define OUTPUT_POWER_MAX    (output_power[0].power)

/*---------------------------------------------------------------------------*/
static void
cc2838_rf_on(void)
{
  CC2538_RF_CSP_ISFLUSHRX();
  CC2538_RF_CSP_ISRXON();
}
///*---------------------------------------------------------------------------*/
static void
cc2838_rf_off(void)
{
  /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
  while(REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

  if(!(REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_FIFOP)) {
    CC2538_RF_CSP_ISFLUSHRX();
  }

  /* Don't turn off if we are off as this will trigger a Strobe Error */
  if(REG(RFCORE_XREG_RXENABLE) != 0) {
    CC2538_RF_CSP_ISRFOFF();
  }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Get the current operating channel
 * \return Returns a value in [11,26] representing the current channel
 */
uint8_t
cc2538_rf_get_channel()
{
  uint8_t chan = REG(RFCORE_XREG_FREQCTRL) & RFCORE_XREG_FREQCTRL_FREQ;

  return (chan - CC2538_RF_CHANNEL_MIN) / CC2538_RF_CHANNEL_SPACING
         + CC2538_RF_CHANNEL_MIN;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set the current operating channel
 * \param channel The desired channel as a value in [11,26]
 * \return Returns a value in [11,26] representing the current channel
 *         or a negative value if \e channel was out of bounds
 */
int8_t
cc2538_rf_set_channel(uint8_t channel)
{
  uint8_t was_on = 0;

  PRINTF("RF: Set Channel %u\n", channel);

  if((channel < CC2538_RF_CHANNEL_MIN) || (channel > CC2538_RF_CHANNEL_MAX)) {
    return CC2538_RF_CHANNEL_SET_ERROR;
  }

  /* Changes to FREQCTRL take effect after the next recalibration */

  /* If we are not off, switch off */
  if((REG(RFCORE_XREG_FSMSTAT0) & RFCORE_XREG_FSMSTAT0_FSM_FFCTRL_STATE) != 0) {
    was_on = 1;
    cc2838_rf_off();
  }
  REG(RFCORE_XREG_FREQCTRL) = CC2538_RF_CHANNEL_MIN +
                              (channel - CC2538_RF_CHANNEL_MIN) * CC2538_RF_CHANNEL_SPACING;

  /* switch radio back on only if radio was on before */
  if(was_on) {
    cc2838_rf_on();
  }

  return (int8_t)channel;
}

/*---------------------------------------------------------------------------*/
/* Returns the current TX power in dBm */
radio_value_t
cc2538_rf_get_tx_power(void)
{
  int i;
  uint8_t reg_val = REG(RFCORE_XREG_TXPOWER) & 0xFF;

  /*
   * Find the TXPOWER value in the lookup table
   * If the value has been written with set_tx_power, we should be able to
   * find the exact value. However, in case the register has been written in
   * a different fashion, we return the immediately lower value of the lookup
   */
  for(i = 0; i < OUTPUT_CONFIG_COUNT; i++) {
    if(reg_val >= output_power[i].txpower_val) {
      return output_power[i].power;
    }
  }
  return OUTPUT_POWER_MIN;
}
/*---------------------------------------------------------------------------*/
/*
 * Set TX power to 'at least' power dBm
 * This works with a lookup table. If the value of 'power' does not exist in
 * the lookup table, TXPOWER will be set to the immediately higher available
 * value
 */
void
cc2538_rf_set_tx_power(radio_value_t power)
{
  int i;

  for(i = OUTPUT_CONFIG_COUNT - 1; i >= 0; --i) {
    if(power <= output_power[i].power) {
      REG(RFCORE_XREG_TXPOWER) = output_power[i].txpower_val;
      return;
    }
  }
}

int
cc2538_rf_csp_reset(void)
{

  /* Clears the CSP program memory */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISCLEAR;
  /* Reset the write pointer */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISSTOP;
  /* Disable MAC timer EVENT1 */
  REG(RFCORE_SFR_MTCSPCFG) |= RFCORE_SFR_MTCSPCFG_MACTIMER_EVENT1_CFG;
  /* Disable MAC timer EVENMT */
  REG(RFCORE_SFR_MTCSPCFG) |= RFCORE_SFR_MTCSPCFG_MACTIMER_EVENMT_CFG;

  return 1;
}

/*---------------------------------------------------------------------------*/
static void
mac_timer_init(void)
{
  /* Wait until external 32 MHz clock becomes stable */
  CLOCK_STABLE();

  /* FIXME: Not sure why the MAC timer has to be started and restarted */

  /* Start timer synchronously */
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_SYNC;
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_RUN;
  /* Wait until timer starts to run */
  while(!(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE));

  /* Stop the timer */
  REG(RFCORE_SFR_MTCTRL) &= ~RFCORE_SFR_MTCTRL_RUN;
  /* Wait until the timer stops */
  while(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE);

  /* Start timer synchronously */
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_SYNC;
  REG(RFCORE_SFR_MTCTRL) |= (RFCORE_SFR_MTCTRL_RUN);
  /* Wait until timer starts to run */
  while(!(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE));

  /* Disable all MAC timer interrupts */
  REG(RFCORE_SFR_MTIRQM) = 0;
  /* Clear pending MAC timer interrupts */
  REG(RFCORE_SFR_MTIRQF) = 0;
  /* But enable from NVIC */
  NVIC_EnableIRQ(MACT_IRQn);
}

/*---------------------------------------------------------------------------*/
/**
 * @brief Get the current value of the MAC timer time stamp.
 *        This function take 94 cycles to execute as measured with debug cycle counter.
 * @return
 */
uint64_t
cc2538_rf_get_mac_time_now(void)
{
  uint64_t timer_val, buffer;

  /* Set MTMSEL bits to 000 as we are going to read timer counter value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000000;
  /* Set LATCH_MODE bits to latch high byte (MTM1) and
   * the entire overflow counter (MTMOVF0, MTMOVF1 and MTMOVF2) at once when
   * low byte (MTM0) is read
   */
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_LATCH_MODE;
  /* Read the low byte (1st octet) of the timer. Note that timer counter is 16-bit */
  timer_val = REG(RFCORE_SFR_MTM0) & RFCORE_SFR_MTM0_MTM0;
  /* Read the high byte (2st octet) of the timer */
  timer_val |= ((REG(RFCORE_SFR_MTM1) & RFCORE_SFR_MTM1_MTM1) << 8);
  /* Set MTMOVFSEL bits to 000 as we are going to read overflow counter value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000000;
  /* Read 3rd octet of the timer. Note that overflow counter is 24-bit */
  timer_val |= ((REG(RFCORE_SFR_MTMOVF0) & RFCORE_SFR_MTMOVF0_MTMOVF0) << 16);
  /* Read 4th octet of the timer */
  timer_val |= ((REG(RFCORE_SFR_MTMOVF1) & RFCORE_SFR_MTMOVF1_MTMOVF1) << 24);
  /* Read 5th octet of the timer */
  buffer = REG(RFCORE_SFR_MTMOVF2) & RFCORE_SFR_MTMOVF2_MTMOVF2;
  timer_val |= (buffer << 32);

  return timer_val;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief Get the SFD time stamp from MAC timer
 *        This function take 103 cycles to execute as measured with debug cycle counter.
 * @return
 */
uint64_t
cc2538_rf_get_sfd_timestamp(void)
{
  uint64_t sfd, buffer;

  /* Set MTMSEL bits to 000 as we are going to read timer capture value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000001;
  /* Set LATCH_MODE bits to latch high byte (MTM1) and
   * the entire overflow counter (MTMOVF0, MTMOVF1 and MTMOVF2) at once when
   * low byte (MTM0) is read
   */
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_LATCH_MODE;
  /* Read the low byte (1st octet) of capture value. Note that capture value is 16-bit */
  sfd = REG(RFCORE_SFR_MTM0) & RFCORE_SFR_MTM0_MTM0;
  /* Read the high byte (2st octet) of the capture value */
  sfd |= ((REG(RFCORE_SFR_MTM1) & RFCORE_SFR_MTM1_MTM1) << 8);
  /* Set MTMOVFSEL bits to 001 as we are going to read overflow counter value of the capture value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000010;
  /* Read 3th octet of the timer */
  sfd |= ((REG(RFCORE_SFR_MTMOVF0) & RFCORE_SFR_MTMOVF0_MTMOVF0) << 16);
  /* Read 4th octet of the timer */
  sfd |= ((REG(RFCORE_SFR_MTMOVF1) & RFCORE_SFR_MTMOVF1_MTMOVF1) << 24);
  /* Read 5th octet of the timer */
  buffer = REG(RFCORE_SFR_MTMOVF2) & RFCORE_SFR_MTMOVF2_MTMOVF2;
  sfd |= (buffer << 32);

  return sfd;
}

/*---------------------------------------------------------------------------*/
int
cc2538_rf_init(void)
{
  /* Enable clock for the RF Core while Running, in Sleep and Deep Sleep */
  REG(SYS_CTRL_RCGCRFC) = 1;
  REG(SYS_CTRL_SCGCRFC) = 1;
  REG(SYS_CTRL_DCGCRFC) = 1;

  REG(RFCORE_XREG_CCACTRL0) = CC2538_RF_CCA_THRES;

  /*
   * Changes from default values
   * See User Guide, section "Register Settings Update"
   */
  REG(RFCORE_XREG_TXFILTCFG) = 0x09;    /** TX anti-aliasing filter bandwidth */
  REG(RFCORE_XREG_AGCCTRL1) = 0x15;     /** AGC target value */
  REG(ANA_REGS_IVCTRL) = 0x0B;          /** Bias currents */

  /*
   * Defaults:
   * Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
   * RX and TX modes with FIFOs
   */
  REG(RFCORE_XREG_FRMCTRL0) = RFCORE_XREG_FRMCTRL0_AUTOCRC;

  /* No auto acknowledging */
  REG(RFCORE_XREG_FRMCTRL0) &= ~RFCORE_XREG_FRMCTRL0_AUTOACK;

  /* Disable frame filtering */
  REG(RFCORE_XREG_FRMFILT0) &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;

  /* Disable source address matching and autopend */
  REG(RFCORE_XREG_SRCMATCH) = 0;

  /* MAX FIFOP threshold */
  REG(RFCORE_XREG_FIFOPCTRL) = CC2538_RF_RXFIFO_THRES;

  /* Set TX Power */
  REG(RFCORE_XREG_TXPOWER) = CC2538_RF_TX_POWER;

  /* Set RF channel */
  cc2538_rf_set_channel(CC2538_RF_CHANNEL);

  /* Enable all RF Error interrupts */
  REG(RFCORE_XREG_RFERRM) = RFCORE_XREG_RFERRM_RFERRM;
  NVIC_EnableIRQ(RF_ERR_IRQn);

  /* Enable SFD interrupt */
  /* Enable RXPKTDONE interrupt */
  REG(RFCORE_XREG_RFIRQM0) = RFCORE_XREG_RFIRQM0_SFD | RFCORE_XREG_RFIRQM0_RXPKTDONE | RFCORE_XREG_RFIRQM0_FIFOP;
  /* Enable TXDONE interrupt */
  REG(RFCORE_XREG_RFIRQM1) = RFCORE_XREG_RFIRQM1_TXDONE;

  NVIC_EnableIRQ(RF_TX_RX_IRQn);


  /* FIXME: We use RFCORE_SFR_RFDATA register directly to read from TX and RX FIFOs
   *        for the moment. We have to check how much delay is introduced from uDMA
   *        transfer.
   */

  /* Initialize MAC timer */
  mac_timer_init();

  return 1;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief The cc2538 RF RX/TX ISR
 *
 *        This is the interrupt service routine for all RF interrupts relating
 *        to RX and TX. Implementation of this moved to the source file of
 *        glossy's implementation.
 */

//void
//cc2538_rf_rx_tx_isr(void)
//{
//
//  REG(RFCORE_SFR_RFIRQF0) = 0;
//
//}
/*---------------------------------------------------------------------------*/
/**
 * \brief The cc2538 RF Error ISR
 *
 *        This is the interrupt service routine for all RF errors. We
 *        enable every error type just for debugging.
 */
//void
//cc2538_rf_err_isr(void)
//{
//#if ENERGEST_CONF_ON
//  ENERGEST_ON(ENERGEST_TYPE_IRQ);
//#endif /* ENERGEST_CONF_ON */
//
//  PRINTF("RF Error: 0x%08lx\n", REG(RFCORE_SFR_RFERRF));
//  /*FIXME: Add to RF error statistics. */
//
//  /* Clear pending interrupts */
//  REG(RFCORE_SFR_RFERRF) = 0;
//
//#if ENERGEST_CONF_ON
//  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
//#endif /* ENERGEST_CONF_ON */
//}
/*---------------------------------------------------------------------------*/
/** @} */
