/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

#include "retargetserial.h"
#include <stdio.h>
#include "em_rmu.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[8192]; //DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

/* Gecko configuration parameters (see gecko_configuration.h) */
static gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

int16_t txPower = 30;

void test_buram(void) {
	  for(int i = 0; i < 32; i++) {
		  printf("%08lx%s",RTCC->RET[i].REG,((i&7)==7)?"\n":" ");
		  RTCC->RET[i].REG = 0;
	  }
}

#define AZ(X) do { uint16_t rc = X->result; if(rc) { printf("%s failed.  result:%04x\nresetting...\n\n",#X,rc); gecko_cmd_system_reset(0); }} while(0)
int main(void) {
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  /* Initialize peripherals */
  enter_DefaultMode_from_RESET();

  RETARGET_SerialInit();
  RETARGET_SerialCrLf(1);
  printf("Hello, World!\n");
  uint32_t reason = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
  printf("Reason: %08x\n",reason);
  if(1 == reason) test_buram();
  //test_buram();

  uint32_t reg;
  int flag = 1;
  if (0xdead == (RTCC->RET[0].REG >> 16)) {
	  config.bluetooth.heap_size = RTCC->RET[0].REG;
	  printf("Setting config.bluetooth.heap_size = %d\n",config.bluetooth.heap_size);
	  flag = 0;
  }
  if (0xbeef == (RTCC->RET[1].REG >> 16)) {
	  txPower = RTCC->RET[1].REG & 0xffff;
	  printf("Setting txPower = %d\n",txPower);
	  flag = 0;
  }

  /* Initialize stack */
  gecko_init(&config);



  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

    	  txPower = gecko_cmd_system_set_tx_power(txPower)->set_power;
    	  printf("gecko_cmd_system_set_tx_power(txPower)->set_power: %d\n",txPower);

    	  if(flag) {
    		  gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name,0,14,(void*)"Hello, World!!");
    		  flag = 0;
    	  }

    	  /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        AZ(gecko_cmd_le_gap_set_adv_parameters(160, 160, 7));

        /* Start general advertising and enable connections. */
        AZ(gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable));
        break;

      case gecko_evt_le_connection_closed_id:
    	  if(flag) {
    		  NVIC_SystemReset();
    	  }
    	  /* Restart advertising after client has disconnected */
    	  AZ(gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable));
    	  break;

      case gecko_evt_gatt_server_user_write_request_id:
#define ED evt->data.evt_gatt_server_user_write_request
        switch (ED.characteristic) {
        case gattdb_heap_size:
        	memcpy(&reg,&ED.value.data[0],ED.value.len);
        	RTCC->RET[0].REG = (0xdead << 16) | (reg & 0xffff);
        	printf("RTCC->RET[0].REG set to %08lx\n",RTCC->RET[0].REG);
        	flag = 1;
        	break;
        case gattdb_tx_power:
        	memcpy(&reg,&ED.value.data[0],ED.value.len);
        	RTCC->RET[1].REG = (0xbeef << 16) | (reg & 0xffff);
        	printf("RTCC->RET[1].REG set to %08lx\n",RTCC->RET[1].REG);
        	flag = 1;
        	break;
        }
        AZ(gecko_cmd_gatt_server_send_user_write_response(ED.connection, ED.characteristic, bg_err_success));
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
