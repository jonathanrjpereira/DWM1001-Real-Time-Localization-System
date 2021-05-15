/*
 * Simple Anchor / Tag configurator for a Real-Time Location System.
 * Author: Jonathan Pereira
 */

#include "dwm.h"
#include <stdio.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"


/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	int len;    
	int i;

	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_LOC_READY:
		printf("\nT:%lu ", dwm_systime_us_get());
		if (p_evt->loc.pos_available) {
			printf("POS:[%ld,%ld,%ld,%u] ", p_evt->loc.pos.x,
					p_evt->loc.pos.y, p_evt->loc.pos.z,
					p_evt->loc.pos.qf);
		} else {
			printf("POS:N/A ");
		}

		for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
			printf("DIST%d:", i);

			printf("0x%04X", (unsigned int)(p_evt->loc.anchors.dist.addr[i] & 0xffff));
			if (i < p_evt->loc.anchors.an_pos.cnt) {
				printf("[%ld,%ld,%ld]",
						p_evt->loc.anchors.an_pos.pos[i].x,
						p_evt->loc.anchors.an_pos.pos[i].y,
						p_evt->loc.anchors.an_pos.pos[i].z);
			}

			printf("=[%lu,%u] ", p_evt->loc.anchors.dist.dist[i],
					p_evt->loc.anchors.dist.qf[i]);
		}
		printf("\n");
		break;

	case DWM_EVT_USR_DATA_READY:
		len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
		if (len <= 0)
			break;

		printf("iot received, len=%d:", len);
		for (i = 0; i < len; ++i) {
			printf(" %02X", p_evt->usr_data[i]);
		}
		break;

	case DWM_EVT_USR_DATA_SENT:
		printf("iot sent\n");
		break;

	case DWM_EVT_BH_INITIALIZED_CHANGED:
		printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
		break;

	case DWM_EVT_UWBMAC_JOINED_CHANGED:
		printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
		break;

	default:
		break;
	}
}


void app_thread_entry(uint32_t data){

  /* Set the node type to either Anchor (0) or Tag (1) */
  int node_type = 1;
  int rv;

  /* Check and Verify the Firmware version of the module */
  dwm_ver_t ver;
  rv = dwm_ver_get(&ver);
  if (rv == DWM_OK){
    printf("FW Major: %d\n", ver.fw.maj);
    printf("FW Minor: %d\n", ver.fw.min);
    printf("FW Patch: %d\n", ver.fw.patch);
    printf("FW Res: %d\n", ver.fw.res);
    printf("FW Var: %d\n", ver.fw.var);
    printf("HW Version: %08x\n", ver.hw);
    printf("CFG Version: %08x\n", ver.cfg);
  }

  /* 
   * Configure the node as an Anchor
   * If the anchor is the first to join the network, set it as an Initiator
   * Enable Bluetooth
   * Set the UWB Mode to Active
   */
  if (node_type == 0){
    dwm_cfg_anchor_t set_a_cfg;
    set_a_cfg.initiator = 1;
    set_a_cfg.bridge = 0;
    set_a_cfg.common.enc_en = 1;
    set_a_cfg.common.led_en = 1;
    set_a_cfg.common.ble_en = 1;
    set_a_cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
    set_a_cfg.common.fw_update_en = 0;

    rv = dwm_cfg_anchor_set(&set_a_cfg);
    if(rv == DWM_OK){
      printf("Anchor Set role: %s\n", set_a_cfg.initiator == 1 ? "Initiator" : "Bridge");
      printf("Anchor Set Encryption: %s\n", set_a_cfg.common.enc_en == 0 ? "Disabled" : "Enabled");
      printf("Anchor Set LEDs: %s\n", set_a_cfg.common.led_en == 0 ? "Disabled" : "Enabled");
      printf("Anchor Set BLE: %s\n", set_a_cfg.common.ble_en == 0 ? "Disabled" : "Enabled");
      printf("Anchor Set UWB Mode: %s\n", set_a_cfg.common.uwb_mode == 0 ? "Off" : set_a_cfg.common.uwb_mode == 1 ? "Passive" : "Active");
      printf("Anchor Set FW Update: %s\n", set_a_cfg.common.fw_update_en == 0 ? "Disabled" : "Enabled");
    }
    else{
      printf("RV: %d\n", rv);
    }

    /* Verify if the node is configured as Anchor correctly */
    dwm_cfg_t get_a_cfg;
    dwm_cfg_get(&get_a_cfg);
    printf("Anchor Get mode %u \n", get_a_cfg.mode);
    printf("Anchor Get initiator %u \n", get_a_cfg.initiator);
    printf("Anchor Get bridge %u \n", get_a_cfg.bridge);
    printf("Anchor Get motion detection enabled %u \n", get_a_cfg.stnry_en);
    printf("Anchor Get measurement mode %u \n", get_a_cfg.meas_mode);
    printf("Anchor Get low power enabled %u \n", get_a_cfg.low_power_en);
    printf("Anchor Get internal location engine enabled %u \n", get_a_cfg.loc_engine_en);
    printf("Anchor Get encryption enabled %u \n", get_a_cfg.common.enc_en);
    printf("Anchor Get LED enabled %u \n", get_a_cfg.common.led_en);
    printf("Anchor Get BLE enabled %u \n", get_a_cfg.common.ble_en);
    printf("Anchor Get firmware update enabled %u \n", get_a_cfg.common.fw_update_en);
    printf("Anchor Get UWB mode %u \n", get_a_cfg.common.uwb_mode);
  }

  /* 
   * Configure the node as a Tag.
   * Set the Measurement mode to TWR.
   * Enable the Normal Update mode.
   * Enable Internal Location Engine.
   * Set the UWB Mode to Active.
   */
  else if (node_type == 1){
    dwm_cfg_tag_t set_t_cfg;
    set_t_cfg.stnry_en = 0;
    set_t_cfg.meas_mode = DWM_MEAS_MODE_TWR;
    set_t_cfg.low_power_en = 0;
    set_t_cfg.loc_engine_en = 1;
    set_t_cfg.common.enc_en = 0;
    set_t_cfg.common.led_en = 1;
    set_t_cfg.common.ble_en = 1;
    set_t_cfg.common.fw_update_en = 0;
    set_t_cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE;

    dwm_cfg_tag_set(&set_t_cfg);
    if(rv == DWM_OK){
      printf("Tag Set Stationary Detection: %s\n", set_t_cfg.stnry_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set Measurement Mode: %s\n", set_t_cfg.meas_mode == 0 ? "TWR" : "Reserved");
      printf("Tag Set Low Power Mode: %s\n", set_t_cfg.low_power_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set Internal Location Engine: %s\n", set_t_cfg.loc_engine_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set Encryption: %s\n", set_t_cfg.common.enc_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set LEDs: %s\n", set_t_cfg.common.led_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set BLE: %s\n", set_t_cfg.common.ble_en == 0 ? "Disabled" : "Enabled");
      printf("Tag Set UWB Mode: %s\n", set_t_cfg.common.uwb_mode == 0 ? "Off" : set_t_cfg.common.uwb_mode == 1 ? "Passive" : "Active");
      printf("Tag Set FW Update: %s\n", set_t_cfg.common.fw_update_en == 0 ? "Disabled" : "Enabled");
    }
    else{
      printf("RV: %d\n", rv);
    }

    /* Verify if the node is configured as Tag correctly */
    dwm_cfg_t get_t_cfg;
    dwm_cfg_get(&get_t_cfg);
    printf("Tag Get mode %u \n", get_t_cfg.mode);
    printf("Tag Get initiator %u \n", get_t_cfg.initiator);
    printf("Tag Get bridge %u \n", get_t_cfg.bridge);
    printf("Tag Get motion detection enabled %u \n", get_t_cfg.stnry_en);
    printf("Tag Get measurement mode %u \n", get_t_cfg.meas_mode);
    printf("Tag Get low power enabled %u \n", get_t_cfg.low_power_en);
    printf("Tag Get internal location engine enabled %u \n", get_t_cfg.loc_engine_en);
    printf("Tag Get encryption enabled %u \n", get_t_cfg.common.enc_en);
    printf("Tag Get LED enabled %u \n", get_t_cfg.common.led_en);
    printf("Tag Get BLE enabled %u \n", get_t_cfg.common.ble_en);
    printf("Tag Get firmware update enabled %u \n", get_t_cfg.common.fw_update_en);
    printf("Tag Get UWB mode %u \n", get_t_cfg.common.uwb_mode);
  }

  /* Set the PAN ID of all the nodes in the network to 0x0001 */ 
  uint16_t panid = 0x0001;
  rv = dwm_panid_set(panid);
  if (rv == DWM_OK){
    printf("PAN ID: %x\n" , panid);
  }
  else{
    printf("%d\n", rv);
  }

  /* Set the default position of the node */
  dwm_pos_t pos;
  pos.qf = 100;
  pos.x = 10;
  pos.y = 10;
  pos.z = 0;
  rv = dwm_pos_set(&pos);
  if(rv == DWM_OK){
    printf("Set Position Quality Factor: %d\n", pos.qf);
    printf("Set Position coordinates X: %d\n", pos.x);
    printf("Set Position coordinates Y: %d\n", pos.y);
    printf("Set Position coordinates Z: %d\n", pos.z);
  }
  else{
    printf("%d\n", rv);
  }

  /* Verify if the node position has been set correctly */
  dwm_pos_t gpos;
  rv = dwm_pos_get(&gpos);
  if (rv == DWM_OK){
    printf("Get Anchor Position [%d, %d, %d]\n", gpos.x, gpos.y, gpos.z);
  }
  else{
    printf("%d\n", rv);
  }

  printf("Configurations Completed\n");


  while(1){
    /* Read list of surrounding anchors in the network */
    if (node_type == 0){
      dwm_anchor_list_t list;
      int i;
      rv = dwm_anchor_list_get(&list);
      if (rv == DWM_OK){
        for (i = 0; i < list.cnt; ++i) {
          printf("%d. id=0x%04X pos=[%ld,%ld,%ld] rssi=%d seat=%u neighbor=%d\n", i,
          list.v[i].node_id,
          list.v[i].x,
          list.v[i].y,
          list.v[i].z,
          list.v[i].rssi,
          list.v[i].seat,
          list.v[i].neighbor_network);
        }
      }
      else{
        printf("RV: %d\n", rv);
      }
    }

    /* Read last measured distances to anchors (tag is currently ranging to) and the associated position */
    else if (node_type == 1){
      dwm_loc_data_t loc;
      int rv, i;
      rv = dwm_loc_get(&loc);
      if (rv == DWM_OK) {
        if (loc.pos_available) {
        }
        for (i = 0; i < loc.anchors.dist.cnt; ++i) {
          printf("0x%04x: ", loc.anchors.dist.addr[i]);
          printf("=%lu | ", loc.anchors.dist.dist[i]);
        }
        printf("\n");
      } 
      else {
        printf("err code: %d\n", rv);
      }
    }
  }
}


/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	//dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
