/**
+ * LEAPS - Low Energy Accurate Positioning System.
 *
 * DWM API application public interface
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#ifndef DWM_H_
#define DWM_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief DWM Error codes
 */
#define	DWM_OK				(0)
#define DWM_ERR_INVAL_ADDR	(-1)
#define DWM_ERR_INTERNAL	(-2)
#define DWM_ERR_INVAL_PARAM	(-3)
#define DWM_ERR_BUSY		(-4)
#define DWM_ERR_PERM		(-5)
#define DWM_ERR_OVERRUN		(-6)
#define DWM_ERR_I2C_ANACK	(-10)
#define DWM_ERR_I2C_DNACK	(-11)

/**
 * @brief Get system time
 *
 * @note Timer will overflow each 35 minutes
 *
 * @return Time in microseconds since restart
 */
uint32_t dwm_systime_us_get(void);

/**
 * @brief Resets DWM module
 */
void dwm_reset(void);

/**
 * @brief Puts node to factory settings. Environment is erased and set to
 * default state. Resets the node. This call does a write to internal flash,
 * hence should not be used frequently and can take in worst case
 * hundreds of milliseconds!
 */
void dwm_factory_reset(void);

/**
 * @brief Prevents entering of the sleep state (if in low power mode).
 * Should be called only from the thread context. Wakes up dwm_sleep().
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_PERM not in low power mode
 */
int dwm_wake_up(void);

/**
 * @brief Enables device entering the sleep state (if in low power mode).
 * The entering the sleep mode can be delayed internally if it is necessary, simply speaking,
 * there is no guarantee that device will enter sleep mode immediately after the call to dwm_sleep().
 * Should be called only from the thread context. Blocks until dwm_wake_up is called.
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_PERM not in low power mode
 */
int dwm_sleep(void);

/**
 * @brief DWM status
 */
typedef struct dwm_status_t {
	bool loc_data;
	bool uwbmac_joined;
	bool bh_data_ready;
	bool bh_status_changed;
	bool bh_initialized;
	bool uwb_scan_ready;
	bool usr_data_ready;
	bool usr_data_sent;
	bool fwup_in_progress;
} dwm_status_t;

/**
 * @brief Reads DWM status
 *
 * @param[out] p_status
 *
 * @return Error code
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_status_get(dwm_status_t* p_status);

/**
 * @brief Firmware version data
 */
typedef struct dwm_fw_ver_t {
	uint8_t maj;
	uint8_t min;
	uint8_t patch;
	uint8_t res;
	uint8_t var;
}dwm_fw_ver_t;

/**
 * @brief Version data
 */
typedef struct dwm_ver_t {
	dwm_fw_ver_t fw;
	uint32_t cfg;
	uint32_t hw;
} dwm_ver_t;

/**
 * @brief Get firmware version, configuration version and hardware
 * version of the module.
 *
 * @param[out] p_ver pointer to version data
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_INTERNAL failed to retrieve FW version
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_ver_get(dwm_ver_t* p_ver);

/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

#if defined(CFG_BH) && (CFG_BH == 1)
typedef enum {
	DWM_UWB_BH_ROUTING_OFF = 0,
	DWM_UWB_BH_ROUTING_ON = 1,
	DWM_UWB_BH_ROUTING_AUTO = 2
} dwm_uwb_bh_routing_t;
#endif /* CFG_BH */

typedef struct dwm_cfg_common {
	dwm_uwb_mode_t uwb_mode;
	bool fw_update_en;
	bool ble_en;
	bool led_en;
	bool enc_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
#if defined(CFG_BH) && (CFG_BH == 1)
	dwm_uwb_bh_routing_t uwb_bh_routing;
#endif /* CFG_BH */
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
#if defined(CFG_BH) && (CFG_BH == 1)
	dwm_uwb_bh_routing_t uwb_bh_routing;
#endif /* CFG_BH */
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

/**
 * @brief Configure node as anchor with given options. BLE option can’t be
 * enabled together with encryption otherwise the configuration
 * is considered invalid and it is refused. This call requires reset for new
 * configuration to take effect (dwm_reset). Enabling encryption on initiator
 * will cause automatic enabling of encryption of all nodes that have the same
 * encryption key set (dwm_enc_key_set). This allows to enable encryption for
 * whole network that has the same pan ID (network ID) and the same encryption
 * key remotely from one place.
 * Encryption can’t be enabled if encryption key is not set.
 * This call do a write to internal flash in case of new value being set,
 * hence should not be used frequently and can take in worst case hundreds
 * of milliseconds!
 *
 * @param[in] p_cfg Anchor configuration options
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to set configuration
 * @retval DWM_ERR_INVAL_PARAM Invalid UWB mode
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 * @retval DWM_ERR_PERM Invalid combination of configuration options or
 * trying to enable encryption with no key
 */
int dwm_cfg_anchor_set(dwm_cfg_anchor_t* p_cfg);

/**
 * @brief Configure node as tag with given options. BLE option can’t be enabled
 * together with encryption otherwise the configuration is considered
 * invalid and it is refused. This call requires reset for new configuration to
 * take effect (dwm_reset).
 * Encryption can’t be enabled if encryption key is not set.
 * This call do a write to internal flash in case of
 * new value being set, hence should not be used frequently and can take in worst
 * case hundreds of milliseconds!
 *
 * @param[in] p_cfg Tag configuration options
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to set configuration
 * @retval DWM_ERR_INVAL_PARAM Invalid UWB mode or measurement mode
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 * @retval DWM_ERR_PERM Invalid combination of configuration options or trying
 * to enable encryption with no key
 */
int dwm_cfg_tag_set(dwm_cfg_tag_t* p_cfg);

/**
 * @brief Reads configuration of the node
 *
 * @param[out] p_cfg Node configuration
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to read configuration
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_cfg_get(dwm_cfg_t* p_cfg);

typedef enum {
	DWM_STNRY_SENSITIVITY_LOW = 0,
	DWM_STNRY_SENSITIVITY_NORMAL = 1,
	DWM_STNRY_SENSITIVITY_HIGH = 2
} dwm_stnry_sensitivity_t;

/**
 * @brief Writes configuration of the stationary mode which is used by tag node. The configuration
 * can be written even if stationary detection is disabled (see dwm_cfg_tag_set). Writes internal non-volatile memory
 * so should be used carefully. New sensitivity setting takes effect immediately if stationary mode is enabled. Default
 * sensitivity is “HIGH”.
 *
 * @param[in] sensitivity Stationary sensitivity
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to write configuration
 * @retval DWM_ERR_INVAL_PARAM unknown sensitivity level
 */
int dwm_stnry_cfg_set(dwm_stnry_sensitivity_t sensitivity);

/**
 * @brief Reads configuration of the stationary mode which is used by tag node. The
 * configuration can be read even if stationary detection is disabled.
 *
 * @param[out] p_sensitivity Pointer to stationary sensitivity
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to read configuration
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_stnry_cfg_get(dwm_stnry_sensitivity_t* p_sensitivity);

/**
 * @brief GPIO pins available for the user application
 */
typedef enum {
	DWM_GPIO_IDX_2 = 2,  //!< DWM_GPIO_IDX_2
	DWM_GPIO_IDX_8 = 8,  //!< DWM_GPIO_IDX_8
	DWM_GPIO_IDX_9 = 9,  //!< DWM_GPIO_IDX_9
	DWM_GPIO_IDX_10 = 10,  //!< DWM_GPIO_IDX_10
	DWM_GPIO_IDX_12 = 12,  //!< DWM_GPIO_IDX_12
	DWM_GPIO_IDX_13 = 13,  //!< DWM_GPIO_IDX_13
	DWM_GPIO_IDX_14 = 14,  //!< DWM_GPIO_IDX_14
	DWM_GPIO_IDX_15 = 15,  //!< DWM_GPIO_IDX_15
	DWM_GPIO_IDX_22 = 22,  //!< DWM_GPIO_IDX_22
	DWM_GPIO_IDX_23 = 23,  //!< DWM_GPIO_IDX_23
	DWM_GPIO_IDX_27 = 27,  //!< DWM_GPIO_IDX_27
	DWM_GPIO_IDX_30 = 30,  //!< DWM_GPIO_IDX_30
	DWM_GPIO_IDX_31 = 31 //!< DWM_GPIO_IDX_31
} dwm_gpio_idx_t;

/**
 * @brief Enumerator used for selecting the pin to be pulled down
 * or up at the time of pin configuration
 */
typedef enum {
	DWM_GPIO_PIN_NOPULL = 0,
	DWM_GPIO_PIN_PULLDOWN = 1,
	DWM_GPIO_PIN_PULLUP = 3
} dwm_gpio_pin_pull_t;

/**
 * @brief Interrupt type for GPIO pins
 */
typedef enum
{
	DWM_IRQ_TYPE_EDGE_RISING = 1,
	DWM_IRQ_TYPE_EDGE_FALLING = 2,
	DWM_IRQ_TYPE_EDGE_BOTH = 3
} dwm_gpio_irq_type_t;

/**
 * @brief Configures pin as output and sets the pin value
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value Initial value of the output pin
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_cfg_output(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Configures pin as input and sets pull mode
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] pull_mode Pull mode of the input pin (see dwm_gpio_pin_pull_t)
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application or pull mode is not defined
 */
int dwm_gpio_cfg_input(dwm_gpio_idx_t idx, dwm_gpio_pin_pull_t pull_mode);

/**
 * @brief Sets value of the output pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value Pin value (0, 1)
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_value_set(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Gets value of the input pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[out] p_value Pointer to the pin value
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_gpio_value_get(dwm_gpio_idx_t idx, bool* p_value);

/**
 * @brief Toggles value of the output pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_value_toggle(dwm_gpio_idx_t idx);

/**
 * @brief Interrupt callback
 *
 * @param[in] p_data User data
 */
typedef void dwm_gpio_cb_t(void* p_data);

/**
 * @brief Registers interrupt for GPIO pin
 *
 * @note callback is called in interrupt context
 *
 * @param[in] idx Pin index
 * @param[in] irq_type Interrupt type
 * @param[in] p_cb Pointer to interrupt callback
 * @param[in] p_data User data
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM pin is reserved, not available for application
 * @retval DWM_ERR_INTERNAL not possible to register pin, probably maximum number of registered pins has been reached
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_gpio_irq_cfg(dwm_gpio_idx_t idx, dwm_gpio_irq_type_t irq_type, dwm_gpio_cb_t* p_cb, void* p_data);

/**
 * @brief Disable interrupt on the GPIO pin
 *
 * @param[in] idx Pin index
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL pin is not registered
 * @retval DWM_ERR_INVAL_PARAM pin is reserved, not available for application
 */
int dwm_gpio_irq_dis(dwm_gpio_idx_t idx);

/* BLE address length */
#define DWM_BLE_ADDR_LEN 6

/**
 * @brief BLE address
 */
typedef struct {
	uint8_t byte[DWM_BLE_ADDR_LEN];
}dwm_baddr_t;

/**
 * @brief Sets BLE address
 *
 * @param[in] p_baddr Pointer to BLE address structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL address can't be set
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_baddr_set(dwm_baddr_t* p_baddr);

/**
 * @brief Gets BLE address
 *
 * @param[out] p_baddr Pointer to BLE address structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL address can't be read
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_baddr_get(dwm_baddr_t* p_baddr);

/* encryption key length in bytes */
#define DWM_ENC_KEY_LEN 16

/**
 * @brief encryption key
 */
typedef struct {
	uint8_t byte[DWM_ENC_KEY_LEN];
}dwm_enc_key_t;

/**
 * @brief Sets encryption key. The key is stored in nonvolatile memory.
 * he key that consists of just zeros is considered as invalid. If key is set,
 * the node can enable encryption automatically. Automatic enabling of the
 * encryption is triggered via UWB network when the node detects encrypted
 * message and is capable of decrypting the messages with the key. BLE option
 * is disabled when encryption is enabled automatically. The encryption can
 * be disabled by clearing the key (dwm_enc_key_clear).
 * This call writes to internal flash in case of new value being set, hence
 * should not be used frequently and can take in worst case hundreds of
 * milliseconds! Requires reset for new configuration to take effect.
 *
 * @param[in] p_key Pointer to encryption key structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL key can't be set or key is just all zeros
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_enc_key_set(dwm_enc_key_t* p_key);

/**
 * @brief Clears the encryption key and disables encryption option if
 * enabled. Does nothing if the key is not set. This call writes to
 * internal flash in case of new value being set, hence should not be used
 * frequently and can take in worst case hundreds of milliseconds! Requires
 * reset for new configuration to take effect.
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL can't be cleared
 */
int dwm_enc_key_clear(void);

/**
 * @brief Position coordinates in millimeters + quality factor
 */
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;

/**
 * @brief Sets position of anchor node
 *
 * @param[in] p_pos Position coordinates
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be written
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 * @retval DWM_ERR_INVAL_PARAM invalid position, e.g quality factor > 100%
 */
int dwm_pos_set(dwm_pos_t* p_pos);

/**
 * @brief Gets position of the node
 *
 * @param[out] p_pos Pointer to position
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be read
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_pos_get(dwm_pos_t* p_pos);

/**
 * @brief Sets position of anchor node, can be used when one does not
 * want to set all coordinates
 *
 * @param[in] p_x Pointer to x coordinate (millimeters), ignored if null
 * @param[in] p_y Pointer to y coordinate (millimeters), ignored if null
 * @param[in] p_z Pointer to z coordinate (millimeters), ignored if null
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be read
 */
int dwm_pos_set_xyz(int32_t* p_x, int32_t* p_y, int32_t* p_z);

/* Maximum number of neighbor anchors */
#define DWM_RANGING_ANCHOR_CNT_MAX	14
#define DWM_ANCHOR_LIST_CNT_MAX		30

/**
 * @brief Distances of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	uint16_t addr[DWM_RANGING_ANCHOR_CNT_MAX];
	uint32_t dist[DWM_RANGING_ANCHOR_CNT_MAX];
	uint8_t qf[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_distance_t;

/**
 * @brief Position of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_pos_t pos[DWM_RANGING_ANCHOR_CNT_MAX];
}dwm_anchor_pos_t;

/**
 * @brief Distances and position of ranging anchors
 */
typedef struct {
	dwm_distance_t dist;
	dwm_anchor_pos_t an_pos;
}dwm_ranging_anchors_t;

/**
 * @brief Location data (position of current node and list of positions
 * and distances of ranging anchors)
 */
typedef struct dwm_loc_data_t {
	bool pos_available;
	dwm_pos_t pos;
	dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;

/**
 * @brief Gets location data
 *
 * @param[out] p_loc Pointer to location data, if NULL function returns
 * with no error and call is ignored
 *
 * @note if p_loc.p_pos is NULL the location data are not read without returning an error
 *
 * @note if p_loc.p_pos is NULL the position is not read and no error is returned
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL can't read location data or there are none
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_loc_get(dwm_loc_data_t* p_loc);

/**
 * @brief Element of anchor list
 */
typedef struct {
	uint16_t node_id;
	int32_t x,y,z;
	int8_t rssi;
	uint8_t seat;
	bool neighbor_network;
} dwm_anchor_list_elt_t;

/**
 * @brief List of anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_anchor_list_elt_t v[DWM_ANCHOR_LIST_CNT_MAX];
} dwm_anchor_list_t;

/**
 * @brief Reads list of surrounding anchors. Works for anchors only.
 * Anchors in the list can be from the same network or from
 * the neighbor network as well.
 *
 * @param[out] p_list
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_ADDR null pointer
 * @retval DWM_ERR_INTERNAL internal error
 */
int dwm_anchor_list_get(dwm_anchor_list_t* p_list);

/* maximum and minimum update rate in multiple of 100 ms */
enum dwm_upd_rate{
	DWM_UPD_RATE_MAX = 600,	/* 1 minute */
	DWM_UPD_RATE_MIN = 1	/* 100 ms */
};

/**
 * @brief Sets update rate, update rate is relevant for Tag node
 *
 * @param[in] ur Update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[in] urs Stationary update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't write the value
 * @retval DWM_ERR_INVAL_PARAM Values not within the limits
 */
int dwm_upd_rate_set(const uint16_t ur, const uint16_t urs);

/**
 * @brief Gets update rate
 *
 * @param[out] p_ur Pointer to update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[out] p_urs Pointer to stationary update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read update rate
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_upd_rate_get(uint16_t* p_ur, uint16_t* p_urs);

/**
 * @brief Sets PAN ID
 *
 * @param[in] panid The id
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't set PAN ID
 */
int dwm_panid_set(uint16_t panid);

/**
 * @brief Gets PAN ID
 *
 * @param[out] p_panid Pointer to the PAN ID
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read PAN ID
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_panid_get(uint16_t *p_panid);

/**
 * @brief Gets UWB address/ID of the node
 *
 * @param[out] p_node_id Pointer to the node ID
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read node ID
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_node_id_get(uint64_t *p_node_id);

#define DWM_UWB_SCAN_RESULT_CNT_MAX 16

/**
 * @brief UWB scan result data
 */
typedef struct {
	uint8_t cnt;
	uint8_t mode[DWM_UWB_SCAN_RESULT_CNT_MAX];
	int8_t rssi[DWM_UWB_SCAN_RESULT_CNT_MAX];
} dwm_uwb_scan_result_t;

/**
 * @brief Starts UWB scan
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 */
int dwm_uwb_scan_start(void);

/**
 * @brief Read latest UWB scan result
 *
 * @param[out] p_result The scan result data pointer
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_uwb_scan_result_get(dwm_uwb_scan_result_t *p_result);

/**
 * @brief UWB configuration parameters
 */
typedef struct {
	uint8_t pg_delay;
	uint32_t tx_power;
	struct {
		uint8_t pg_delay;
		uint32_t tx_power;
	} compensated;
} dwm_uwb_cfg_t;

/**
 * @brief Sets UWB configuration parameters
 *
 * @param[in] p_cfg Configuration parameters
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_uwb_cfg_set(dwm_uwb_cfg_t *p_cfg);

/**
 * @brief Gets UWB configuration parameters
 *
 * @param[out] p_cfg Configuration parameters
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't get configuration
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_uwb_cfg_get(dwm_uwb_cfg_t *p_cfg);

/* valid UWB preamble codes */
typedef enum {
	DWM_UWB_PRAMBLE_CODE_9 = 9,
	DWM_UWB_PRAMBLE_CODE_10 = 10,
	DWM_UWB_PRAMBLE_CODE_11 = 11,
	DWM_UWB_PRAMBLE_CODE_12 = 12
} dwm_uwb_preamble_code_t;

/**
 * @brief Sets UWB preamble code
 *
 * @param[in] code One of the preamble codes in dwm_uwb_preamble_code_t
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INTERNAL DWM internal error
 * @retval DWM_ERR_INVAL_PARAM Invalid preamble code
 */
int dwm_uwb_preamble_code_set(dwm_uwb_preamble_code_t code);

/**
 * @brief Gets UWB preamble code
 *
 * @param[out] p_code Pointer to the preamble code
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 * @retval DWM_ERR_INTERNAL DWM internal error
 */
int dwm_uwb_preamble_code_get(dwm_uwb_preamble_code_t *p_code);

/**
 * @brief Read data from I2C slave.
 *
 * @param[in] addr Address of a slave device (only 7 LSB)
 * @param[out] p_data Pointer to a receive buffer
 * @param[in] len Number of bytes to be received
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_BUSY     If the driver is not ready for a new transfer.
 * @retval DWM_ERR_INTERNAL      If an error was detected by hardware.
 * @retval DWM_ERR_I2C_ANACK If NACK received after sending the address.
 * @retval DWM_ERR_I2C_DNACK If NACK received after sending a data byte.
 * @retval DWM_ERR_OVERRUN If the unread data was replaced by new data.
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_i2c_read(uint8_t addr, uint8_t* p_data, uint8_t len);

/**
 * @brief Write data to I2C slave.
 *
 * @param[in] addr Address of a slave device (only 7 LSB)
 * @param[in] p_data Pointer to a transmit buffer
 * @param[in] len Number of bytes to be send
 * @param[in] no_stop If set, the stop condition is not generated
 * on the bus after the transfer has completed successfully
 * (allowing for a repeated start in the next transfer)
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_BUSY      If the driver is not ready for a new transfer.
 * @retval DWM_ERR_INTERNAL       If an error was detected by hardware.
 * @retval DWM_ERR_I2C_ANACK If NACK received after sending the address.
 * @retval DWM_ERR_I2C_DNACK If NACK received after sending a data byte.
 * @retval DWM_ERR_INVAL_ADDR If the EasyDMA is used and memory address in not in RAM.
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_i2c_write(uint8_t addr, uint8_t* p_data, uint8_t len, bool no_stop);

#define DWM_USR_DATA_LEN_MAX	34

/**
 * @brief Read user data received from the network.
 *
 * @param[out] p_data Pointer to a receive buffer
 * @param[in, out] p_len Pointer to number of bytes, if provided
 * length is not enough(DWM_ERR_OVERRUN) then this parameter contains expected
 * length DWM_USR_DATA_LEN_MAX
 *
 * @return Error code
 * @retval 0 Success
 * @retval DWM_ERR_INTERNAL Internal error
 * @retval DWM_ERR_OVERRUN If length provided is not enough.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_usr_data_read(uint8_t* p_data, uint8_t* p_len);

/**
 * @brief Nonblocking write of user data to the bridge node.
 *
 * @param[in] p_data Pointer to the user data
 * @param[in] len Number of bytes to be written
 * @param[in] overwrite If true, might overwrite recent data
 *
 * @return Error code
 * @retval len Success
 * @retval DWM_ERR_BUSY If overwrite == false and the device is busy
 * handling last request.
 * @retval DWM_ERR_INTERNAL No backhaul available
 * @retval DWM_ERR_INVAL_PARAM If length exceeds DWM_USR_DATA_LEN_MAX.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_usr_data_write(uint8_t* p_data, uint8_t len, bool overwrite);

#define DWM_LABEL_LEN_MAX	16

/**
 * @brief Reads label from the node
 *
 * @param[out] p_label Pointer to label buffer
 * @param[in, out] p_len Pointer to number of bytes, if provided
 * length is not enough(DWM_ERR_OVERRUN) then this parameter contains expected
 * length DWM_LABEL_LEN_MAX
 *
 * @return Error code
 * @retval 0 Success
 * @retval DWM_ERR_INTERNAL Internal error
 * @retval DWM_ERR_OVERRUN If length provided is not enough.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_label_read(uint8_t* p_label, uint8_t* p_len);

/**
 * @brief Writes label to the node, nonblocking.
 *
 * @param[in] p_label Pointer to label bytes
 * @param[in] len Number of bytes to be written
 *
 * @return Error code
 * @retval len Success
 * @retval DWM_ERR_INTERNAL Internal error
 * @retval DWM_ERR_INVAL_PARAM If length exceeds DWM_LABEL_LEN_MAX.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_label_write(uint8_t* p_label, uint8_t len);

#define DWM_NVM_USR_DATA_LEN_MAX 250

/**
 * @brief Reads user data from non-volatile memory. Reads from internal flash.
 *
 * @param[out] p_data Pointer to data
 * @param[in, out] p_len Pointer to number of bytes, if provided
 * length is not enough(DWM_ERR_OVERRUN) then this parameter contains expected
 * length DWM_NVM_USR_DATA_LEN_MAX
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Internal error
 * @retval DWM_ERR_OVERRUN If length provided is not enough.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_nvm_usr_data_get(uint8_t* p_data, uint8_t* p_len);

/**
 * @brief Stores user data to non-volatile memory. Writes internal non-volatile
 * memory so should be used carefully. Old data are rewritten.
 *
 * @param[in] p_label Pointer to label bytes
 * @param[in] len Number of bytes to be written
 *
 * @return Error code
 * @retval len Success
 * @retval DWM_ERR_INTERNAL Internal error
 * @retval DWM_ERR_INVAL_PARAM If length exceeds DWM_NVM_USR_DATA_LEN_MAX.
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 */
int dwm_nvm_usr_data_set(uint8_t* p_data, uint8_t len);

/**
 * @brief DWM events IDs
 */
typedef enum {
	DWM_EVT_LOC_READY				= 1,
	DWM_EVT_UWBMAC_JOINED_CHANGED	= 2,
	DWM_EVT_BH_INITIALIZED_CHANGED	= 16,
	DWM_EVT_UWB_SCAN_READY			= 32,	/* TODO */
	DWM_EVT_USR_DATA_READY			= 64,
	DWM_EVT_USR_DATA_SENT			= 128
/* new events TBD*/
}dwm_evt_id_t;

/**@brief DWM Event header. */
typedef struct {
	dwm_evt_id_t id;
	void *p_context;
	uint16_t len;	/* event data length including header*/
} dwm_evt_hdr_t;

/**@brief Common DWM Event type, wrapping the module specific event reports. */
typedef struct {
	dwm_evt_hdr_t header;
	union {
		dwm_loc_data_t loc;
		uint8_t usr_data[DWM_USR_DATA_LEN_MAX];
		dwm_uwb_scan_result_t uwb_scan;
		bool bh_initialized;
		bool uwbmac_joined;
	};
} dwm_evt_t;

/**
 * @brief Registers callback for new location data, should be called once
 *
 * @param[in] evt_id_map IDs of events that are to be listened to
 * @param[in] p_context User context data
 */
void dwm_evt_listener_register(uint32_t evt_id_map, void* p_context);

/**
 * @brief Sleeps until new event
 *
 * @param[out] p_evt The event
 *
 * @return Error code 0 if success
 * @retval DWM_ERR_INVAL_ADDR Null pointer supplied
 * @retval DWM_ERR_OVERRUN Event buffer overflow
 */
int dwm_evt_wait(dwm_evt_t *p_evt);

/* @brief interrupt handler prototype */
typedef void dwm_irq_hndlr_t(void *p_context);

typedef struct {
	uint8_t vector_nr;
	dwm_irq_hndlr_t *p_hndlr;
	void *p_context; /* can be NULL*/
} dwm_irq_t;

/**
 * @brief Registers interrupt
 *
 * @param[in] p_irq Pointer to DWM interrupt structure
 *
 * @return Error code 0 if success
 * @retval DWM_ERR_INVAL_ADDR p_hndlr is NULL pointer
 */
int dwm_interrupt_register(dwm_irq_t *p_irq);

/**
 * @brief Masks particular interrupt source
 *
 * @param[in] vector_nr Interrupt vector number
 */
void dwm_interrupt_mask(uint8_t vector_nr);

/**
 * @brief Unmasks particular interrupt source
 *
 * @param[in] vector_nr Interrupt vector number
 */
void dwm_interrupt_unmask(uint8_t vector_nr);

/**
 * @brief thread entry function prototype
 * */
typedef void dwm_thread_entry_t(uint32_t);

/**
 * @brief Creates a new thread, at most 5 threads are allowed
 *
 * @param[in] prio Thread priority, values from interval <20,29> are allowed
 * @param[in] entry Thread entry function
 * @param[in] p_data User data
 * @param[in] name Thread name
 * @param[in] stack_size Stack size in bytes
 * @param[out] p_hndl Thread handle
 *
 * @return Error code 0 if success
 * @retval DWM_ERR_INTERNAL maximum number of threads has been reached
 * @retval DWM_ERR_INVAL_PARAM invalid priority
 */
int dwm_thread_create(uint8_t prio, dwm_thread_entry_t *entry, void* p_data,
		char *name, uint16_t stack_size, uint8_t* p_hndl);

/**
 * @brief Resumes suspended thread
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 */
int dwm_thread_resume(uint8_t hndl);

/**
 * @brief Suspends thread
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Invalid handle
 */
int dwm_thread_suspend(uint8_t hndl);

/**
 * @brief Deletes the thread, free the stack memory
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Invalid handle
 * @retval DWM_ERR_INTERNAL Thread was not deleted
 */
int dwm_thread_delete(uint8_t hndl);

/**
 * @brief Allows a thread to suspend until the specified number of clock
 * ticks have occurred
 *
 * @param[in[ delay Delay in clock ticks, 1 tick is 10 ms
 */
void dwm_thread_delay(uint64_t delay);

/* mutex data type */
typedef uint8_t dwm_mutex_t[20];

/**
 * @brief Initializes mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_init(dwm_mutex_t* p_mutex);

/**
 * @brief Locks the mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_lock(dwm_mutex_t* p_mutex);

/**
 * @brief Unlocks the mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_unlock(dwm_mutex_t* p_mutex);

/* condition variable data type */
typedef uint8_t dwm_cond_t[8];

/**
 * @brief Initializes the condition variable. A condition
 * variable is attached to a specific mutex.
 *
 * @param[in] p_cond Pointer to condition variable
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_cond_init(dwm_cond_t* p_cond, dwm_mutex_t* p_mutex);

/**
 * @brief Causes the current thread to wait on the condition variable, while
 * simultaneously unlocking the corresponding mutex. dwm_cond_wait() may be
 * called by a thread which has the corresponding mutex locked.
 *
 * @param[in] p_cond Pointer to condition variable
 * @return "true" if no error, "false" otherwise
 */
bool dwm_cond_wait(dwm_cond_t* p_cond);

/**
 * @brief Wakes up at least one thread which is waiting on the condition variable
 * . When a thread is awakened it will become the owner of the mutex. dwm_cond_signal()
 * may be called by the thread which currently owns the mutex to which the condition
 * variable is attached.
 *
 * @param[in] p_cond Pointer to condition variable
 */
void dwm_cond_signal(dwm_cond_t* p_cond);

/**
 * @brief Destroys the condition variable. This must not be done on a condition
 * variable which is in use. After it has been destroyed, it may be subsequently
 * reinitialized.
 *
 * @param[in] p_cond Pointer to condition variable
 */
void dwm_cond_destroy(dwm_cond_t* p_cond);

/* semaphore data type */
typedef uint8_t dwm_sem_t[8];

/**
 * @brief Initializes a semaphore. The initial semaphore count is set to val.
 *
 * @param[in] p_sem Pointer to the semaphore
 * @param[in] val Initial value of the semaphore
 */
void dwm_semaphore_init(dwm_sem_t* p_sem, uint32_t val);

/**
 * @brief If the semaphore count is zero, the current thread will wait on the
 *  semaphore. If the count is non-zero, it will be decremented and the
 *  thread will continue running.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_wait(dwm_sem_t* p_sem);

/**
 * @brief If there are threads waiting on this semaphore this will wake
 * exactly one of them. Otherwise it simply increments the semaphore count.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_post(dwm_sem_t* p_sem);

/**
 * @brief Destroys a semaphore. This must not be done while there are any
 * threads waiting on it.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_destroy(dwm_sem_t* p_sem);

/* handle data type */
typedef int32_t dwm_hndl_t;

/* mailbox data type */
typedef uint8_t dwm_mbox_t[56];

/**
 * @brief Creates a mailbox
 *
 * @param[out] p_hndl Handle object
 * @param[in] p_mbox Mailbox object
 */
void dwm_mbox_create(dwm_hndl_t* p_hndl, dwm_mbox_t* p_mbox);

/**
 * @brief Deletes a mailbox
 *
 * @param[in] hndl Handle object
 */
void dwm_mbox_delete(dwm_hndl_t hndl);

/**
 * @brief This places a message into an mailbox. If the mailbox is already full
 * this function will block until the message is placed into the mbox. If the thread
 * is awaken by the kernel during a wait, this function will return an error.
 *
 * @param[in] hndl Handle object
 * @param[in] p_item Pointer to be placed in the mailbox
 *
 * @return "true" if the message was placed into the mailbox, "false" otherwise
 */
bool dwm_mbox_put(dwm_hndl_t hndl, void* p_item);

/**
 * @brief Reads number of messages in mailbox
 *
 * @param[in] hndl Handle object
 *
 * @return Error code The number of messages currently in the given mailbox.
 */
uint32_t dwm_mbox_peek(dwm_hndl_t hndl);

/**
 * @brief This reads a pointer from an mailbox. If the mailbox has no data in it, this
 * function will block the calling thread until data is available.
 *
 * @param[in] hndl Handle object
 *
 * @return Pointer to data that was placed in the mailbox.
 */
void* dwm_mbox_get(dwm_hndl_t hndl);

/**
 * @brief Enables BLE for compilation
 */
void dwm_ble_compile(void);

/**
 * @brief Enables location engine for compilation
 */
void dwm_le_compile(void);

/**
 * @brief Enables API on SPI, UART interfaces for compilation
 */
void dwm_serial_uart_compile(void);
void dwm_serial_spi_compile(void);

/**
 * @brief Enables SHELL for compilation
 */
void dwm_shell_compile(void);

#endif /* DWM_H_ */
