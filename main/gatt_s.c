/**
 *   @file name     gatt_s.c
 *   Project:       Motion_Tracking
 *   @Purpose       GATT_S source file
 *   Author:        TaiVV2@fsoft.com.vn
 *   Final editors: SangTN@fsoft.com.vn
 *   @date:         07-30-2019
 */
/*==================================================================================================
 *                                        INCLUDE FILES
 ==================================================================================================*/
#include "gatt_s.h"
#include "ExternVariablesFunctions.h"
/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
static void bt_init(void);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/*==================================================================================================
*                                      VARIABLES
==================================================================================================*/
_tracking_variables tracking_variables_gtr;
_motion_tracking_flag motion_tracking_flag_gstr;
_receive_from_app receive_from_app_gtr;
static uint8_t adv_config_done = 0;
//uint8_t flag_gatts_u8 = 0;

uint16_t motion_tracking_handle_table[6];

/*Major: BATTERY LEVEL*/
uint8_t a = 0x34;
/*MINOR _ WORKING MODE*/
uint8_t b = 0x35;

uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] = { 0x4C, 0x00, // # Company identifier code (0x004C == Apple)
        0x02, // # Byte 0 of iBeacon advertisement indicator
        0x15, // # Byte 1 of iBeacon advertisement indicator
        0xe2, 0xc5, 0x6d, 0xb5, 0xdf, 0xfb, 0x48, 0xd2, 0xb0, 0x60, 0xd0, 0xf5, 0xa7, 0x10, 0x96, 0xe0, //# iBeacon proximity uuid
        0x00, // # major
        0x00, // # minor
        };

static uint8_t sec_service_uuid[16] = {
/* LSB <--------------------------------------------------------------------------------> MSB */
/* first uuid, 16bit, [12],[13] is the value */
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00, };

/* config adv data */
static esp_ble_adv_data_t motion_tracking_adv_config = {
        .set_scan_rsp = false,
        .include_txpower = true,
        .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = 0x00,
        .manufacturer_len = TEST_MANUFACTURER_DATA_LEN,
        .p_manufacturer_data = &test_manufacturer[0],
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(sec_service_uuid),
        .p_service_uuid = sec_service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };

/* config scan response data */
static esp_ble_adv_data_t motion_tracking_scan_rsp_config = {
        .set_scan_rsp = true,
        .include_name = true,
        .manufacturer_len = sizeof(test_manufacturer),
        .p_manufacturer_data = test_manufacturer,
};

static esp_ble_adv_params_t motion_tracking_adv_params = {
        .adv_int_min = 0x100,
        .adv_int_max = 0x100,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst motion_tracking_profile_tab[MOTION_TRACKING_PROFILE_NUM] = {
        [MOTION_TRACKING_APP_IDX] = {
                .gatts_cb = gatts_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        },
};

/*==================================================================================================
*                                      Motion_Tracking PROFILE ATTRIBUTES
==================================================================================================*/

/* MPU6050 Sensor Service */
static const uint16_t motion_tracking_svc = MOTION_TRACKING_SERVICE_UUID;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;

static const uint16_t motion_tracking_notify = MOTION_TRACKING_CHAR_N_UUID;
static const uint8_t motion_tracking_notify_data[2];
static const uint16_t motion_tracking_rw = MOTION_TRACKING_CHAR_RW_UUID;
static const uint8_t motion_tracking_rw_data[1];

static const esp_gatts_attr_db_t motion_tracking_gatt_db[6] = {
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &primary_service_uuid,
          ESP_GATT_PERM_READ,
          sizeof(uint16_t), //16
          sizeof(motion_tracking_svc),
          (uint8_t *) &motion_tracking_svc }
        },

        /* Characteristic Declaration */
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &character_declaration_uuid,
          ESP_GATT_PERM_READ,
          CHAR_DECLARATION_SIZE,
          CHAR_DECLARATION_SIZE,
          (uint8_t *) &char_prop_notify }
        },

        //  Characteristic Value
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &motion_tracking_notify,
          ESP_GATT_PERM_READ,
          HRPS_HT_MEAS_MAX_LEN,
          0,
          NULL }
        },

        /* Client Characteristic Configuration Descriptor */
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &character_client_config_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
          sizeof(uint16_t),
          sizeof(motion_tracking_notify_data),
          (uint8_t *) motion_tracking_notify_data }
        },

        /* Characteristic Declaration */
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &character_declaration_uuid,
          ESP_GATT_PERM_READ,
          CHAR_DECLARATION_SIZE,
          CHAR_DECLARATION_SIZE,
          (uint8_t *) &char_prop_read_write }
        },

        /* Characteristic Value */
        { { ESP_GATT_AUTO_RSP },
          { ESP_UUID_LEN_16,
          (uint8_t *) &motion_tracking_rw,
          ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
          sizeof(uint8_t),
          sizeof(motion_tracking_rw_data),
          (uint8_t *) motion_tracking_rw_data }
        },
};

/*================================================================================================*/
/**
* @function name  esp_key_type_to_str
* @brief          esp_key_type_to_str
* @parameter      key_type
* @return value   none
* @note           none
*/
static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch (key_type)
    {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

    }

    return key_str;
}

/*================================================================================================*/
/**
* @function name  esp_auth_req_to_str
* @brief          esp_auth_req_to_str
* @parameter      auth_req
* @return value   none
* @note           none
*/
static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char *auth_str = NULL;
    switch (auth_req)
    {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
    }

    return auth_str;
}

/*================================================================================================*/
/**
* @function name  caculate_checksum
* @brief          calculating checksum
* @parameter      uint8_t *array_data
*                 uint8_t length
* @return value   none
* @note           none
*/
static uint8_t caculate_checksum(uint8_t *array_data, uint8_t length)
{
    uint8_t ret_value_u8;
    uint8_t counter_u8;
    /*---*/
    ret_value_u8 = 0;
    for (counter_u8 = 0; counter_u8 < length; counter_u8++)
    {
        ret_value_u8 ^= array_data[counter_u8];
    }
    return ret_value_u8;
}

/*================================================================================================*/
/**
* @function name  show_bonded_devices
* @brief          show bonding devices
* @parameter      none
* @return value   none
* @note           none
*/
static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);

    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);
    for (int i = 0; i < dev_num; i++)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}
/*================================================================================================*/
/**
* @function name  remove_all_bonded_devices
* @brief          remove bonding devices
* @parameter      none
* @return value   none
* @note           none
*/
static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

/*================================================================================================*/
/**
* @function name  gap_event_handler
* @brief          gap_event_handler
* @parameter      esp_gap_ble_cb_event_t event
*                 esp_ble_gap_cb_param_t *param
* @return value   none
* @note           none
*/
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&motion_tracking_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&motion_tracking_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: /* OOB request event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
         show the passkey number to the user to confirm it with the number displayed by peer deivce. */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
         If not accept the security request, should sent the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer deivce.
        ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
    {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x", (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
    {
        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }
        test_manufacturer[21] = a;
        test_manufacturer[23] = b;
        esp_err_t ret = esp_ble_gap_config_adv_data(&motion_tracking_adv_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&motion_tracking_scan_rsp_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;
    default:
        break;
    }
}

/*================================================================================================*/
/**
* @function name  gatts_profile_event_handler
* @brief          gatts_profile_event_handler
* @parameter      esp_gatts_cb_event_t event
*                 esp_gatt_if_t gatts_if
*                 esp_ble_gatts_cb_param_t *param
* @return value   none
* @note           none
*/
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    uint8_t counter_receive_u8;
    ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name(DEVICE_NAME);
        /* generate a resolvable random address */
        esp_ble_gap_config_local_privacy(true);
        esp_ble_gatts_create_attr_tab(motion_tracking_gatt_db, gatts_if, 6, MOTION_TRACKING_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        break;
        /*receive data from app*/
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        for (counter_receive_u8 = 0; counter_receive_u8 < RECEIVE_FROM_APP_DATA_LEN; counter_receive_u8++)
        {
            receive_from_app_gtr.array[counter_receive_u8] = param->write.value[counter_receive_u8];
            #ifdef MOTION_TRACKING_DEBUG
                ESP_LOGE(GATTS_TABLE_TAG, "data[%d] = %d\n", counter_receive_u8, receive_from_app_gtr.array[counter_receive_u8]);
            #endif
        }
        motion_tracking_flag_gstr.received_data_from_app = true;
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        /*EVEN: CONNECT WITH CENTRAL*/
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
        /* start security connect with peer device when receive the connect event sent by the master */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);

        if (tracking_variables_gtr.state_machine == BEACON_STATE)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "CENTRAL CONNECT OK");
            ESP_LOGI(GATTS_TABLE_TAG, "CHANGE TO: WORKING STATE");
            tracking_variables_gtr.state_machine = WORKING_STATE;
        }
        else
            ESP_LOGI(GATTS_TABLE_TAG, "EVEN ERROR");
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        /*EVEN: DIS_CONNECT WITH CENTRAL*/
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        /* start advertising again when missing the connect */
        esp_ble_gap_start_advertising(&motion_tracking_adv_params);
//        flag_gatts_u8 = 1;
        if (tracking_variables_gtr.state_machine == WORKING_STATE)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "CENTRAL DIS_CONNECT OK");
            ESP_LOGI(GATTS_TABLE_TAG, "CHANGE TO: BEACON STATE");
            tracking_variables_gtr.state_machine = BEACON_STATE;
        }
        else
            ESP_LOGI(GATTS_TABLE_TAG, "EVEN ERROR");
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
        if (param->create.status == ESP_GATT_OK)
        {
            if (param->add_attr_tab.num_handle == 6)
            {
                memcpy(motion_tracking_handle_table, param->add_attr_tab.handles, sizeof(motion_tracking_handle_table));
                esp_ble_gatts_start_service(motion_tracking_handle_table[0]);
            }
            else
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, 6);
            }
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
        }
        break;
    }

    default:
        break;
    }
}

/*================================================================================================*/
/**
* @function name  gatts_event_handler
* @brief          gatts_event_handler
* @parameter      esp_gatts_cb_event_t event
*                 esp_gatt_if_t gatts_if
*                 esp_ble_gatts_cb_param_t *param
* @return value   none
* @note           none
*/
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            motion_tracking_profile_tab[MOTION_TRACKING_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < MOTION_TRACKING_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            gatts_if == motion_tracking_profile_tab[idx].gatts_if)
            {
                if (motion_tracking_profile_tab[idx].gatts_cb)
                {
                    motion_tracking_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    }
    while (0);
}
/*================================================================================================*/
/**
* @function name  bt_init
* @brief          bluetooth_init
* @parameter      none
* @return value   none
* @note           none
*/
static void bt_init(void)
{
    esp_err_t ret;
    static uint8_t mem_release_u8 = 0;

    if (!mem_release_u8)
    {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        mem_release_u8 = 1;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
    ;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
}
/*================================================================================================*/
/**
* @function name  ble_gatt_init
* @brief          ble_gatt_init
* @parameter      none
* @return value   none
* @note           none
*/
void ble_gatt_init(void)
{
    esp_err_t ret;
    bt_init();
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(MOTION_TRACKING_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND; //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; //set the IO capability to No output No input
    uint8_t key_size = 16; //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    //set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
     and the response key means which key you can distribute to the Master;
     If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
     and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

/*================================================================================================*/
/**
* @function name  gatt_deinit
* @brief          deinit_gatt
* @parameter      none
* @return value   none
* @note           none
*/
void gattclient_deinit(void)
{
    esp_err_t ret = ESP_OK;

//    ret = esp_ble_gattc_app_unregister(gl_profile_tab[PROFILE_A_APP_ID].gattc_if);
//    if(ret){
//        ESP_LOGE_OP(GATTC_TAG, "esp_ble_gattc_app_unregister fail");
//        return;
//    }

    ret = esp_bluedroid_disable();
    if(ret){
//        ESP_LOGE_OP(GATTS_TABLE_TAG, "esp_bluedroid_disable fail");
        return;
    }

    ret = esp_bluedroid_deinit();
    if(ret){
//        ESP_LOGE_OP(GATTS_TABLE_TAG, "esp_bluedroid_deinit fail");
        return;
    }

    ret = esp_bt_controller_disable();
    if(ret){
//          ESP_LOGE_OP(GATTS_TABLE_TAG, "esp_bt_controller_disable fail");
          return;
    }

    ret = esp_bt_controller_deinit();
    if(ret){
//          ESP_LOGE_OP(GATTS_TABLE_TAG, "esp_bt_controller_deinit fail");
          return;
    }

//    ret = esp_bt_mem_release(ESP_BT_MODE_CLASSIC_BT);
//    if(ret){
//          ESP_LOGE_OP(GATTC_TAG, "esp_bt_mem_release fail");
//          return;
//    }
}

/*================================================================================================*/
/**
* @function name  ble_gatt_beacon_advertising_task
* @brief          Read ADC from Battery and Adaptor, Check Adaptor pluggin and Level of Battery, display LED(GPIO/PWM)
* @parameter      * arg
* @return value   none
* @note           none
*/
void ble_gatt_beacon_advertising_task(void* arg)
{
    while (1)
    {
        if ((tracking_variables_gtr.state_machine == WORKING_STATE)&&(motion_tracking_flag_gstr.MPU_notify2app))
        {
            notify2app_gstr.data.timestamp = tracking_variables_gtr.counter_milisecond;
            notify2app_gstr.data.addr = tracking_variables_gtr.device_address;

            notify2app_gstr.data.checksum = caculate_checksum(notify2app_gstr.array, 34);
            esp_ble_gatts_send_indicate(motion_tracking_profile_tab[MOTION_TRACKING_APP_IDX].gatts_if,
                    motion_tracking_profile_tab[MOTION_TRACKING_APP_IDX].conn_id, motion_tracking_handle_table[2], sizeof(notify2app_gstr.array),
                    notify2app_gstr.array, false);
            /*request notify via UART*/
            motion_tracking_flag_gstr.MPU_notify2uart = true;
            motion_tracking_flag_gstr.MPU_notify2app = false;
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        else
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}
/*================================================================================================*/
