/* main.c - Application main entry point */

/*
* SPDX-FileCopyrightText: 2017 Intel Corporation
* SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_rpr_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "inc/msg-defs.h"
#include "inc/model.h"
#include "inc/buffer.h"

#include "ble_mesh_example_init.h"

/********************************************************************
 * Definitions
 ********************************************************************/
#define TAG "EXAMPLE"
#define DEBUG_TEST_COM      1

#define LED_OFF             0x0
#define LED_ON              0x1

#define PROV_OWN_ADDR       0x0001
#define GROUP_ADDRESS       0xC000

#define MSG_SEND_TTL        3
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_PROVISIONER

#define COMP_DATA_PAGE_0    0x00

#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

#define PB_REMOTE           0x04

// UART configuration
#define UART_TXD 1
#define UART_RXD 3
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      2
#define UART_BAUD_RATE     9600
#define TASK_STACK_SIZE    2048
#define BUF_SIZE           80

/********************************************************************
 * Local variables
 ********************************************************************/
static ipac_uart_cmd_queue_t cmd_queue;

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[16];
    uint8_t  app_key_cache[16];
} prov_key;

static uint8_t dev_uuid[16];
static uint16_t cur_rpr_cli_opcode;
static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};
#if CONFIG_BLE_MESH_RPR_CLI
static esp_ble_mesh_client_t remote_prov_client;
#endif

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),     // this is for connecting to smart devices
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
#if CONFIG_BLE_MESH_RPR_CLI
    ESP_BLE_MESH_MODEL_RPR_CLI(&remote_prov_client),
#endif
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid           = dev_uuid,
    .prov_unicast_addr   = PROV_OWN_ADDR,
    .prov_start_address  = 0x0005,
    .prov_attention      = 0x00,
    .prov_algorithm      = 0x00,
    .prov_pub_key_oob    = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags               = 0x00,
    .iv_index            = 0x00,
};

/* Sensor Client Model definitions */
static const esp_ble_mesh_client_op_pair_t sensor_model_op_pair[] = {
    { SENSOR_MODEL_OPCODE_GET, SENSOR_MODEL_OPCODE_STATUS },
    { SENSOR_MODEL_OPCODE_SET, SENSOR_MODEL_OPCODE_STATUS },
};

static esp_ble_mesh_client_t sensor_client = {
    .op_pair_size = ARRAY_SIZE(sensor_model_op_pair),
    .op_pair = sensor_model_op_pair,
};

static esp_ble_mesh_model_op_t sensor_model_op[] = {
    // message minimum length is 2 octets
    // ESP_BLE_MESH_MODEL_OP(SENSOR_MODEL_OPCODE_STATUS,
    //                     sizeof(ipac_ble_mesh_model_msg_sensor_data_status_t)),
    ESP_BLE_MESH_MODEL_OP(SENSOR_MODEL_OPCODE_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

// ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, 1, MSG_ROLE);

/* Device Info Client Model definitions */
static const esp_ble_mesh_client_op_pair_t device_info_model_op_pair[] = {
    { DEVICE_INFO_MODEL_OPCODE_GET, DEVICE_INFO_MODEL_OPCODE_STATUS },
};

static esp_ble_mesh_client_t device_info_client = {
    .op_pair_size = ARRAY_SIZE(device_info_model_op_pair),
    .op_pair = device_info_model_op_pair,
};

static esp_ble_mesh_model_op_t device_info_model_op[] = {
    // message minimum length is 2 octets
    ESP_BLE_MESH_MODEL_OP(DEVICE_INFO_MODEL_OPCODE_STATUS,
                        sizeof(ipac_ble_mesh_model_msg_device_info_status_t)),
    ESP_BLE_MESH_MODEL_OP_END,
};

// ESP_BLE_MESH_MODEL_PUB_DEFINE(device_info_cli_pub, 1, MSG_ROLE);

/* AC Control Client Model definitions */
static const esp_ble_mesh_client_op_pair_t ac_control_model_op_pair[] = {
    { AC_CONTROL_STATE_OPCODE_GET, AC_CONTROL_STATE_OPCODE_STATUS },
    { AC_CONTROL_STATE_OPCODE_SET, AC_CONTROL_STATE_OPCODE_STATUS },
};

static esp_ble_mesh_client_t ac_control_client = {
    .op_pair_size = ARRAY_SIZE(ac_control_model_op_pair),
    .op_pair = ac_control_model_op_pair,
};

static esp_ble_mesh_model_op_t ac_control_model_op[] = {
    // message minimum length is 2 octets
    ESP_BLE_MESH_MODEL_OP(AC_CONTROL_STATE_OPCODE_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

// ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, 1, MSG_ROLE);

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, SENSOR_MODEL_ID_CLIENT,
    sensor_model_op, NULL, &sensor_client),
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, AC_CONTROL_MODEL_CLIENT,
        ac_control_model_op, NULL, &ac_control_client),
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, DEVICE_INFO_MODEL_ID_CLIENT,
        device_info_model_op, NULL, &device_info_client),
};

// static esp_ble_mesh_model_t* device_info_cli_model = &vnd_models[0];
// static esp_ble_mesh_model_t* sensor_cli_model = &vnd_models[1];

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

// uart command
static void ipac_uart_cmd_recv_get_local_keys(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_update_local_keys(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_scan_device(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_stop_scan(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_add_device(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_delete_node(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_get_composition_data(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_add_app_key(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_model_app_bind(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_model_pub_set(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_model_sub_add(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_relay_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_relay_set(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_proxy_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_proxy_set(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_friend_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_friend_set(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_heartbeat_pub_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_heartbeat_pub_set(void *arg, uint8_t status);
#if CONFIG_BLE_MESH_RPR_CLI
static void ipac_uart_cmd_recv_rpr_scan_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_rpr_scan_start(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_rpr_scan_stop(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_rpr_link_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_rpr_link_open(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_rpr_link_close(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_remote_prov(void *arg, uint8_t status);
#endif
static void ipac_uart_cmd_recv_sensor_data_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_device_info_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_ac_control_state_set(void *arg, uint8_t status);

#if defined(DEBUG_TEST_COM) && (DEBUG_TEST_COM == 1)
static void test_simple_msg(void *arg, uint8_t status);
#endif
static const ipac_uart_command_t uart_cmd[] = {
    {OPCODE_GET_LOCAL_KEYS, MSG_ARG_SIZE_GET_LOCAL_KEYS, ipac_uart_cmd_recv_get_local_keys},
    {OPCODE_UPDATE_LOCAL_KEYS, MSG_ARG_SIZE_UPDATE_LOCAL_KEYS, ipac_uart_cmd_recv_update_local_keys},
    {OPCODE_SCAN_UNPROV_DEV, MSG_ARG_SIZE_SCAN_UNPROV_DEV, ipac_uart_cmd_recv_scan_device},
    {OPCODE_PROVISIONER_DISABLE, MSG_ARG_SIZE_STOP_SCAN, ipac_uart_cmd_recv_stop_scan},
    {OPCODE_ADD_UNPROV_DEV, MSG_ARG_SIZE_ADD_UNPROV_DEV, ipac_uart_cmd_recv_add_device},
    {OPCODE_DELETE_DEVICE, MSG_ARG_SIZE_DELETE_DEVICE, ipac_uart_cmd_recv_delete_node},
    {OPCODE_GET_COMPOSITION_DATA, MSG_ARG_SIZE_GET_COMPOSITION_DATA, ipac_uart_cmd_recv_get_composition_data},
    {OPCODE_ADD_APP_KEY, MSG_ARG_SIZE_ADD_APP_KEY, ipac_uart_cmd_recv_add_app_key},
    {OPCODE_BIND_MODEL_APP, MSG_ARG_SIZE_BIND_MODEL_APP, ipac_uart_cmd_recv_model_app_bind},
    {OPCODE_SET_MODEL_PUB, MSG_ARG_SIZE_SET_MODEL_PUB, ipac_uart_cmd_recv_model_pub_set},
    {OPCODE_SET_MODEL_SUB, MSG_ARG_SIZE_SET_MODEL_SUB, ipac_uart_cmd_recv_model_sub_add},
    {OPCODE_SENSOR_DATA_GET, MSG_ARG_SIZE_SENSOR_DATA_GET, ipac_uart_cmd_recv_sensor_data_get},
    {OPCODE_AC_CONTROL_STATE_GET, MSG_ARG_SIZE_AC_CONTROL_STATE_SET, ipac_uart_cmd_recv_ac_control_state_set},
    {OPCODE_DEVICE_INFO_GET, MSG_ARG_SIZE_DEVICE_INFO_GET, ipac_uart_cmd_recv_device_info_get},
    {OPCODE_RELAY_GET, MSG_ARG_SIZE_RELAY_GET, ipac_uart_cmd_recv_relay_get},
    {OPCODE_RELAY_SET, MSG_ARG_SIZE_RELAY_SET, ipac_uart_cmd_recv_relay_set},
    {OPCODE_PROXY_GET, MSG_ARG_SIZE_PROXY_GET, ipac_uart_cmd_recv_proxy_get},
    {OPCODE_PROXY_SET, MSG_ARG_SIZE_PROXY_SET, ipac_uart_cmd_recv_proxy_set},
    {OPCODE_FRIEND_GET, MSG_ARG_SIZE_FRIEND_GET, ipac_uart_cmd_recv_friend_get},
    {OPCODE_FRIEND_SET, MSG_ARG_SIZE_FRIEND_SET, ipac_uart_cmd_recv_friend_set},
    {OPCODE_HEARTBEAT_PUB_GET, MSG_ARG_SIZE_HEARTBEAT_PUB_GET, ipac_uart_cmd_recv_heartbeat_pub_get},
    {OPCODE_HEARTBEAT_PUB_SET, MSG_ARG_SIZE_HEARTBEAT_PUB_SET, ipac_uart_cmd_recv_heartbeat_pub_set},
#if CONFIG_BLE_MESH_RPR_CLI
    {OPCODE_RPR_SCAN_START, MSG_ARG_SIZE_RPR_SCAN_START, ipac_uart_cmd_recv_rpr_scan_start},
    {OPCODE_RPR_SCAN_STOP, MSG_ARG_SIZE_RPR_SCAN_STOP, ipac_uart_cmd_recv_rpr_scan_stop},
    {OPCODE_RPR_LINK_GET, MSG_ARG_SIZE_RPR_LINK_GET, ipac_uart_cmd_recv_rpr_link_get},
    {OPCODE_RPR_LINK_OPEN, MSG_ARG_SIZE_RPR_LINK_OPEN, ipac_uart_cmd_recv_rpr_link_open},
    {OPCODE_RPR_LINK_CLOSE, MSG_ARG_SIZE_RPR_LINK_CLOSE, ipac_uart_cmd_recv_rpr_link_close},
    {OPCODE_REMOTE_PROVISIONING, MSG_ARG_SIZE_REMOTE_PROV, ipac_uart_cmd_recv_remote_prov},
#endif
#if defined(DEBUG_TEST_COM) && (DEBUG_TEST_COM == 1)
    {OPCODE_TEST_SIMPLE_MSG, MSG_ARG_SIZE_TEST_SIMPLE_MSG, test_simple_msg}
#endif
};

/********************************************************************
 * Ultilities functions
 ********************************************************************/
/**
 * @brief Calculate checksum for UART communication
 * 
 * @param msg: message data
 * @param opcode: opcode of message. This opcode value is plus to checksum.
 * To validate checksum of received message:
 * - Set opcode to message opcode
 * - Set argument of command as `msg`
 * - Set packet_len as size of `msg`
 * The checksum is valid if the result is 0x00
 * 
 * To calculate checksum of send message:
 * - Set opcode to 0
 * - Set argument of command as `msg`
 * - Set packet_len as size of `msg` (exclude checksum byte)
 * @param packet_len: length of message
 */
uint8_t ipac_cal_checksum(const void *msg, uint8_t opcode, size_t packet_len) {
    const uint8_t *data = (const uint8_t *)msg;
    uint8_t checksum = opcode;

    for (size_t i = 0; i < packet_len; i++) {
        checksum += data[i];
    }
    return (~checksum); // 8-bit checksum
}

static esp_err_t ipac_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                                uint16_t unicast,
                                                esp_ble_mesh_model_t *model,
                                                uint32_t opcode)
{
    if (!common || !model || !unicast) {
        return ESP_ERR_INVALID_ARG;
    }

    common->opcode = opcode;    // config client opcode
    common->model = model;      // this is equal to config client model
    common->ctx.net_idx = prov_key.net_idx;     // netkey can be changed if user want to change subnet
    common->ctx.app_idx = prov_key.app_idx;     // similar to netkey
    common->ctx.addr = unicast;                 // need unicast only when received cmd from RPi
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->msg_timeout = MSG_TIMEOUT;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common->msg_role = MSG_ROLE;
#endif

    return ESP_OK;
}

/********************************************************************
 * UART communication message
 ********************************************************************/
static void ipac_uart_cmd_recv_get_local_keys(void *arg, uint8_t status) {
    uint8_t* net_key = NULL;
    uint8_t* app_key = NULL;
    ipac_ble_mesh_msg_send_get_local_keys_t res = {
        .opcode = OPCODE_GET_LOCAL_KEYS,
        .checksum = 0x00,
    };

    net_key = esp_ble_mesh_provisioner_get_local_net_key(ESP_BLE_MESH_KEY_PRIMARY);
    if (net_key != NULL) {
        app_key = esp_ble_mesh_provisioner_get_local_app_key(ESP_BLE_MESH_KEY_PRIMARY, APP_KEY_IDX);
        if (app_key != NULL) {
            res.status = RESPONSE_BYTE_STATUS_OK;
            memcpy(res.net_key, net_key, ESP_BLE_MESH_OCTET16_LEN);
            memcpy(res.app_key, app_key, ESP_BLE_MESH_OCTET16_LEN);
        }
        else {
            res.status = RESPONSE_BYTE_STATUS_FAILED;
        }
    }
    else {
        res.status = RESPONSE_BYTE_STATUS_FAILED;
    }

    res.checksum = ipac_cal_checksum((void*) &res, 0, MSG_SIZE_GET_LOCAL_KEYS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &res, MSG_SIZE_GET_LOCAL_KEYS);
}

static void ipac_uart_cmd_recv_update_local_keys(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    uint8_t res[3] = {OPCODE_UPDATE_LOCAL_KEYS};

    if (status != PACKET_OK) {
        // error msg
        return;
    }
    
    if (ipac_cal_checksum(arg, OPCODE_UPDATE_LOCAL_KEYS, MSG_ARG_SIZE_UPDATE_LOCAL_KEYS) != 0x00)
    {
        // error msg
        return;
    }

    memcpy(prov_key.app_key_cache, ((ipac_ble_mesh_msg_recv_update_local_keys_t*)arg)->app_key, 16);
    err = esp_ble_mesh_provisioner_update_local_net_key(((ipac_ble_mesh_msg_recv_update_local_keys_t*)arg)->net_key, ESP_BLE_MESH_KEY_PRIMARY);
    if (err != ESP_OK) {
        res[1] = RESPONSE_BYTE_STATUS_FAILED;
        res[2] = ipac_cal_checksum((void*) res, 0, sizeof(res));
        uart_write_bytes(UART_PORT_NUM, (const void *) res, sizeof(res));
        return;
    }
}

// the 0x02 cmd is used for update local keys only, so status feedback is enough. 
static void ipac_uart_cmd_send_update_local_keys_status(uint8_t status) {
    uint8_t res[3] = {OPCODE_UPDATE_LOCAL_KEYS};

    res[1] = (status == ESP_OK) ? RESPONSE_BYTE_STATUS_OK : RESPONSE_BYTE_STATUS_FAILED;
    res[2] = ipac_cal_checksum((void*) res, 0, sizeof(res));
    uart_write_bytes(UART_PORT_NUM, (const void *) res, sizeof(res));
}

static void ipac_uart_cmd_recv_scan_device(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    uint8_t res[3] = {OPCODE_SCAN_UNPROV_DEV};

    err = esp_ble_mesh_provisioner_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    res[1] = (err != ESP_OK) ? RESPONSE_BYTE_STATUS_FAILED : RESPONSE_BYTE_STATUS_OK;
    res[2] = ipac_cal_checksum((void*) res, 0, sizeof(res));
    uart_write_bytes(UART_PORT_NUM, (const void *) res, sizeof(res));
}

static void ipac_uart_cmd_recv_stop_scan(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    uint8_t res[3] = {OPCODE_PROVISIONER_DISABLE};

    err = esp_ble_mesh_provisioner_prov_disable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    res[1] = (err != ESP_OK) ? RESPONSE_BYTE_STATUS_FAILED : RESPONSE_BYTE_STATUS_OK;
    res[2] = ipac_cal_checksum((void*) res, 0, sizeof(res));
    uart_write_bytes(UART_PORT_NUM, (const void *) res, sizeof(res));
}

static void ipac_uart_cmd_send_scan_result(esp_ble_mesh_prov_cb_param_t *param)
{
    ipac_ble_mesh_msg_scan_result_t msg = {
        .opcode = OPCODE_SCAN_RESULT,
        .addr_type = param->provisioner_recv_unprov_adv_pkt.addr_type,
        .oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info,
        .adv_type = param->provisioner_recv_unprov_adv_pkt.adv_type,
        .bearer_type = (uint8_t)(param->provisioner_recv_unprov_adv_pkt.bearer),
        .rssi = param->provisioner_recv_unprov_adv_pkt.rssi,
    };

    memcpy(msg.device_name, "IPAC_LAB_SMART_FARM", DEVICE_NAME_MAX_SIZE);
    memcpy(msg.uuid, param->provisioner_recv_unprov_adv_pkt.dev_uuid, 16);
    memcpy(msg.addr, param->provisioner_recv_unprov_adv_pkt.addr, BD_ADDR_LEN);
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_SCAN_RESULT);

    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_SCAN_RESULT);
}

static void ipac_uart_cmd_recv_add_device(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    ipac_ble_mesh_msg_sent_add_unprov_dev_ack_t res = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_ADD_UNPROV_DEV, MSG_ARG_SIZE_ADD_UNPROV_DEV) != (uint8_t)0x00)
    {
        // error msg
        return;
    }

    // get data trong msg packet
    add_dev.oob_info = ((ipac_ble_mesh_msg_recv_add_unprov_dev_t*)arg)->oob_info;
    add_dev.addr_type = (esp_ble_mesh_addr_type_t)((ipac_ble_mesh_msg_recv_add_unprov_dev_t*)arg)->addr_type;
    add_dev.bearer = (esp_ble_mesh_prov_bearer_t)(((ipac_ble_mesh_msg_recv_add_unprov_dev_t*)arg)->bearer_type);
    memcpy(add_dev.uuid, ((ipac_ble_mesh_msg_recv_add_unprov_dev_t*)arg)->uuid, 16);
    memcpy(add_dev.addr, ((ipac_ble_mesh_msg_recv_add_unprov_dev_t*)arg)->addr, BD_ADDR_LEN);

    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
        (esp_ble_mesh_dev_add_flag_t)(ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG));
    
    res.opcode = OPCODE_ADD_UNPROV_DEV;
    res.addr_type = add_dev.addr_type;
    res.oob_info = add_dev.oob_info;
    res.bearer_type = add_dev.bearer;
    memcpy(res.uuid, add_dev.uuid, 16);
    memcpy(res.addr, add_dev.addr, 6);
    res.status = (err == ESP_OK) ? RESPONSE_BYTE_STATUS_OK : RESPONSE_BYTE_STATUS_FAILED;
    res.checksum = ipac_cal_checksum((void*) &res, 0, MSG_SIZE_ADD_UNPROV_DEV_ACK);
    uart_write_bytes(UART_PORT_NUM, (const void *) &res, MSG_SIZE_ADD_UNPROV_DEV_ACK);
}

static esp_err_t send_new_node_info_msg(ipac_ble_mesh_msg_send_new_node_info_t* node)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(node->unicast)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* RPi side need to judge if the device has been provisioned before
    If no, modify provisioned node data in database
    Otherwise, add node to database */
    node->opcode = OPCODE_SEND_NEW_NODE_INFO;
    node->checksum = ipac_cal_checksum((void*) node, 0, MSG_SIZE_SEND_NEW_NODE_INFO);
    uart_write_bytes(UART_PORT_NUM, (const void *) node, MSG_SIZE_SEND_NEW_NODE_INFO);
    return ESP_OK;
}

static void ipac_uart_cmd_send_add_device_complete(void *param, uint8_t remote) {
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
    esp_err_t err = ESP_OK;
    ipac_ble_mesh_msg_send_new_node_info_t node = {0};
    
    node.remote = remote; 
    if (remote == false) {
        node.rpr_srv_addr = 0x0000;
        node.unicast = ((esp_ble_mesh_prov_cb_param_t*)param)->provisioner_prov_complete.unicast_addr,
        node.net_idx = ((esp_ble_mesh_prov_cb_param_t*)param)->provisioner_prov_complete.netkey_idx,
        node.elem_num = ((esp_ble_mesh_prov_cb_param_t*)param)->provisioner_prov_complete.element_num,
        memcpy(node.uuid, ((esp_ble_mesh_prov_cb_param_t*)param)->provisioner_prov_complete.device_uuid, 16);
    }
    else {
        node.rpr_srv_addr = ((esp_ble_mesh_rpr_client_cb_param_t*)param)->prov.rpr_srv_addr;
        node.unicast = ((esp_ble_mesh_rpr_client_cb_param_t*)param)->prov.unicast_addr,
        node.net_idx = ((esp_ble_mesh_rpr_client_cb_param_t*)param)->prov.net_idx,
        node.elem_num = ((esp_ble_mesh_rpr_client_cb_param_t*)param)->prov.element_num,    
        memcpy(node.uuid, ((esp_ble_mesh_rpr_client_cb_param_t*)param)->prov.uuid, 16);
    }

    err = send_new_node_info_msg(&node);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "%s: Store node info failed", __func__);
    }

    // get node info:
    // - primary address is unicast (ESP_BLE_MESH_ADDR_IS_UNICAST)
    // - search database with primary address

    // ipac_ble_mesh_set_msg_common(&common, node.unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    // get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    // err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    // if (err) {
    //     // ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
    // }
}

static void ipac_uart_cmd_recv_delete_node(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    ipac_ble_mesh_msg_send_delete_device_status_t res = {
        .opcode = OPCODE_DELETE_DEVICE,
    };

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_DELETE_DEVICE, MSG_ARG_SIZE_DELETE_DEVICE) != 0x00)
    {
        // error
        return;
    }

    err = esp_ble_mesh_provisioner_delete_node_with_addr(((ipac_ble_mesh_msg_recv_delete_device_t*)arg)->unicast);
    if (err != ESP_OK) {
        res.unicast = ((ipac_ble_mesh_msg_recv_delete_device_t*)arg)->unicast;
        res.status = RESPONSE_BYTE_STATUS_FAILED;
        res.checksum = ipac_cal_checksum((void*) &res, 0, MSG_SIZE_DELETE_DEVICE_STATUS);
        uart_write_bytes(UART_PORT_NUM, (const void *) &res, MSG_SIZE_DELETE_DEVICE_STATUS);
    }
}

static void ipac_uart_cmd_send_delete_device_status(esp_ble_mesh_prov_cb_param_t *param) {
    ipac_ble_mesh_msg_send_delete_device_status_t msg = {
        .opcode = OPCODE_DELETE_DEVICE,
        .unicast = param->provisioner_delete_node_with_addr_comp.unicast_addr,
        .status = (param->provisioner_delete_node_with_addr_comp.err_code == ESP_OK)
                    ? RESPONSE_BYTE_STATUS_OK : RESPONSE_BYTE_STATUS_FAILED,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_DELETE_DEVICE_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_DELETE_DEVICE_STATUS);
}

static void ipac_uart_cmd_recv_get_composition_data(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_GET_COMPOSITION_DATA, MSG_ARG_SIZE_GET_COMPOSITION_DATA) != 0x00)
    {
        // error
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_get_composition_data_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
    }
}

static void ipac_uart_cmd_composition_data_status(esp_ble_mesh_cfg_client_cb_param_t *param) {
    uint8_t* ptr = param->status_cb.comp_data_status.composition_data->data;
    uint16_t comp_data_len = param->status_cb.comp_data_status.composition_data->len;

    // total required bytes = 6 bytes + comp_data_len
    uint8_t* msg = (uint8_t*) malloc(6 + comp_data_len);

    msg[0] = OPCODE_GET_COMPOSITION_DATA;
    memcpy(&msg[1], (void*) &(param->params->ctx.addr), 2);
    memcpy(&msg[3], (void*) &(comp_data_len), 2);
    memcpy(&msg[5], (void*) ptr, comp_data_len);
    msg[comp_data_len + 5] = ipac_cal_checksum((void*) msg, 0, comp_data_len + 5);
    
    uart_write_bytes(UART_PORT_NUM, (const void *) msg, 6 + comp_data_len);
}

static void ipac_uart_cmd_recv_add_app_key(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    esp_ble_mesh_client_common_param_t common = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_ADD_APP_KEY, MSG_ARG_SIZE_ADD_APP_KEY) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_add_app_key_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    set_state.app_key_add.net_idx = prov_key.net_idx;
    set_state.app_key_add.app_idx = prov_key.app_idx;
    memcpy(set_state.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
        return;
    }
}

static void ipac_uart_cmd_send_app_key_status(esp_ble_mesh_cfg_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_add_app_key_status_t msg = {
        .opcode = OPCODE_ADD_APP_KEY,
        .unicast = param->params->ctx.addr,
        .status = param->status_cb.appkey_status.status,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_ADD_APP_KEY_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_ADD_APP_KEY_STATUS);
}

static void ipac_uart_cmd_recv_model_app_bind(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_BIND_MODEL_APP, MSG_ARG_SIZE_BIND_MODEL_APP) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_bind_model_app_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
    if (err) {
        // error msg
        return;
    }
    set_state.model_app_bind.element_addr = ((ipac_ble_mesh_msg_recv_bind_model_app_t*)arg)->unicast;
    set_state.model_app_bind.model_app_idx = prov_key.app_idx;
    set_state.model_app_bind.model_id = ((ipac_ble_mesh_msg_recv_bind_model_app_t*)arg)->model_id;
    set_state.model_app_bind.company_id = ((ipac_ble_mesh_msg_recv_bind_model_app_t*)arg)->cid;
    // set_state.model_app_bind.model_id = 0x1000;
    // set_state.model_app_bind.company_id = 0xFFFF;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_model_app_status(esp_ble_mesh_cfg_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_model_app_status_t msg = {
        .opcode = OPCODE_BIND_MODEL_APP,
        .unicast = param->params->ctx.addr,
        .model_id = param->status_cb.model_app_status.model_id,
        .cid = param->status_cb.model_app_status.company_id,
        .status = param->status_cb.model_app_status.status,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_MODEL_APP_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_MODEL_APP_STATUS);
}

static void ipac_uart_cmd_recv_model_pub_set(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_SET_MODEL_PUB, MSG_ARG_SIZE_SET_MODEL_PUB) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    set_state.model_pub_set.element_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast;
    set_state.model_pub_set.publish_app_idx = prov_key.app_idx;
    set_state.model_pub_set.publish_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->group_addr;
    set_state.model_pub_set.model_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->model_id;
    set_state.model_pub_set.company_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->cid;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_model_pub_status(esp_ble_mesh_cfg_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_model_pub_sub_status_t msg = {
        .opcode = OPCODE_SET_MODEL_PUB,
        .unicast = param->params->ctx.addr,
        .group_addr = param->status_cb.model_pub_status.publish_addr,
        .model_id = param->status_cb.model_pub_status.model_id,
        .cid = param->status_cb.model_pub_status.company_id,
        .status = param->status_cb.model_pub_status.status,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_MODEL_PUB_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_MODEL_PUB_STATUS);
}

static void ipac_uart_cmd_recv_model_sub_add(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_SET_MODEL_SUB, MSG_ARG_SIZE_SET_MODEL_SUB) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    set_state.model_sub_add.element_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast;
    set_state.model_sub_add.sub_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->group_addr;
    set_state.model_sub_add.model_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->model_id;
    set_state.model_sub_add.company_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->cid;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_model_sub_status(esp_ble_mesh_cfg_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_model_pub_sub_status_t msg = {
        .opcode = OPCODE_SET_MODEL_SUB,
        .unicast = param->params->ctx.addr,
        .group_addr = param->status_cb.model_sub_status.sub_addr,
        .model_id = param->status_cb.model_sub_status.model_id,
        .cid = param->status_cb.model_sub_status.company_id,
        .status = param->status_cb.model_sub_status.status,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_MODEL_SUB_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_MODEL_SUB_STATUS);
}

static void ipac_uart_cmd_recv_relay_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RELAY_GET, MSG_ARG_SIZE_RELAY_GET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_node_role_get_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_RELAY_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_recv_relay_set(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RELAY_SET, MSG_ARG_SIZE_RELAY_SET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_relay_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_RELAY_SET);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    set_state.relay_set.relay = ((ipac_ble_mesh_msg_recv_relay_set_t*)arg)->relay_state;
    set_state.relay_set.relay_retransmit = ((ipac_ble_mesh_msg_recv_relay_set_t*)arg)->relay_retransmit;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_relay_status(esp_ble_mesh_cfg_client_cb_param_t *param, uint8_t uart_opcode) {
    ipac_ble_mesh_msg_send_relay_status_t msg = {
        .opcode = uart_opcode,
        .unicast = param->params->ctx.addr,
        .relay_state = param->status_cb.relay_status.relay,
        .relay_retransmit = param->status_cb.relay_status.retransmit,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_RELAY_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_RELAY_STATUS);
}

static void ipac_uart_cmd_recv_proxy_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_PROXY_GET, MSG_ARG_SIZE_PROXY_GET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_node_role_get_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_GATT_PROXY_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_recv_proxy_set(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_PROXY_SET, MSG_ARG_SIZE_PROXY_SET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_proxy_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_GATT_PROXY_SET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    set_state.gatt_proxy_set.gatt_proxy = ((ipac_ble_mesh_msg_recv_proxy_set_t*)arg)->proxy_state;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_proxy_status(esp_ble_mesh_cfg_client_cb_param_t *param, uint8_t uart_opcode) {
    ipac_ble_mesh_msg_send_proxy_status_t msg = {
        .opcode = uart_opcode,
        .unicast = param->params->ctx.addr,
        .proxy_state = param->status_cb.gatt_proxy_status.gatt_proxy,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_PROXY_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_PROXY_STATUS);
}

static void ipac_uart_cmd_recv_friend_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_PROXY_GET, MSG_ARG_SIZE_PROXY_GET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_node_role_get_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_FRIEND_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_recv_friend_set(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_PROXY_GET, MSG_ARG_SIZE_PROXY_GET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_friend_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_FRIEND_SET);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    set_state.friend_set.friend_state = ((ipac_ble_mesh_msg_recv_friend_set_t*)arg)->friend_state;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_friend_status(esp_ble_mesh_cfg_client_cb_param_t *param, uint8_t uart_opcode) {
    ipac_ble_mesh_msg_send_friend_status_t msg = {
        .opcode = uart_opcode,
        .unicast = param->params->ctx.addr,
        .friend_state = param->status_cb.friend_status.friend_state,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_FRIEND_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_FRIEND_STATUS);
}

static void ipac_uart_cmd_recv_heartbeat_pub_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_HEARTBEAT_PUB_SET, MSG_ARG_SIZE_HEARTBEAT_PUB_SET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_heartbeat_pub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_GET);
    if (err != ESP_OK) {
        return;
    }

    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_recv_heartbeat_pub_set(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_HEARTBEAT_PUB_SET, MSG_ARG_SIZE_HEARTBEAT_PUB_SET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_heartbeat_pub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET);
    if (err != ESP_OK) {
        return;
    }

    set_state.heartbeat_pub_set.count = 0xFF;
    set_state.heartbeat_pub_set.feature = 0x0F;
    set_state.heartbeat_pub_set.net_idx = prov_key.net_idx;
    set_state.heartbeat_pub_set.dst = ((ipac_ble_mesh_msg_recv_heartbeat_pub_set_t*)arg)->dst;
    set_state.heartbeat_pub_set.period = ((ipac_ble_mesh_msg_recv_heartbeat_pub_set_t*)arg)->period;
    set_state.heartbeat_pub_set.ttl = ((ipac_ble_mesh_msg_recv_heartbeat_pub_set_t*)arg)->ttl;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err != ESP_OK) {
        // error msg
        return;
    }
}

static void ipac_uart_cmd_send_heartbeat_pub_status(esp_ble_mesh_cfg_client_cb_param_t *param, uint8_t uart_opcode) {
    ipac_ble_mesh_msg_send_heartbeat_pub_status_t msg = {
        .opcode = uart_opcode,
        .unicast = param->params->ctx.addr,
        .dst = param->status_cb.heartbeat_pub_status.dst,
        .ttl = param->status_cb.heartbeat_pub_status.ttl,
        .period = param->status_cb.heartbeat_pub_status.period,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_HEARTBEAT_PUB_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_HEARTBEAT_PUB_STATUS);
}

static void pac_uart_cmd_send_heartbeat_msg(esp_ble_mesh_prov_cb_param_t *param) {
    ipac_ble_mesh_msg_send_heartbeat_msg_t msg = {
        .opcode = OPCODE_HEARTBEAT_MSG,
        .feature = param->heartbeat_msg_recv.feature,
        .hops = param->heartbeat_msg_recv.hops,
    };

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_HEARTBEAT_MSG);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_HEARTBEAT_MSG);
}

#if CONFIG_BLE_MESH_RPR_CLI
static void ipac_uart_cmd_recv_rpr_scan_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_SCAN_GET, MSG_ARG_SIZE_RPR_SCAN_GET) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_scan_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_SCAN_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    err = esp_ble_mesh_rpr_client_send(&common, NULL);      // can append message at here instead of NULL
    if (err != ESP_OK) {
        // error msg
    }
}

static void ipac_uart_cmd_recv_rpr_scan_start(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_rpr_client_msg_t msg = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_SCAN_START, MSG_ARG_SIZE_RPR_SCAN_START) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_scan_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_SCAN_START);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    msg.scan_start.scan_items_limit = 0; /* 0 indicates there is no limit for scan items' count */
    msg.scan_start.timeout = 0x0A;       /* 0x0A is the default timeout */
    msg.scan_start.uuid_en = 0;          /* If uuid enabled, a specify device which have the same uuid will be report */
                                            /* If uuid disable, any unprovision device all will be report */

    err = esp_ble_mesh_rpr_client_send(&common, &msg);
    if (err != ESP_OK) {
        // error msg
    }
}

static void ipac_uart_cmd_recv_rpr_scan_stop(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_SCAN_STOP, MSG_ARG_SIZE_RPR_SCAN_STOP) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_scan_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_SCAN_STOP);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    err = esp_ble_mesh_rpr_client_send(&common, NULL);      // can append message at here instead of NULL
    if (err != ESP_OK) {
        // error msg
    }
}

static void ipac_uart_cmd_send_rpr_scan_status(uint8_t uart_opcode, esp_ble_mesh_rpr_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_rpr_scan_status_t msg = {0};
    
    msg.opcode = uart_opcode;
    msg.unicast = param->recv.params->ctx.addr;
    msg.status = param->recv.val.scan_status.status;
    msg.rpr_scanning = param->recv.val.scan_status.rpr_scanning;
    msg.scan_items_limit = param->recv.val.scan_status.scan_items_limit;
    msg.timeout = param->recv.val.scan_status.timeout;

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_RPR_SCAN_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_RPR_SCAN_STATUS);
}

static void ipac_uart_cmd_send_rpr_scan_report(esp_ble_mesh_rpr_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_rpr_scan_report_t msg = {0};

    msg.opcode = OPCODE_RPR_SCAN_RESULT;
    msg.unicast = param->recv.params->ctx.addr;
    msg.rssi = param->recv.val.scan_report.rssi;
    memcpy(msg.uuid, param->recv.val.scan_report.uuid, 16);
    msg.oob_info = param->recv.val.scan_report.oob_info;
    msg.uri_hash = param->recv.val.scan_report.uri_hash;

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_RPR_SCAN_REPORT);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_RPR_SCAN_REPORT);
}

static void ipac_uart_cmd_recv_rpr_link_get(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_LINK_GET, MSG_ARG_SIZE_RPR_LINK_GET) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_link_get_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_LINK_GET);
    if (err != ESP_OK) {
        // error msg
        return;
    }

    err = esp_ble_mesh_rpr_client_send(&common, NULL);
    if (err != ESP_OK) {
        // error msg
    }
}   

static void ipac_uart_cmd_recv_rpr_link_open(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_rpr_client_msg_t msg = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_LINK_OPEN, MSG_ARG_SIZE_RPR_LINK_OPEN) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_link_open_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_LINK_OPEN);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    msg.link_open.uuid_en = 1;
    memcpy(msg.link_open.uuid, ((ipac_ble_mesh_msg_recv_rpr_link_open_t*)arg)->uuid, 16);
    msg.link_open.timeout_en = 0;

    err = esp_ble_mesh_rpr_client_send(&common, NULL);
    if (err != ESP_OK) {
        // error msg
    }
}

static void ipac_uart_cmd_recv_rpr_link_close(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_rpr_client_msg_t msg = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_RPR_LINK_OPEN, MSG_ARG_SIZE_RPR_LINK_OPEN) != 0x00)
    {
        // error msg
        return;
    }
    
    remote_rpr_srv_addr = ((ipac_ble_mesh_msg_recv_rpr_link_close_t*)arg)->unicast;
    err = ipac_ble_mesh_set_msg_common(&common, remote_rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_LINK_CLOSE);
    if (err != ESP_OK) {
        // error msg
        return;
    }
    
    msg.link_close.reason = ((ipac_ble_mesh_msg_recv_rpr_link_close_t*)arg)->reason;
    
    err = esp_ble_mesh_rpr_client_send(&common, &msg);
    if (err != ESP_OK) {
        // error msg
    }
}

static void ipac_uart_cmd_send_rpr_link_status(uint8_t uart_opcode, esp_ble_mesh_rpr_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_rpr_link_status_t msg = {0};

    msg.opcode = uart_opcode;
    msg.unicast = param->recv.params->ctx.addr;
    msg.status = param->recv.val.link_status.status;
    msg.rpr_state = param->recv.val.link_status.rpr_state;

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_RPR_LINK_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_RPR_LINK_STATUS);
} 

static void ipac_uart_cmd_send_rpr_link_report(esp_ble_mesh_rpr_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_rpr_link_report_t msg = {0};

    msg.opcode = OPCODE_RPR_LINK_REPORT;
    msg.unicast = param->recv.params->ctx.addr;
    msg.status = param->recv.val.link_report.status;
    msg.rpr_state = param->recv.val.link_report.rpr_state;
    msg.reason_en = (uint8_t)(param->recv.val.link_report.reason_en);
    msg.reason = param->recv.val.link_report.reason;
    
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_RPR_LINK_REPORT);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_RPR_LINK_REPORT);
}

static void ipac_uart_cmd_recv_remote_prov(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    esp_ble_mesh_rpr_client_act_param_t param = {0};
    uint16_t remote_rpr_srv_addr = 0;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_REMOTE_PROVISIONING, MSG_ARG_SIZE_REMOTE_PROV) != 0x00)
    {
        // error msg
        return;
    }
    
    param.start_rpr.model = remote_prov_client.model;
    param.start_rpr.rpr_srv_addr = ((ipac_ble_mesh_msg_recv_remote_prov_t*)arg)->unicast;

    /* Let remote provisioning server start provisioning */
    err = esp_ble_mesh_rpr_client_action(ESP_BLE_MESH_RPR_CLIENT_ACT_START_RPR, &param);
    if (err) {
        // ESP_LOGE(TAG, "Failed to perform Remote Provisioning Client action: Start Prov");
    }
}

static void ipac_uart_cmd_send_remote_prov_ack(esp_ble_mesh_rpr_client_cb_param_t *param) {
    ipac_ble_mesh_msg_send_remote_prov_ack_t msg = {0};

    msg.opcode = OPCODE_REMOTE_PROVISIONING;
    msg.status = (param->act.start_rpr_comp.err_code == 0) ? RESPONSE_BYTE_STATUS_OK : RESPONSE_BYTE_STATUS_FAILED;
    msg.unicast = param->act.start_rpr_comp.rpr_srv_addr;
    
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_REMOTE_PROV_ACK);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_REMOTE_PROV_ACK);
}

#endif

static void ipac_uart_cmd_recv_sensor_data_get(void *arg, uint8_t status) {
    esp_ble_mesh_model_publish(sensor_client.model, SENSOR_MODEL_OPCODE_GET, 0, NULL, MSG_ROLE);
}

static void ipac_uart_cmd_send_sensor_data_status(esp_ble_mesh_model_cb_param_t *param, bool publish) {
    ipac_ble_mesh_msg_send_sensor_data_status_t msg = {0};

    msg.opcode = OPCODE_SENSOR_DATA_STATUS;
    if (publish == false) {
        msg.unicast = param->model_operation.ctx->addr;
        msg.id = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->id;
        msg.temp = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->temp;
        msg.humid = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->humid;
        msg.light = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->light;
        msg.co2 = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->co2;
        msg.motion = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->motion;
        msg.dust = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->dust;
        msg.battery = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->battery;
    }
    else {
        msg.unicast = param->client_recv_publish_msg.ctx->addr;
        msg.id = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->id;
        msg.temp = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->temp;
        msg.humid = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->humid;
        msg.light = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->light;
        msg.co2 = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->co2;
        msg.motion = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->motion;
        msg.dust = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->dust;
        msg.battery = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->client_recv_publish_msg.msg))->battery;
    }
    
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_SENSOR_DATA_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_SENSOR_DATA_STATUS);
}

static void ipac_uart_cmd_recv_ac_control_state_set(void *arg, uint8_t status) {
    esp_ble_mesh_client_common_param_t common = {0};
    ipac_ble_mesh_model_msg_ac_control_state_set_t msg = {0};
    esp_err_t err = ESP_OK;

    if (status != PACKET_OK) {
        // error msg
        return;
    }

    if (ipac_cal_checksum(arg, OPCODE_AC_CONTROL_STATE_SET, MSG_ARG_SIZE_AC_CONTROL_STATE_SET) != 0x00)
    {
        // error msg
        return;
    }

    err = ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_ac_control_state_set_t*)arg)->unicast, 
        ac_control_client.model, AC_CONTROL_STATE_OPCODE_SET);

    if (err != ESP_OK) {
        return;
    }

    msg.device_id = ((ipac_ble_mesh_msg_recv_ac_control_state_set_t*)arg)->device_id;
    msg.device_state = ((ipac_ble_mesh_msg_recv_ac_control_state_set_t*)arg)->device_state;

    err = esp_ble_mesh_client_model_send_msg(ac_control_client.model, &(common.ctx), common.opcode, 
        sizeof(msg), (uint8_t*) &msg, MSG_TIMEOUT, true, MSG_ROLE);
    if (err != ESP_OK) {
        return;
    }
}

static void ipac_uart_cmd_send_ac_control_state_status(esp_ble_mesh_model_cb_param_t *param, uint8_t opcode, bool publish) {
    ipac_ble_mesh_msg_send_ac_control_state_status_t msg = {0};

    msg.opcode = opcode;
    if (publish == false) {
        msg.unicast = param->model_operation.ctx->addr;
        msg.device_id = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->model_operation.msg))->device_id;
        msg.device_state = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->model_operation.msg))->device_state;
        msg.status = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->model_operation.msg))->status;
    }
    else {
        msg.unicast = param->client_recv_publish_msg.ctx->addr;
        msg.device_id = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->client_recv_publish_msg.msg))->device_id;
        msg.device_state = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->client_recv_publish_msg.msg))->device_state;
        msg.status = ((ipac_ble_mesh_model_msg_ac_control_state_status_t*)(param->client_recv_publish_msg.msg))->status;
    }
    
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_AC_CONTROL_STATE_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_AC_CONTROL_STATE_STATUS);
}

static void ipac_uart_cmd_recv_device_info_get(void *arg, uint8_t status) {
    esp_ble_mesh_model_publish(device_info_client.model, DEVICE_INFO_MODEL_OPCODE_GET, 0, NULL, MSG_ROLE);
}

static void ipac_uart_cmd_send_device_info_status(esp_ble_mesh_model_cb_param_t *param) {
    ipac_ble_mesh_msg_send_device_info_status_t msg = {0};

    msg.opcode = OPCODE_SENSOR_DATA_STATUS;
    msg.unicast = param->model_operation.ctx->addr;
    msg.function = ((ipac_ble_mesh_model_msg_device_info_status_t*)(param->model_operation.msg))->function;
    msg.tx_power = ((ipac_ble_mesh_model_msg_device_info_status_t*)(param->model_operation.msg))->tx_power;
    memcpy(msg.device_name, ((ipac_ble_mesh_model_msg_device_info_status_t*)(param->model_operation.msg))->device_name, DEVICE_NAME_MAX_SIZE);

    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_DEVICE_INFO_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_DEVICE_INFO_STATUS);
}

/* Test UART communication */
#if defined(DEBUG_TEST_COM) && (DEBUG_TEST_COM == 1)
static void test_simple_msg(void *arg, uint8_t status) {
    esp_err_t err = ESP_OK;
    uint8_t res[3] = {OPCODE_TEST_SIMPLE_MSG};

    res[1] = (err != ESP_OK) ? RESPONSE_BYTE_STATUS_FAILED : RESPONSE_BYTE_STATUS_OK;
    res[2] = ipac_cal_checksum((void*) res, 0, sizeof(res));
    uart_write_bytes(UART_PORT_NUM, (const void *) res, sizeof(res));
}
#endif /* End test UART communication */

static void serial_com_init() {
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
}

static void serial_com_task() {
    ipac_uart_cmd_buffer_t cmd_item = {0};
    // Configure a temporary buffer for the incoming command
    // uint8_t *command = (uint8_t *) malloc(BUF_SIZE);
    uint8_t command[BUF_SIZE] = {0};
    // Read data from the UART
    // memset(command, 0, BUF_SIZE);
    
    int len = uart_read_bytes(UART_PORT_NUM, command, 1, 20 / portTICK_PERIOD_MS);
    if (len <= 0) {
        return;
    }

    // Check if command is match
    for (int i = 0; i < ARRAY_SIZE(uart_cmd); i++) {
        if (uart_cmd[i].opcode == command[0]) {
            cmd_item.opcode = uart_cmd[i].opcode;
            cmd_item.handler = uart_cmd[i].handler;
            if (uart_cmd[i].msg_arg_size == MSG_ARG_NONE) {
                cmd_item.len = MSG_ARG_NONE;
                ipac_uart_cmd_queue_enqueue(&cmd_queue, &cmd_item);
                break;
            }
            int len = uart_read_bytes(UART_PORT_NUM, cmd_item.arg, uart_cmd[i].msg_arg_size, 1000 / portTICK_PERIOD_MS);
            if (len == uart_cmd[i].msg_arg_size) {
                cmd_item.len = uart_cmd[i].msg_arg_size;
                ipac_uart_cmd_queue_enqueue(&cmd_queue, &cmd_item);
            }
            break;
        }
    }
}

static void ipac_uart_cmd_handle_task() {
    ipac_uart_cmd_buffer_t cmd;
    uint8_t status = ipac_uart_cmd_queue_dequeue(&cmd_queue, &cmd);
    if (status == 0) {
        cmd.handler((void*) cmd.arg, PACKET_OK);
    }
}

static void main_handle_task() {
    while(1) {
        serial_com_task();
        ipac_uart_cmd_handle_task();
    }
}

/********************************************************************
 * Bluetooth App Functions
 ********************************************************************/
static void prov_link_open(esp_ble_mesh_prov_bearer_t bearer)
{
    // ESP_LOGI(TAG, "%s link open", bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
}

static void prov_link_close(esp_ble_mesh_prov_bearer_t bearer, uint8_t reason)
{
    // ESP_LOGI(TAG, "%s link close, reason 0x%02x",
            //  bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", reason);
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    int err;

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
    * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
    * to the application layer.
    */

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (esp_ble_mesh_addr_type_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, 16);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (esp_ble_mesh_prov_bearer_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we should not add
            device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
            (esp_ble_mesh_dev_add_flag_t)(ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG));
    if (err) {
        // ESP_LOGE(TAG, "%s: Add unprovisioned device into queue failed", __func__);
    }

    return;
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                            esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        ipac_uart_cmd_send_scan_result(param);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        prov_link_open(param->provisioner_prov_link_open.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        prov_link_close(param->provisioner_prov_link_close.bearer, param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        ipac_uart_cmd_send_add_device_complete((void*) param, false);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT: {
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK) {
            esp_err_t err = ESP_OK;
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            // when adding vendor model to provisioner, bind them here
            err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                    SENSOR_MODEL_ID_CLIENT, CID_ESP);
            if (err != ESP_OK) {
                return;
            }
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        if (param->node_bind_app_key_to_model_comp.err_code == ESP_OK) {
            esp_err_t err = ESP_OK;
            err = esp_ble_mesh_model_subscribe_group_addr(PROV_OWN_ADDR, CID_ESP, SENSOR_MODEL_ID_CLIENT, GROUP_ADDRESS);
            if (err != ESP_OK) {
                 return;
            }
        }
        // gpio_set_level(GPIO_NUM_2, 1);
        break;
    case ESP_BLE_MESH_PROVISIONER_UPDATE_LOCAL_NET_KEY_COMP_EVT: 
        if (param->provisioner_update_net_key_comp.err_code != ESP_OK) {
            ipac_uart_cmd_send_update_local_keys_status(RESPONSE_BYTE_STATUS_FAILED);
            break;
        }
        prov_key.net_idx = param->provisioner_update_net_key_comp.net_idx;
        esp_err_t err = esp_ble_mesh_provisioner_update_local_app_key(prov_key.app_key_cache, ESP_BLE_MESH_KEY_PRIMARY, APP_KEY_IDX);
        if (err != ESP_OK) {
            ipac_uart_cmd_send_update_local_keys_status(RESPONSE_BYTE_STATUS_FAILED);
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_UPDATE_LOCAL_APP_KEY_COMP_EVT: 
        if (param->provisioner_update_app_key_comp.err_code != ESP_OK) {
            ipac_uart_cmd_send_update_local_keys_status(RESPONSE_BYTE_STATUS_FAILED);
            break;
        }
        prov_key.app_idx = param->provisioner_update_app_key_comp.app_idx;
        ipac_uart_cmd_send_update_local_keys_status(RESPONSE_BYTE_STATUS_OK);
        break;
    case ESP_BLE_MESH_PROVISIONER_DELETE_NODE_WITH_ADDR_COMP_EVT:
        ipac_uart_cmd_send_delete_device_status(param);
        break;

    default:
        break;
    }

    return;
}

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                            esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    uint32_t opcode;
    uint16_t addr;
    int err;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    if (param->error_code) {
        // ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04" PRIx32, opcode);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            ipac_uart_cmd_composition_data_status(param);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_RELAY_GET:
            ipac_uart_cmd_send_relay_status(param, OPCODE_RELAY_GET);
            break;
        case ESP_BLE_MESH_MODEL_OP_GATT_PROXY_GET:
            ipac_uart_cmd_send_proxy_status(param, OPCODE_PROXY_GET);
            break;
        case ESP_BLE_MESH_MODEL_OP_FRIEND_GET:
            ipac_uart_cmd_send_friend_status(param, OPCODE_FRIEND_GET);
            break;
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
            ipac_uart_cmd_send_app_key_status(param);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
            ipac_uart_cmd_send_model_app_status(param);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET: {
            ipac_uart_cmd_send_model_pub_status(param);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD: {
            ipac_uart_cmd_send_model_sub_status(param);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_RELAY_SET: {
            ipac_uart_cmd_send_relay_status(param, OPCODE_RELAY_SET);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_GATT_PROXY_SET: {
            ipac_uart_cmd_send_proxy_status(param, OPCODE_PROXY_GET);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_FRIEND_SET: {
            ipac_uart_cmd_send_friend_status(param, OPCODE_FRIEND_GET);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_GET: {
            ipac_uart_cmd_send_friend_status(param, OPCODE_HEARTBEAT_PUB_GET);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET: {
            ipac_uart_cmd_send_friend_status(param, OPCODE_HEARTBEAT_PUB_SET);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS:
            // ESP_LOG_BUFFER_HEX("composition data %s", param->status_cb.comp_data_status.composition_data->data,
                            //    param->status_cb.comp_data_status.composition_data->len);
            ipac_uart_cmd_composition_data_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_STATUS:
            // ipac_uart_cmd_send_app_key_status(param);
            break;
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            esp_ble_mesh_cfg_client_get_state_t get_state = {0};
            // ipac_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get_state.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get_state);
            if (err) {
                // ESP_LOGE(TAG, "%s: Config Composition Data Get failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            // ipac_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err) {
                // ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            // ipac_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            // set_state.model_app_bind.element_addr = node->unicast;
            set_state.model_app_bind.model_app_idx = prov_key.app_idx;
            set_state.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            set_state.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err) {
                // ESP_LOGE(TAG, "%s: Config Model App Bind failed", __func__);
                return;
            }
            break;
        }
        default:
            break;
        }
        break;
    default:
        // ESP_LOGE(TAG, "Not a config client status message event");
        break;
    }
}

void ipac_ble_mesh_send_ac_control_state_msg(bool resend)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    uint32_t opcode;
    esp_err_t err;

    ctx.net_idx = prov_key.net_idx;
    ctx.app_idx = prov_key.app_idx;
    // ctx.addr = server_addr; // see in vendor model exp
    ctx.send_ttl = MSG_SEND_TTL;
    opcode = SENSOR_MODEL_OPCODE_SET;

    if (resend == false) {
        // store.vnd_tid++;
    }
}

static void example_ble_mesh_remote_prov_client_callback(esp_ble_mesh_rpr_client_cb_event_t event,
                                        esp_ble_mesh_rpr_client_cb_param_t *param)
{
    static uint8_t remote_dev_uuid[16] = {0};
    esp_ble_mesh_rpr_client_msg_t msg = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;
    uint16_t addr = 0;

    switch (event) {
        case ESP_BLE_MESH_RPR_CLIENT_SEND_COMP_EVT:
            // ESP_LOGW(TAG, "Remote Prov Client Send Comp, err_code %d", param->send.err_code);
            break;
        case ESP_BLE_MESH_RPR_CLIENT_SEND_TIMEOUT_EVT:
            // ESP_LOGW(TAG, "Remote Prov Client Send Timeout, opcode 0x%04x, to 0x%04x",
            //          param->send.params->opcode, param->send.params->ctx.addr);
            break;
        case ESP_BLE_MESH_RPR_CLIENT_RECV_PUB_EVT:
        case ESP_BLE_MESH_RPR_CLIENT_RECV_RSP_EVT:
            // ESP_LOGW(TAG, "Remote Prov Client Recv RSP, opcode 0x%04x, from 0x%04x",
            //          param->recv.params->ctx.recv_op, param->recv.params->ctx.addr);
            switch (param->recv.params->ctx.recv_op) {
                case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_CAPS_STATUS:
                    break;
                case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_STATUS:
                    switch (param->recv.params->opcode) {
                        case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_GET: 
                            ipac_uart_cmd_send_rpr_scan_status(OPCODE_RPR_SCAN_GET, param);
                            break;
                        case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_START:
                            ipac_uart_cmd_send_rpr_scan_status(OPCODE_RPR_SCAN_START, param);
                            break;
                        case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_STOP:
                            ipac_uart_cmd_send_rpr_scan_status(OPCODE_RPR_SCAN_STOP, param);
                            break;
                    }
                    break;
                case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_REPORT:
                    ipac_uart_cmd_send_rpr_scan_report(param);
                    break;
                case ESP_BLE_MESH_MODEL_OP_RPR_EXT_SCAN_REPORT:
                    break;
                case ESP_BLE_MESH_MODEL_OP_RPR_LINK_STATUS:
                    switch (param->recv.params->ctx.recv_op) {
                        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_GET:
                            ipac_uart_cmd_send_rpr_link_status(OPCODE_RPR_LINK_GET, param);
                            break;
                        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_OPEN: 
                            ipac_uart_cmd_send_rpr_link_status(OPCODE_RPR_LINK_OPEN, param);
                            break;
                        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_CLOSE: 
                            ipac_uart_cmd_send_rpr_link_status(OPCODE_RPR_LINK_CLOSE, param);
                            break;
                        default:
                            ESP_LOGW(TAG, "Unknown Process opcode 0x%04x:%d", cur_rpr_cli_opcode,__LINE__);
                            break;
                        }
                        break;
                case ESP_BLE_MESH_MODEL_OP_RPR_LINK_REPORT:
                    ipac_uart_cmd_send_rpr_link_report(param);
                    break;
                case ESP_BLE_MESH_MODEL_OP_RPR_LINK_CLOSE:      // no need
                    switch (param->recv.val.link_report.status)
                    {
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_BY_CLIENT:
                        ESP_LOGI(TAG, "Link closed by client");
                        break;
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_BY_DEVICE:
                        ESP_LOGI(TAG, "Link closed by device");
                        break;
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_BY_SERVER:
                        ESP_LOGI(TAG, "Link closed by server");
                        break;
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_AS_CANNOT_RECEIVE_PDU:
                        ESP_LOGI(TAG, "Link closed as cannot receive pdu");
                        break;
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_AS_CANNOT_SEND_PDU:
                        ESP_LOGI(TAG, "Link closed as cannot send pdu");
                        break;
                    case ESP_BLE_MESH_RPR_STATUS_LINK_CLOSED_AS_CANNOT_DELIVER_PDU_REPORT:
                        ESP_LOGI(TAG, "Link closed as cannot send pdu report");
                        break;
                    default:
                        ESP_LOGW(TAG, "Unknown link close status, %d", param->recv.val.link_report.status);
                        break;
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown rocess opcode 0x%04x:%d", cur_rpr_cli_opcode,__LINE__);
                    break;
            }
            break;
        case ESP_BLE_MESH_RPR_CLIENT_ACT_COMP_EVT:          // start remote prov successfully
            ipac_uart_cmd_send_remote_prov_ack(param);
            break;
        case ESP_BLE_MESH_RPR_CLIENT_LINK_OPEN_EVT:
            // ESP_LOGW(TAG, "Remote Prov Client Link Open");
            break;
        case ESP_BLE_MESH_RPR_CLIENT_LINK_CLOSE_EVT:
            // ESP_LOGW(TAG, "Remote Prov Client Link Close");
            break;
        case ESP_BLE_MESH_RPR_CLIENT_PROV_COMP_EVT:
            ipac_uart_cmd_send_add_device_complete((void*) param, true);
            break;
    default:
        break;
    }
}

static void ipac_ble_mesh_sensor_cli_model_cb(esp_ble_mesh_model_cb_event_t event,
                                            esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        // see in vendor models exp
        if (param->model_operation.opcode == SENSOR_MODEL_OPCODE_STATUS) {
            ipac_uart_cmd_send_sensor_data_status(param, false);
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        // if (param->model_send_comp.err_code) {
        //     // ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
        //     break;
        // }
        // start_time = esp_timer_get_time();
        // // ESP_LOGI(TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        // ESP_LOGI(TAG, "Receive publish message 0x%06" PRIx32, param->client_recv_publish_msg.opcode);
        gpio_set_level(GPIO_NUM_2, 0);
        ipac_uart_cmd_send_sensor_data_status(param, true);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        // ESP_LOGW(TAG, "Client message 0x%06" PRIx32 " timeout", param->client_send_timeout.opcode);
        // example_ble_mesh_send_vendor_message(true);
        break;
    default:
        break;
    }
}

static void ipac_ble_mesh_ac_control_cli_model_cb(esp_ble_mesh_model_cb_event_t event,
    esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
        case ESP_BLE_MESH_MODEL_OPERATION_EVT:
            if (param->model_operation.opcode == AC_CONTROL_STATE_OPCODE_STATUS) {
                if (param->model_operation.ctx->recv_op == AC_CONTROL_STATE_OPCODE_GET) {
                    ipac_uart_cmd_send_ac_control_state_status(param, AC_CONTROL_STATE_OPCODE_GET, false);
                }
                else if (param->model_operation.ctx->recv_op == AC_CONTROL_STATE_OPCODE_SET) {
                    ipac_uart_cmd_send_ac_control_state_status(param, AC_CONTROL_STATE_OPCODE_SET, false);
                }
            }
            break;
        case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
            break;
        case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
            if (param->client_recv_publish_msg.opcode == AC_CONTROL_STATE_OPCODE_STATUS) {
                if (param->client_recv_publish_msg.ctx->recv_op == AC_CONTROL_STATE_OPCODE_GET) {
                    ipac_uart_cmd_send_ac_control_state_status(param, AC_CONTROL_STATE_OPCODE_GET, true);
                }
                else if (param->client_recv_publish_msg.ctx->recv_op == AC_CONTROL_STATE_OPCODE_SET) {
                    ipac_uart_cmd_send_ac_control_state_status(param, AC_CONTROL_STATE_OPCODE_SET, true);
                }
            }
            // ipac_uart_cmd_send_sensor_data_status(param, true);
            break;
        case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
            // ESP_LOGW(TAG, "Client message 0x%06" PRIx32 " timeout", param->client_send_timeout.opcode);
            // example_ble_mesh_send_vendor_message(true);
            break;
        default:
            break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = {0xdd, 0xdd};
    esp_err_t err = ESP_OK;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_rpr_client_callback(example_ble_mesh_remote_prov_client_callback);
    esp_ble_mesh_register_custom_model_callback(ipac_ble_mesh_sensor_cli_model_cb);
    esp_ble_mesh_register_custom_model_callback(ipac_ble_mesh_ac_control_cli_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        return err;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(vnd_models); i++) {
        err = esp_ble_mesh_client_model_init(&vnd_models[i]);
        if (err) {
            return err;
        }
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to set matching device uuid (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to enable mesh provisioner (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        return err;
    }

    // ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
}

static void gpio_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
}

static void print_reset_reason() {
    esp_reset_reason_t reason;
    reason = esp_reset_reason();
    for (uint8_t i = 0; i < reason; i++) {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t err;

    // ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    gpio_init();

    // gpio_set_level(GPIO_NUM_2, 1);
    // vTaskDelay(300 / portTICK_PERIOD_MS);
    // gpio_set_level(GPIO_NUM_2, 0);
    // vTaskDelay(300 / portTICK_PERIOD_MS);
    // gpio_set_level(GPIO_NUM_2, 1);
    // vTaskDelay(300 / portTICK_PERIOD_MS);
    // gpio_set_level(GPIO_NUM_2, 0);
    // vTaskDelay(300 / portTICK_PERIOD_MS);

    err = bluetooth_init();
    if (err) {
        // ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        // ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    ipac_uart_cmd_queue_init(&cmd_queue);
    serial_com_init();

    print_reset_reason();

    /* Run command handling task */
    xTaskCreate(main_handle_task, "main_handle_task", TASK_STACK_SIZE * 2, NULL, 10, NULL);
}
