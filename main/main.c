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

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "inc/msg-defs.h"
#include "inc/model.h"

#include "ble_mesh_example_init.h"

/********************************************************************
 * Definitions
 ********************************************************************/
#define TAG "EXAMPLE"
#define DEBUG_TEST_COM      1

#define LED_OFF             0x0
#define LED_ON              0x1

#define PROV_OWN_ADDR       0x0001

#define MSG_SEND_TTL        3
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_PROVISIONER

#define COMP_DATA_PAGE_0    0x00

#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

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
static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[16];
    uint8_t  app_key_cache[16];
} prov_key;

static uint8_t dev_uuid[16];
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

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),     // this is for connecting to smart devices
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
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
    ESP_BLE_MESH_MODEL_OP(SENSOR_MODEL_OPCODE_STATUS,
                        sizeof(ipac_ble_mesh_model_msg_sensor_data_status_t)),
    ESP_BLE_MESH_MODEL_OP_END,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, 1, MSG_ROLE);

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

ESP_BLE_MESH_MODEL_PUB_DEFINE(device_info_cli_pub, 1, MSG_ROLE);

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, SENSOR_MODEL_ID_CLIENT,
    device_info_model_op, &device_info_cli_pub, &device_info_client),
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, SENSOR_MODEL_ID_CLIENT,
    sensor_model_op, &sensor_cli_pub, &sensor_client),
};

static esp_ble_mesh_model_t* device_info_cli_model = &vnd_models[0];
static esp_ble_mesh_model_t* sensor_cli_model = &vnd_models[1];

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
static void ipac_uart_cmd_recv_sensor_data_get(void *arg, uint8_t status);
static void ipac_uart_cmd_recv_device_info_get(void *arg, uint8_t status);

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
    {OPCODE_DEVICE_INFO_GET, MSG_ARG_SIZE_DEVICE_INFO_GET, ipac_uart_cmd_recv_device_info_get},
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
    memcpy(res.addr, add_dev.addr, 16);
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

static void ipac_uart_cmd_send_add_device_complete(esp_ble_mesh_prov_cb_param_t *param) {
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
    int err;
    ipac_ble_mesh_msg_send_new_node_info_t node = {
        .node_idx = param->provisioner_prov_complete.node_idx,
        .unicast = param->provisioner_prov_complete.unicast_addr,
        .net_idx = param->provisioner_prov_complete.netkey_idx,
        .elem_num = param->provisioner_prov_complete.element_num,
    };
    memcpy(node.uuid, param->provisioner_prov_complete.device_uuid, 16);

    err = send_new_node_info_msg(&node);
    if (err) {
        // ESP_LOGE(TAG, "%s: Store node info failed", __func__);
    }

    // get node info:
    // - primary address is unicast (ESP_BLE_MESH_ADDR_IS_UNICAST)
    // - search database with primary address

    // ipac_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    // get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    // err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    // if (err) {
    //     // ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
    //     return ESP_FAIL;
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

    ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_get_composition_data_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err) {
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

    ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_add_app_key_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
    set_state.app_key_add.net_idx = prov_key.net_idx;
    set_state.app_key_add.app_idx = prov_key.app_idx;
    memcpy(set_state.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) {
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

    ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_bind_model_app_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
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

    ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET);
    set_state.model_pub_set.element_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast;
    set_state.model_pub_set.publish_app_idx = prov_key.app_idx;
    set_state.model_pub_set.publish_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->group_addr;
    set_state.model_pub_set.model_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->model_id;
    set_state.model_pub_set.company_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->cid;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) {
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

    ipac_ble_mesh_set_msg_common(&common, ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD);
    set_state.model_sub_add.element_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->unicast;
    set_state.model_sub_add.sub_addr = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->group_addr;
    set_state.model_sub_add.model_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->model_id;
    set_state.model_sub_add.company_id = ((ipac_ble_mesh_msg_recv_model_pub_sub_set_t*)arg)->cid;
    err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) {
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

static void ipac_uart_cmd_recv_sensor_data_get(void *arg, uint8_t status) {
    esp_ble_mesh_model_publish(sensor_cli_model, SENSOR_MODEL_OPCODE_GET, 0, NULL, MSG_ROLE);
}

static void ipac_uart_cmd_send_sensor_data_status(esp_ble_mesh_model_cb_param_t *param) {
    ipac_ble_mesh_msg_send_sensor_data_status_t msg = {0};

    msg.opcode = OPCODE_SENSOR_DATA_STATUS;
    msg.unicast = param->model_operation.ctx->addr;
    msg.temp = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->temp;
    msg.humid = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->humid;
    msg.light = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->light;
    msg.co2 = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->co2;
    msg.motion = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->motion;
    msg.dust = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->dust;
    msg.battery = ((ipac_ble_mesh_model_msg_sensor_data_status_t*)(param->model_operation.msg))->battery;
    
    msg.checksum = ipac_cal_checksum((void*) &msg, 0, MSG_SIZE_SENSOR_DATA_STATUS);
    uart_write_bytes(UART_PORT_NUM, (const void *) &msg, MSG_SIZE_SENSOR_DATA_STATUS);
}

static void ipac_uart_cmd_recv_device_info_get(void *arg, uint8_t status) {
    esp_ble_mesh_model_publish(device_info_cli_model, DEVICE_INFO_MODEL_OPCODE_GET, 0, NULL, MSG_ROLE);
}

static void ipac_uart_cmd_send_sensor_data_status(esp_ble_mesh_model_cb_param_t *param) {
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

static void serial_com_task() {
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

    // Configure a temporary buffer for the incoming command
    uint8_t *command = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        memset(command, 0, BUF_SIZE);
        int len = uart_read_bytes(UART_PORT_NUM, command, 1, 20 / portTICK_PERIOD_MS);
        if (len <= 0) {
            continue;
        }
        // Check if command is match
        for (int i = 0; i < ARRAY_SIZE(uart_cmd); i++) {
            if (uart_cmd[i].opcode == command[0]) {
                if (uart_cmd[i].msg_arg_size == MSG_ARG_NONE) {
                    uart_cmd[i].handler((void*) command, PACKET_OK);
                    break;
                }
                int len = uart_read_bytes(UART_PORT_NUM, command, uart_cmd[i].msg_arg_size, 1000 / portTICK_PERIOD_MS);
                uart_cmd[i].handler((void*) command, PACKET_OK);
                // if (len != uart_cmd[i].msg_arg_size) {
                //     uart_cmd[i].handler((void*) command, PACKET_LOSS);
                //     break;
                // }
                // else {
                //     uart_cmd[i].handler((void*) command, PACKET_OK);
                //     break;
                // }
            }
        }
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
        ipac_uart_cmd_send_add_device_complete(param);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT: {
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK) {
            esp_err_t err = 0;
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            // when adding vendor model to provisioner, bind them here
            err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                    ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
            if (err != ESP_OK) {
                return;
            }
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
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

    // ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04" PRIx32,
            //  __func__, param->error_code, event, param->params->ctx.addr, opcode);

    if (param->error_code) {
        // ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04" PRIx32, opcode);
        return;
    }

    // node = example_ble_mesh_get_node_info(addr);
    // if (!node) {
    //     // ESP_LOGE(TAG, "%s: Get node info failed", __func__);
    //     return;
    // }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            ipac_uart_cmd_composition_data_status(param);
            break;
        }
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
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS:
            // ESP_LOG_BUFFER_HEX("composition data %s", param->status_cb.comp_data_status.composition_data->data,
                            //    param->status_cb.comp_data_status.composition_data->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_STATUS:
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

void example_ble_mesh_send_vendor_message(bool resend)
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

    // err = esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, opcode,
    //         sizeof(store.vnd_tid), (uint8_t *)&store.vnd_tid, MSG_TIMEOUT, true, MSG_ROLE);
    // if (err != ESP_OK) {
    //     // ESP_LOGE(TAG, "Failed to send vendor message 0x%06" PRIx32, opcode);
    //     return;
    // }

    // mesh_example_info_store(); /* Store proper mesh example info */
}

static void ipac_ble_mesh_sensor_cli_model_cb(esp_ble_mesh_model_cb_event_t event,
                                            esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        // see in vendor models exp
        if (param->model_operation.opcode == SENSOR_MODEL_OPCODE_STATUS) {
            ipac_uart_cmd_send_sensor_data_status(param);
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
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        // ESP_LOGW(TAG, "Client message 0x%06" PRIx32 " timeout", param->client_send_timeout.opcode);
        example_ble_mesh_send_vendor_message(true);
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
    // esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb);
    esp_ble_mesh_register_custom_model_callback(ipac_ble_mesh_sensor_cli_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_client_model_init(sensor_cli_model);
    if (err) {
        return err;
    }

    err = esp_ble_mesh_client_model_init(device_info_cli_model);
    if (err) {
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to set matching device uuid (err %d)", err);
        return err;
    }

    // err = esp_ble_mesh_provisioner_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    // if (err != ESP_OK) {
    //     // ESP_LOGE(TAG, "Failed to enable mesh provisioner (err %d)", err);
    //     return err;
    // }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        return err;
    }

    // ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
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
    /* Run communication handling task */
    xTaskCreate(serial_com_task, "serial_com_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
