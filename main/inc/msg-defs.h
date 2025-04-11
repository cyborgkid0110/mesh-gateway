#include <stdint.h>
#include "esp_ble_mesh_defs.h"

// command opcode
#define OPCODE_GET_LOCAL_KEYS           0x01
#define OPCODE_UPDATE_LOCAL_KEYS        0x02
#define OPCODE_SCAN_UNPROV_DEV          0x03
#define OPCODE_PROVISIONER_DISABLE      0x04
#define OPCODE_ADD_UNPROV_DEV           0x05
#define OPCODE_DELETE_DEVICE            0x06
#define OPCODE_GET_COMPOSITION_DATA     0x07
#define OPCODE_ADD_APP_KEY              0x08
#define OPCODE_BIND_MODEL_APP           0x09
#define OPCODE_SET_MODEL_PUB            0x0A
#define OPCODE_SET_MODEL_SUB            0x0B

#define OPCODE_SCAN_RESULT              0x40
#define OPCODE_SEND_NEW_NODE_INFO       0x41

#define OPCODE_SENSOR_DATA_GET          0x50
#define OPCODE_SENSOR_DATA_STATUS       0x51

#define OPCODE_DEVICE_INFO_GET          0x80
#define OPCODE_DEVICE_INFO_STATUS       0x81

#define OPCODE_TEST_SIMPLE_MSG          0xC4

// received packet state
#define PACKET_OK   0x00
#define PACKET_LOSS 0x01

// byte definition
#define RESPONSE_BYTE_STATUS_OK     0x01
#define RESPONSE_BYTE_STATUS_FAILED 0x02

#define DEVICE_NAME_MAX_SIZE        20

/********************************************************************
 * Data structures
 ********************************************************************/
typedef uint8_t ipac_opcode_t;

// command menu structure
typedef struct ipac_uart_command {
    uint8_t opcode;
    uint8_t msg_arg_size;
    void (*handler)(void*, uint8_t);
} ipac_uart_command_t;

// msg structure for sending local keys
typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_get_local_keys {
    uint8_t opcode;         // 1 byte
    uint8_t status;         // 1 byte
    uint8_t net_key[16];    // 16 bytes
    uint8_t app_key[16];    // 16 bytes
    uint8_t checksum;       // 1 byte
} ipac_ble_mesh_msg_send_get_local_keys_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_update_local_keys {
    uint8_t net_key[16];    // 16 bytes
    uint8_t app_key[16];    // 16 bytes
    uint8_t checksum;       // 1 byte
} ipac_ble_mesh_msg_recv_update_local_keys_t;

// msg structure for unprovisioned device data
typedef struct __attribute__((packed)) ipac_ble_mesh_msg_scan_result {
    uint8_t opcode;
    uint8_t uuid[16];                   // 16 bytes
    uint8_t addr[BD_ADDR_LEN];          // 6 bytes (BD_ADDR_LEN = 6)
    esp_ble_mesh_addr_type_t addr_type; // 1 byte
    uint16_t oob_info;                  // 2 bytes
    uint8_t adv_type;                   // 1 byte
    uint8_t bearer_type;                // 1 byte
    int8_t  rssi;                       // 1 byte
    uint8_t checksum;
} ipac_ble_mesh_msg_scan_result_t;

// msg structure for unprovisioned device data
typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_add_unprov_dev {
    uint8_t uuid[16];                   // 16 bytes
    uint8_t addr[BD_ADDR_LEN];          // 6 bytes (BD_ADDR_LEN = 6)
    esp_ble_mesh_addr_type_t addr_type; // 1 byte
    uint16_t oob_info;                  // 2 bytes
    uint8_t bearer_type;                // 1 byte
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_add_unprov_dev_t;

// msg structure for unprovisioned device data
typedef struct __attribute__((packed)) ipac_ble_mesh_msg_sent_add_unprov_dev_ack {
    uint8_t opcode;                     // 1 bytes
    uint8_t uuid[16];                   // 16 bytes
    uint8_t addr[BD_ADDR_LEN];          // 6 bytes (BD_ADDR_LEN = 6)
    esp_ble_mesh_addr_type_t addr_type; // 1 byte
    uint16_t oob_info;                  // 2 bytes
    uint8_t bearer_type;                // 1 byte
    uint8_t status;                     // 1 byte
    uint8_t checksum;
} ipac_ble_mesh_msg_sent_add_unprov_dev_ack_t;

// msg structure for node data (provisioned device)
typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_new_node_info {
    ipac_opcode_t opcode;
    uint16_t node_idx;
    uint8_t  uuid[16];
    uint16_t unicast;
    uint16_t net_idx;
    uint8_t elem_num;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_new_node_info_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_delete_device {
    uint16_t unicast;
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_delete_device_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_delete_device_status {
    uint8_t opcode;
    uint16_t unicast;
    uint8_t status;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_delete_device_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_get_composition_data {
    uint16_t unicast;
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_get_composition_data_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_common_target_info {
    uint16_t net_idx;
    uint16_t app_idx;
    uint16_t unicast;
} ipac_ble_mesh_msg_common_target_info_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_add_app_key {
    uint16_t unicast;
    uint8_t app_key[16];
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_add_app_key_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_add_app_key_status {
    uint8_t opcode;
    uint16_t unicast;
    uint8_t status;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_add_app_key_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_bind_model_app {
    uint16_t unicast;
    uint16_t model_id;
    uint16_t cid;
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_bind_model_app_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_model_app_status {
    uint8_t opcode;
    uint16_t unicast;
    uint16_t model_id;
    uint16_t cid;
    uint8_t status;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_model_app_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_recv_model_pub_sub_set {
    uint16_t unicast;
    uint16_t group_addr;
    uint16_t model_id;
    uint16_t cid;
    uint8_t checksum;
} ipac_ble_mesh_msg_recv_model_pub_sub_set_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_model_pub_sub_status {
    uint8_t opcode;
    uint16_t unicast;
    uint16_t group_addr;
    uint16_t model_id;
    uint16_t cid;
    uint8_t status;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_model_pub_sub_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_model_msg_sensor_data_status {
    float temp;             // 4 bytes
    float humid;            // 4 bytes
    uint16_t light;         // 2 bytes
    uint16_t co2;           // 2 bytes
    uint8_t motion;         // 1 byte
    float dust;             // 4 bytes
    uint8_t battery;        // 1 byte
} ipac_ble_mesh_model_msg_sensor_data_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_sensor_data_status {
    uint8_t opcode;         // 1 byte
    uint16_t unicast;       // 2 bytes
    float temp;             // 4 bytes
    float humid;            // 4 bytes
    uint16_t light;         // 2 bytes
    uint16_t co2;           // 2 bytes
    uint8_t motion;         // 1 byte
    float dust;             // 4 bytes
    uint8_t battery;        // 1 byte
    uint8_t checksum;       // 1 byte
} ipac_ble_mesh_msg_send_sensor_data_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_model_msg_device_info_status {
    uint8_t device_name[DEVICE_NAME_MAX_SIZE];
    uint8_t function;
    int8_t tx_power;
} ipac_ble_mesh_model_msg_device_info_status_t;

typedef struct __attribute__((packed)) ipac_ble_mesh_msg_send_device_info_status {
    uint8_t opcode;
    uint16_t unicast;
    uint8_t device_name[DEVICE_NAME_MAX_SIZE];
    uint8_t function;
    int8_t tx_power;
    uint8_t checksum;
} ipac_ble_mesh_msg_send_device_info_status_t;

// size of arguments in message in command
#define MSG_ARG_NONE                            0
#define MSG_ARG_SIZE_GET_LOCAL_KEYS             MSG_ARG_NONE
#define MSG_ARG_SIZE_UPDATE_LOCAL_KEYS          sizeof(ipac_ble_mesh_msg_recv_update_local_keys_t)
#define MSG_ARG_SIZE_SCAN_UNPROV_DEV            MSG_ARG_NONE
#define MSG_ARG_SIZE_STOP_SCAN                  MSG_ARG_NONE
#define MSG_ARG_SIZE_ADD_UNPROV_DEV             sizeof(ipac_ble_mesh_msg_recv_add_unprov_dev_t)
#define MSG_ARG_SIZE_DELETE_DEVICE              sizeof(ipac_ble_mesh_msg_recv_delete_device_t)
#define MSG_ARG_SIZE_GET_COMPOSITION_DATA       sizeof(ipac_ble_mesh_msg_recv_get_composition_data_t)
#define MSG_ARG_SIZE_ADD_APP_KEY                sizeof(ipac_ble_mesh_msg_recv_add_app_key_t)
#define MSG_ARG_SIZE_BIND_MODEL_APP             sizeof(ipac_ble_mesh_msg_recv_bind_model_app_t)
#define MSG_ARG_SIZE_SET_MODEL_PUB              sizeof(ipac_ble_mesh_msg_recv_model_pub_sub_set_t)
#define MSG_ARG_SIZE_SET_MODEL_SUB              sizeof(ipac_ble_mesh_msg_recv_model_pub_sub_set_t)
#define MSG_ARG_SIZE_SENSOR_DATA_GET            MSG_ARG_NONE
#define MSG_ARG_SIZE_DEVICE_INFO_GET            MSG_ARG_NONE

#define MSG_SIZE_GET_LOCAL_KEYS                 sizeof(ipac_ble_mesh_msg_send_get_local_keys_t)
#define MSG_SIZE_UPDATE_LOCAL_KEYS              sizeof(ipac_ble_mesh_msg_send_get_local_keys_t)
#define MSG_SIZE_SCAN_RESULT                    sizeof(ipac_ble_mesh_msg_scan_result_t)
#define MSG_SIZE_ADD_UNPROV_DEV_ACK             sizeof(ipac_ble_mesh_msg_sent_add_unprov_dev_ack_t)
#define MSG_SIZE_SEND_NEW_NODE_INFO             sizeof(ipac_ble_mesh_msg_send_new_node_info_t)
#define MSG_SIZE_DELETE_DEVICE_STATUS           sizeof(ipac_ble_mesh_msg_send_delete_device_status_t)
#define MSG_SIZE_ADD_APP_KEY_STATUS             sizeof(ipac_ble_mesh_msg_send_add_app_key_status_t)
#define MSG_SIZE_MODEL_APP_STATUS               sizeof(ipac_ble_mesh_msg_send_model_app_status_t)
#define MSG_SIZE_MODEL_PUB_STATUS               sizeof(ipac_ble_mesh_msg_send_model_pub_sub_status_t)
#define MSG_SIZE_MODEL_SUB_STATUS               sizeof(ipac_ble_mesh_msg_send_model_pub_sub_status_t)
#define MSG_SIZE_SENSOR_DATA_STATUS             sizeof(ipac_ble_mesh_msg_send_sensor_data_status_t)
#define MSG_SIZE_DEVICE_INFO_STATUS             sizeof(ipac_ble_mesh_msg_send_device_info_status_t)

#define MSG_ARG_SIZE_TEST_SIMPLE_MSG            MSG_ARG_NONE