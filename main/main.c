#include <stdio.h>
#include <main.h>
#include <ds3231.h>
#include <string.h>
#include <time.h>

// #include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "esp_ble_mesh_defs.h"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

// static const char *TAG = "example";

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_WHITE_LED     ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x05, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x06, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_WHITE_LED      ESP_BLE_MESH_MODEL_OP_3(0x07, CID_ESP)

#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x10, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_RED_LED     ESP_BLE_MESH_MODEL_OP_3(0x12, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x13, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x14, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x15, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x16, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_RED_LED      ESP_BLE_MESH_MODEL_OP_3(0x17, CID_ESP)

#define ESP_BLE_MESH_VND_MODEL_OP_AUTO_MODE      ESP_BLE_MESH_MODEL_OP_3(0xF0, CID_ESP)

#define ESP_BLE_MESH_VND_MODEL_OP_CUR_HOUR     ESP_BLE_MESH_MODEL_OP_3(0xA0, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_CUR_MINUTE      ESP_BLE_MESH_MODEL_OP_3(0xA1, CID_ESP)

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_WHITE_LED          (25) // Define the output GPIO
#define LEDC_OUTPUT_IO_RED_LED          (26) // Define the output GPIO
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define CID_ESP 0x02E5

#define LED_R 1
#define LED_G 2

#define LED_ON 1
#define LED_OFF 0

void board_init(void);

static uint8_t dev_uuid[16] = {0xdd, 0xdd};

int array_time_white_led[1440] = {0};
int array_time_red_led[1440] = {0};

uint8_t hourStartWhiteLed = 0;
uint8_t minuteStartWhiteLed = 0;
uint8_t hourWorkingWhiteLed = 0;
uint8_t minuteWorkingWhiteLed = 0;
uint8_t hourSunsetWhiteLed = 0;
uint8_t minuteSunsetWhiteLed = 0;
uint8_t maxLightningWhiteLed = 100;

uint8_t hourStartRedLed = 0;
uint8_t minuteStartRedLed = 0;
uint8_t hourWorkingRedLed = 0;
uint8_t minuteWorkingRedLed = 0;
uint8_t hourSunsetRedLed = 0;
uint8_t minuteSunsetRedLed = 0;
uint8_t maxLightningRedLed = 100;

uint8_t auto_mode = 0;

uint16_t startInMin = 0;
uint16_t workingInMin = 0;
uint16_t sunsetInMin = 0;

int curHour = 0;
int curMinute = 0;


i2c_dev_t dev;

struct _led_state
{
    uint8_t current;
    uint8_t pin;
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_actions = ESP_BLE_MESH_PUSH,
    .input_size = 4,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_1, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_1 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 2 + 3, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_0, &level_server_0),
    // ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};

static esp_ble_mesh_model_t root_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_1, &level_server_1),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_WHITE_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_WHITE_LED, 2),

    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_RED_LED, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_RED_LED, 2),

    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_AUTO_MODE, 2),

    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_CUR_HOUR, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_CUR_MINUTE, 2),

    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
};

// static esp_ble_mesh_model_op_t vnd_op2[] = {
//     ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND2, 2),
//     ESP_BLE_MESH_MODEL_OP_END,
// };

// static esp_ble_mesh_model_t vnd_models2[] = {
//     ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
//     vnd_op2, NULL, NULL),
// };

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, root_model_1, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, ESP_BLE_MESH_MODEL_NONE, vnd_models),
    // ESP_BLE_MESH_ELEMENT(0, ESP_BLE_MESH_MODEL_NONE, vnd_models2),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static nvs_handle_t NVS_HANDLE;

struct _led_state led_state[2] = {
    {LED_OFF, LED_R},
    {LED_OFF, LED_G},
};

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min);
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_white_led = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_WHITE_LED,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_white_led));

    ledc_channel_config_t ledc_channel_red_led = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_RED_LED,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red_led));
}

void board_init()
{
    gpio_reset_pin(LED_R);
    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_R, LED_OFF);
    // led_state[0].previous = LED_OFF;
}

void board_led_operation(uint8_t pin, uint16_t level_maped)
{   if(auto_mode == 0){
        if(pin == 1){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, level_maped));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
        }
        if(pin == 2){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, level_maped));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
        }
    }
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    // board_led_operation(LED_R, LED_OFF);
}



static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint16_t level_maped)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    struct _led_state *led = NULL;
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                led = &led_state[i];
                board_led_operation(led->pin, level_maped);
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            led = &led_state[model->element->element_addr - primary_addr];
            board_led_operation(led->pin, level_maped);
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        led = &led_state[model->element->element_addr - primary_addr];
        board_led_operation(led->pin, level_maped);
    }
}

static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false) {
            srv->state.onoff = set->onoff;
        } else {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);
        // example_change_led_state(model, ctx, srv->state.onoff);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                //  param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                //  param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        printf("ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    // case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
    //     break;
    // case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
    //     break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            // ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            // ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
            //          param->value.state_change.appkey_add.net_idx,
            //          param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            // ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            // ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
            //          param->value.state_change.mod_app_bind.element_addr,
            //          param->value.state_change.mod_app_bind.app_idx,
            //          param->value.state_change.mod_app_bind.company_id,
            //          param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            // ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            // ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
            //          param->value.state_change.mod_sub_add.element_addr,
            //          param->value.state_change.mod_sub_add.sub_addr,
            //          param->value.state_change.mod_sub_add.company_id,
            //          param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv;

    switch (event)
    {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK){
            int level_maped = 0;
            long level = param->value.state_change.level_set.level;
            level_maped = map(level, -32767, 32767, 0, 8191);
            example_change_led_state(param->model, &param->ctx, level_maped);
            // ESP_LOGW(TAG, "Level set to %d", level_maped);

        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET)
        {
            srv = param->model->user_data;
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK)
        {
            if (param->value.set.onoff.op_en)
            {

            }
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        }
        break;
    default:
        // ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_WHITE_LED) {
            int tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourStartWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteStartWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourWorkingWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteWorkingWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourSunsetWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteSunsetWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_WHITE_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            maxLightningWhiteLed = tid;
            modifyTime(&array_time_white_led, hourStartWhiteLed, minuteStartWhiteLed, hourWorkingWhiteLed, minuteWorkingWhiteLed, hourSunsetWhiteLed, minuteSunsetWhiteLed, maxLightningWhiteLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }


        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_START_RED_LED) {
            int tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourStartRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_START_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteStartRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_WORKING_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourWorkingRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_WORKING_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteWorkingRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HOUR_SUNSET_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            hourSunsetRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MINUTE_SUNSET_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            minuteSunsetRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MAX_LIGHTNING_RED_LED) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            maxLightningRedLed = tid;
            modifyTime(&array_time_red_led, hourStartRedLed, minuteStartRedLed, hourWorkingRedLed, minuteWorkingRedLed, hourSunsetRedLed, minuteSunsetRedLed, maxLightningRedLed);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_AUTO_MODE) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            auto_mode = tid;

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_CUR_HOUR) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            curHour = tid;
            initialize_ds3231();

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_CUR_MINUTE) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x, val is %d", param->model_operation.opcode, tid, tid);
            curMinute = tid;
            initialize_ds3231();

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(tid), (uint8_t *)&tid);
            if (err) {
                // ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            // ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        // ESP_LOGI(TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    // ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return err;
}

void app_main(void)
{
    esp_err_t err;

    printf("Initializing...");

    board_init();

    example_ledc_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    memset(&dev, 0, sizeof(i2c_dev_t));

    err = bluetooth_init();
    if (err)
    {
        // ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Open nvs namespace for storing/restoring mesh example info */
    err = ble_mesh_nvs_open(&NVS_HANDLE);
    if (err)
    {
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err)
    {
        // ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    gpio_set_level(LED_R, LED_OFF);

    prepareTime(&array_time_white_led);
    prepareTime(&array_time_red_led);

    ESP_ERROR_CHECK(i2cdev_init());

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, 21, 22));

    initialize_ds3231();

    xTaskCreate(timerLight, "timer_light", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

}


void initialize_ds3231(){

    struct tm time = {
        .tm_year = 123, // (2022 - 1900)
        .tm_mon  = 4,  // 0-based
        .tm_mday = 26,
        .tm_hour = curHour,
        .tm_min  = curMinute,
        .tm_sec  = 1
    };

    ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));

}


void modifyTime(int* array_timeTemp, uint8_t hourStart, uint8_t minuteStart, uint8_t hourWorking, uint8_t minuteWorking, uint8_t hourSunset, uint8_t minuteSunset, uint8_t maxLingtning){

    uint16_t maxLingtningLevel = map(maxLingtning, 0, 100, 0, 8191);

    for(int i = 0; i < 1440; i++){
        array_timeTemp[i] = 0;
    }

    startInMin = hourStart * 60 + minuteStart;
    workingInMin = hourWorking * 60 + minuteWorking;
    sunsetInMin = hourSunset * 60 + minuteSunset;
    int kExpander = 0;
    if(sunsetInMin != 0){
        kExpander = maxLingtningLevel / sunsetInMin;
    }
    int counterSunset = 0;

    for(uint16_t i = startInMin; i < startInMin + workingInMin; i++){
        if(i < startInMin + sunsetInMin){
            array_timeTemp[i % 1440] = counterSunset;
            counterSunset = counterSunset + kExpander;
        }
        else if(i > startInMin + workingInMin - sunsetInMin){
            array_timeTemp[i % 1440] = counterSunset;
            counterSunset = counterSunset - kExpander;
        }
        else{
            array_timeTemp[i % 1440] = maxLingtningLevel;
            counterSunset = maxLingtningLevel;
        }
    }
}

void prepareTime(int* array_timeTemp){
    for(int i = 0; i < 1440; i++){
        array_timeTemp[i] = 0;
    }
}

void timerLight(void *pvParameters){
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(auto_mode){

            struct tm time1 = {
                .tm_year = 1, // (2022 - 1900)
                .tm_mon  = 1,  // 0-based
                .tm_mday = 1,
                .tm_hour = 1,
                .tm_min  = 1,
                .tm_sec  = 1
            };

            ESP_ERROR_CHECK(ds3231_get_time(&dev, &time1));

            uint16_t curMinutes = time1.tm_hour * 60 + time1.tm_min;

            pwm_set_white_led(array_time_white_led[curMinutes]);

            pwm_set_red_led(array_time_red_led[curMinutes]);
        }

    }

}

void pwm_set_white_led(int duty){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
}

void pwm_set_red_led(int duty){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
}
