/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

#include "bt_app_core_utils.h"
#include "bredr_app_common_utils.h"
#include "a2dp_sink_common_utils.h"
#include "a2dp_utils_tags.h"
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
#include "a2dp_sink_int_codec_utils.h"
#else
#include "a2dp_sink_ext_codec_utils.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sbc.h"
#include "sbc_private.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/ringbuf.h"


#include "driver/i2s_std.h"


/* device name */
static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

#define GPIO_PIN 2

// sbc_t sbc;
// uint8_t s_pckt_cnt = 0;
// ssize_t framelen;

static uint8_t pcm[4096];                    /* off the BTC_TASK stack */  
static uint8_t pcm_copy[4096];                    /* off the BTC_TASK stack */  

static sbc_t sbc;                            /* decoder context persists */  
static bool sbc_inited;  
static uint32_t s_pckt_cnt; 

uint8_t buffer_cnt = 1;

i2s_chan_handle_t tx_handle;
i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);



QueueHandle_t work_queue;


/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/********************************
 * STATIC FUNCTION DECLARATIONS
 *******************************/

/* Device callback function */
static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/* callback function for A2DP sink */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
/* callback function for A2DP sink audio data stream */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
#else
/* callback function for A2DP sink undecoded audio data */
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);
#endif

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

uint32_t checksum(uint8_t *buffer, size_t length)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < length; i++)
    {
        sum += buffer[i];
    }

    return sum;
}
void worker_task(void *arg)
{
    uint8_t worker_pcm[4096];

    while(1)
    {
        if (xQueueReceive(work_queue, worker_pcm, portMAX_DELAY))
        {
            //int64_t t0 = esp_timer_get_time();
            
            // Perform your processing here
            //ESP_LOGI()
            //10 - 18, mainly around 14 us, to complete i2s channel write
            //ESP_LOGI("TIMING", "PCM Value before sending to write: %02X %02X %02X %02X %02X %02X",pcm_copy[0], pcm_copy[1], pcm_copy[2], pcm_copy[3], pcm_copy[10], pcm_copy[30]);
            //ESP_LOG_BUFFER_HEX("I2S Write", pcm_copy, 16);
            //uint32_t sum = checksum(pcm_copy, 4096);
            //ESP_LOGI("CALLBACK", "I2S checksum: %" PRIu32, sum);
            esp_err_t ret = i2s_channel_write(tx_handle, worker_pcm, sizeof(worker_pcm), NULL, 1000);
            //ESP_LOGI("I2S Write", "Operation result: %d (%s)", ret, esp_err_to_name(ret));
            //int64_t t1 = esp_timer_get_time();
            //ESP_LOGI("TIMING", "Callback execution: %" PRId64 " us", t1-t0);
        }
    }
}

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

void i2s_configure()
{
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 1024;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));
    i2s_std_config_t std_tx_cfg = 
    {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_26,
            .ws = GPIO_NUM_27,
            .dout = GPIO_NUM_25,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    // i2s_chan_config_t chan_cfg =
    // {
    //     .id = I2S_NUM_0, 
    //     .role = I2S_ROLE_MASTER,
    //     .dma_desc_num = 8,
    //     .dma_frame_num = 120
    // };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    bredr_app_dev_evt_def_hdl(event, param);
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    bredr_app_gap_evt_def_hdl(event, param);
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:

    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_app_work_dispatch(bt_a2d_evt_def_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);
        break;
    }
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            gpio_set_level(GPIO_PIN, 1);

        }
        else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
        {
            gpio_set_level(GPIO_PIN, 0);
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(BT_AV_TAG, "Config Event");
        break;
    case ESP_A2D_SEP_REG_STATE_EVT: {
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        bt_app_work_dispatch(bt_a2d_evt_int_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);
#else
        bt_app_work_dispatch(bt_a2d_evt_ext_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);

#endif
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    bt_a2d_data_hdl(data, len);
} 
#else
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl,  
                                     esp_a2d_audio_buff_t *audio_buf)  
{  
    //13 - 16 milliseconds to finsih callback, 13000 - 16000 us
    //uint64_t start = esp_timer_get_time();

    //Callback frequency, 10-30 ms, time since prvious callback, mainly around 20000 us (20 ms)
    // static int64_t last_time = 0;

    // int64_t now = esp_timer_get_time();

    // if (last_time != 0)
    // {
    //     ESP_LOGI("TIMING", "Time since previous callback: %" PRId64 " us",
    //              now - last_time);
    // }

    // last_time = now;
    
    
    //int64_t t0 = esp_timer_get_time();

    bt_a2d_audio_data_hdl(conn_hdl, audio_buf);  
  
    if (!sbc_inited) {  
        sbc_init(&sbc, 0L);  
        sbc.endian = SBC_LE;                 /* match your audio pipeline */  
        sbc_inited = true;  
    }  
  
    size_t packet_len = audio_buf->data_len;  
    uint16_t pos = 0, pcm_pos = 0;  
    ssize_t framelen = 0; 
    size_t written = 0;  
    
    //119 SBC bytes in a frame
    //ESP_LOG_BUFFER_HEX("RAW SBC Bytes At Top of Callback", audio_buf->data, 32);

    //int64_t t0 = esp_timer_get_time();
    //ESP_LOGI("BYTES Gen:", "%d", audio_buf->data_len);

    while (pos < packet_len) 
    {
        written = 0;  
        framelen = sbc_decode(&sbc,  
                              audio_buf->data + pos, packet_len - pos,  
                              pcm + pcm_pos, sizeof(pcm) - pcm_pos,  
                              &written);  
        if (framelen <= 0)  
            break;  
        
        pos     += framelen;  
        pcm_pos += written;
    }     

    memcpy(pcm_copy, pcm, sizeof(pcm));
    //ESP_LOG_BUFFER_HEX("RAW SBC Bytes At End of Callback", audio_buf->data, 32);
    //i2s_channel_write(tx_handle, pcm_copy, sizeof(4096), NULL, 1000);
    //esp_err_t ret = i2s_channel_write(tx_handle, pcm_copy, sizeof(pcm_copy), NULL, 1000);

    //int64_t t1 = esp_timer_get_time();
    //ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);

    //ESP_LOG_BUFFER_HEX("PCM_CALLBACK", pcm_copy, 16);
    //uint32_t sum = checksum(pcm_copy, 4096);
    //ESP_LOGI("CALLBACK", "PCM checksum: %" PRIu32, sum);
    //ESP_LOGI("TIMING", "PCM Value in callback: %02X %02X %02X %02X %02X %02X",pcm_copy[0], pcm_copy[1], pcm_copy[2], pcm_copy[3], pcm_copy[10], pcm_copy[30]);
    //memmove(pcm_copy+4096, pcm_copy, sizeof(pcm));  

    
    //int64_t t0 = esp_timer_get_time();
    BaseType_t result = xQueueSend(work_queue, pcm_copy, 0);
    
    //int64_t t1 = esp_timer_get_time();
    //ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);
    //ESP_LOGI("RESULT: ", "%d", result);
    if (++s_pckt_cnt % 100 == 0)  
    {
    ESP_LOGI(BT_AV_TAG,  
                 "pos=%u" " pcm=%u" " last_frame len:=%d" " Bytes of PCM Out:=%d" " in=%02X %02X %02X %02X %02X %02X %02X %02X" " out=%02X %02X %02X %02X %02X %02X %02X %02X" " %d",  
                 pos, pcm_pos, (int)framelen, sizeof(pcm),
                 audio_buf->data[0], audio_buf->data[1],  
                 audio_buf->data[2], audio_buf->data[3],
                audio_buf->data[4], audio_buf->data[5],  
                audio_buf->data[6], audio_buf->data[7], 
                pcm[0], pcm[1], pcm[2], pcm[3],
                pcm[4], pcm[5], pcm[6], pcm[7], 
                sbc.frequency);  
    }
    esp_a2d_audio_buff_free(audio_buf); 
    //int64_t end = esp_timer_get_time();
    // ESP_LOGI("TIMING", "Callback execution: %" PRId64 " us",
    //          end - start);
    // int64_t t1 = esp_timer_get_time();
    // ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);
}
#endif

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP: {
        esp_bt_gap_set_device_name(local_device_name);
        esp_bt_dev_register_callback(bt_app_dev_cb);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        esp_a2d_register_callback(&bt_app_a2d_cb);
        assert(esp_a2d_sink_init() == ESP_OK);
        

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
#else
        esp_a2d_mcc_t mcc = {0};
        mcc.type = ESP_A2D_MCT_SBC;
        mcc.cie.sbc_info.samp_freq = ESP_A2D_SBC_CIE_SF_16K |
                                     ESP_A2D_SBC_CIE_SF_32K |
                                     ESP_A2D_SBC_CIE_SF_44K |
                                     ESP_A2D_SBC_CIE_SF_48K;
        mcc.cie.sbc_info.ch_mode = ESP_A2D_SBC_CIE_CH_MODE_MONO |
                                   ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL |
                                   ESP_A2D_SBC_CIE_CH_MODE_STEREO |
                                   ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
        mcc.cie.sbc_info.block_len = ESP_A2D_SBC_CIE_BLOCK_LEN_4 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_8 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_12 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_16;
        mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_4 | ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
        mcc.cie.sbc_info.alloc_mthd = ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR | ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
        mcc.cie.sbc_info.max_bitpool = 250;
        mcc.cie.sbc_info.min_bitpool = 2;
        /* register stream end point, only support SBC currently */
        esp_a2d_sink_register_stream_endpoint(0, &mcc);
        esp_a2d_sink_register_audio_data_callback(bt_app_a2d_audio_data_cb);
       // mcc.cie.sbc_info.samp_freq = ESP_A2D_SBC_CIE_SF_48K;

        ESP_LOGI(BT_AV_TAG, "Codec Type %d" " Sampling Freq. %X", mcc.type, mcc.cie.sbc_info.samp_freq);
#endif

        /* Get the default value of the delay value */
        esp_a2d_sink_get_delay_value();
        /* Get local device name */
        esp_bt_gap_get_device_name();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}



/*******************************
 * MAIN ENTRY POINT
 ******************************/

void app_main(void)
{
    gpio_set_direction(GPIO_PIN, GPIO_MODE_OUTPUT);
    
    sbc.endian = SBC_LE;

    uint8_t config[4] = {0};
    //config: [(0x4)(0x)]
    config[0] = ESP_A2D_SBC_CIE_SF_48K | ESP_A2D_SBC_CIE_CH_MODE_MONO;     //freq = 48k, channel mode = mono
    config[1] = ESP_A2D_SBC_CIE_BLOCK_LEN_16 | ESP_A2D_SBC_CIE_NUM_SUBBANDS_4 | ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR;
    config[2] = 250;
    config[3] = 2;
    uint8_t conf_len = sizeof(config);
    i2s_configure();
    int ret = sbc_init(&sbc, 0L);
    sbc.frequency = 0x00;
    ESP_LOGI(BT_AV_TAG, "SBC INIT RET VALUE:" "%d", ret);
    ESP_ERROR_CHECK(bredr_app_common_init());

    work_queue = xQueueCreate(3, sizeof(pcm_copy));

    configASSERT(work_queue != NULL);

    xTaskCreate(
        worker_task,
        "worker_task",
        12288,
        NULL,
        5,
        NULL
    );

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL, NULL);
}
