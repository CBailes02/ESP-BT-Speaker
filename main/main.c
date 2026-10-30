// /*
//  * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDX-License-Identifier: Unlicense OR CC0-1.0
//  */

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

#include "led_strip.h"
#include "dl_fft.h"
#include "dl_rfft.h"
#include "esp_dsp.h"
#include "math.h"
#include "esp_random.h"

/* device name */
static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;


// sbc_t sbc;
// uint8_t s_pckt_cnt = 0;
// ssize_t framelen;

static uint8_t pcm[4096];                    /* off the BTC_TASK stack */  
static uint8_t pcm_copy[4096];                    /* off the BTC_TASK stack */  
// static uint8_t pcm_pack[4096];
//uint8_t sbc_pack[952] = {0};
static sbc_t sbc;                            /* decoder context persists */  
static bool sbc_inited;  
static uint32_t s_pckt_cnt; 

uint8_t buffer_cnt = 1;

i2s_chan_handle_t tx_handle;
i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);



//QueueHandle_t decoder_queue;

RingbufHandle_t buf_handle;

TaskHandle_t fft_led_task_handle = NULL;
typedef struct {
    led_strip_handle_t led_strip;
    uint8_t *pcm_copy_for_fft;
    size_t pcm_length;
} led_task_params_t;

led_task_params_t fft_led_struct;

#define LED_STRIP_USE_DMA  0

#if LED_STRIP_USE_DMA
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 256
#define LED_STRIP_MEMORY_BLOCK_WORDS 1024 // this determines the DMA block size
#else
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 40
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically
#endif // LED_STRIP_USE_DMA

// GPIO assignment
#define LED_STRIP_GPIO_PIN  4
#define GPIO_PIN 2

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)
static const char *NEO_TAG = "NEOPIX EX";
static const char *FFT_TAG = "FFT";


#define N_SAMPLES 1024
int N = N_SAMPLES;




// Window coefficients

float wind[N_SAMPLES];
__attribute__((aligned(16)))
// working complex array

float audio_sig_input[N_SAMPLES];
__attribute__((aligned(16)))

float audio_sig_cf[N_SAMPLES*2];
__attribute__((aligned(16)))
float magnitude[N_SAMPLES];
__attribute__((aligned(16)))
// Pointers to result arrays
// float *y1_cf = &y_cf[0];
// float *y2_cf = &y_cf[N_SAMPLES];

// static float column0[256] = {0};
// static float column1[256] = {0};
// static float column2[256] = {0};
// static float column3[256] = {0};
// static float column4[256] = {0};
// static float column5[256] = {0};
// static float column6[256] = {0};
// static float column7[256] = {0};

SemaphoreHandle_t pcm_mutex;


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



led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };


    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(NEO_TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

// void worker_task(void *arg)
// {
//     uint8_t worker_pcm[4096];

//     while(1)
//     {
//         if (xQueueReceive(work_queue, worker_pcm, portMAX_DELAY))
//         {
//             //int64_t t0 = esp_timer_get_time();
            
//             // Perform your processing here
//             //ESP_LOGI()
//             //10 - 18, mainly around 14 us, to complete i2s channel write
//             //ESP_LOGI("TIMING", "PCM Value before sending to write: %02X %02X %02X %02X %02X %02X",pcm_copy[0], pcm_copy[1], pcm_copy[2], pcm_copy[3], pcm_copy[10], pcm_copy[30]);
//             //ESP_LOG_BUFFER_HEX("I2S Write", pcm_copy, 16);
//             //uint32_t sum = checksum(pcm_copy, 4096);
//             //ESP_LOGI("CALLBACK", "I2S checksum: %" PRIu32, sum);
//             esp_err_t ret = i2s_channel_write(tx_handle, worker_pcm, sizeof(worker_pcm), NULL, 1000);
//             //ESP_LOGI("I2S Write", "Operation result: %d (%s)", ret, esp_err_to_name(ret));
//             //int64_t t1 = esp_timer_get_time();
//             //ESP_LOGI("TIMING", "Callback execution: %" PRId64 " us", t1-t0);
//         }
//     }
// }

// void buffer_task(void *arg)
// {
//     uint8_t worker_pcm[4096];

//     while(1)
//     {
//         if (xQueueReceive(work_queue, worker_pcm, portMAX_DELAY))
//         {
//             //int64_t t0 = esp_timer_get_time();
            
//             // Perform your processing here
//             //ESP_LOGI()
//             //10 - 18, mainly around 14 us, to complete i2s channel write
//             //ESP_LOGI("TIMING", "PCM Value before sending to write: %02X %02X %02X %02X %02X %02X",pcm_copy[0], pcm_copy[1], pcm_copy[2], pcm_copy[3], pcm_copy[10], pcm_copy[30]);
//             //ESP_LOG_BUFFER_HEX("I2S Write", pcm_copy, 16);
//             //uint32_t sum = checksum(pcm_copy, 4096);
//             //ESP_LOGI("CALLBACK", "I2S checksum: %" PRIu32, sum);
//             esp_err_t ret = i2s_channel_write(tx_handle, worker_pcm, sizeof(worker_pcm), NULL, 1000);
//             //ESP_LOGI("I2S Write", "Operation result: %d (%s)", ret, esp_err_to_name(ret));
//             //int64_t t1 = esp_timer_get_time();
//             //ESP_LOGI("TIMING", "Callback execution: %" PRId64 " us", t1-t0);
//         }
//     }
// }

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

// void write_fft_leds(uint8_t pcm_input_array[], led_strip_handle_t strip)
// {
//     //ESP_LOGE("Write LED function: ", "Entered Write Leds.");
//     //ESP_LOGI("Write LED Functions", "%d %d %d %d", pcm_input_array[0], pcm_input_array[1], pcm_input_array[2], pcm_input_array[3]);
    
//     int16_t sample_16_b;
//     float sample_32_b;
//     float column_0[128];
//     float column_1[128];
//     float column_2[128];
//     float column_3[128];
//     float column_4[128];
//     float column_5[128];
//     float column_6[128];
//     float column_7[128];
    
//     float col_0_sum = 0;
//     float col_1_sum = 0;
//     float col_2_sum = 0;
//     float col_3_sum = 0;
//     float col_4_sum = 0;
//     float col_5_sum = 0;
//     float col_6_sum = 0;
//     float col_7_sum = 0;

//     float col_0_avg = 0;
//     float col_1_avg = 0;
//     float col_2_avg = 0;
//     float col_3_avg = 0;
//     float col_4_avg = 0;
//     float col_5_avg = 0;
//     float col_6_avg = 0;
//     float col_7_avg = 0;


//     //i*4 for left channel only, skips 2 and 3 offset indices which are the right channel
//     for (int i = 0; i < 1024; i++)
//     {
//         sample_16_b = (int16_t)((uint16_t)(pcm_input_array[i*4]) | (uint16_t)(pcm_input_array[(i*4)+1] << 8));
//         sample_32_b = (float)sample_16_b/32768.0f;
//         audio_sig_input[i] = sample_32_b;
//     }
//     //ESP_LOGI("Write LED Functions: ", "Sample 16 bits %d", sample_16_b);
//     //ESP_LOGI("Write LED Functions: ", "Sample 32 bits %f", sample_32_b);

//     //ESP_LOGI("Write LED Functions", "Audio Input: %f %f %f %f", audio_sig_input[0], audio_sig_input[1], audio_sig_input[2], audio_sig_input[3]);    


//     //audio_sig_cf constain N_SAMPLES*2 bc of the complex element
//     for (int i = 0; i < N_SAMPLES; i++)
//     {
//         audio_sig_cf[i*2] = audio_sig_input[i];
//         audio_sig_cf[(i*2) + 1] = 0.0f;
//     }


// //     ESP_LOGI(TAG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
// //     y_cf[0], y_cf[1], y_cf[2], y_cf[3], y_cf[4],
// //     y_cf[5], y_cf[6], y_cf[7], y_cf[8], y_cf[9],
// //     y_cf[10], y_cf[11], y_cf[12], y_cf[13], y_cf[14],
// //     y_cf[15], y_cf[16], y_cf[17], y_cf[18], y_cf[19]);

//     ESP_ERROR_CHECK(dsps_fft2r_fc32(audio_sig_cf, N_SAMPLES));
//     ESP_ERROR_CHECK(dsps_bit_rev_fc32(audio_sig_cf, N_SAMPLES));


// //     //this is the final real output of fft from x1

//     for (int i = 0; i < N_SAMPLES; i++)
//     {
//         magnitude[i] = (sqrtf((audio_sig_cf[i*2]*audio_sig_cf[i*2]) + (audio_sig_cf[((i*2)+1)]*audio_sig_cf[((i*2)+1)]))/N_SAMPLES);
//     }

//     //ESP_LOGI("Write LED Functions", "FFT Magn. Values: %f %f %f %f %f", magnitude[0], magnitude[1], magnitude[2], magnitude[3], magnitude[4]);

//     for (int i = 0; i < 1024; i++)
//     {
//         if (i < 128)
//         {
//             col_0_sum += magnitude[i];
//         }
//         else if (i >= 128 || i < 255)
//         {
//             col_1_sum += magnitude[i];
//         }
//         else if (i >= 255 || i < 384)
//         {
//             col_2_sum += magnitude[i];
//         }        
//         else if (i >= 384 || i < 512)
//         {
//             col_3_sum += magnitude[i];
//         }
//         else if (i >= 512 || i < 640)
//         {
//             col_4_sum += magnitude[i];
//         }
//         else if (i >= 640 || i < 768)
//         {
//             col_5_sum += magnitude[i];
//         }
//         else if (i >= 768 || i < 896)
//         {
//             col_6_sum += magnitude[i];
//         }
//         else 
//         {
//             col_7_sum += magnitude[i];
//         }
//     }

//     // col_0_avg = col_0_sum/(128.0f);
//     // col_1_avg = col_1_sum/(128.0f);
//     // col_2_avg = col_2_sum/(128.0f);
//     // col_3_avg = col_3_sum/(128.0f);
//     // col_4_avg = col_4_sum/(128.0f);
//     // col_5_avg = col_5_sum/(128.0f);
//     // col_6_avg = col_6_sum/(128.0f);
//     // col_7_avg = col_7_sum/(128.0f);
//     ESP_LOGI("Write LED Functions", "Sum Vals: %f %f %f %f %f", col_0_sum, col_1_sum, col_2_sum, col_3_sum, col_4_sum);

//     uint8_t neo_idx_col_0 = LED_STRIP_LED_COUNT;
//     uint8_t idx_cntr_0 = 0;
//     float val_cntr = 0.0f;

//     for (float i = 0.0f; i <= col_0_sum; i+=0.2f)
//     {   
//         if (neo_idx_col_0 > 8)
//         {
//             neo_idx_col_0 -= 8;
//         }
//         else 
//         {
//             neo_idx_col_0 = 0;
//         }
//         ESP_ERROR_CHECK(led_strip_set_pixel_hsv(strip, neo_idx_col_0, 0, 255, 100));
//         ESP_ERROR_CHECK(led_strip_refresh_async(strip));
//         ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));
//         vTaskDelay(pdMS_TO_TICKS(10));
//         //vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     //vTaskDelay(pdMS_TO_TICKS(10));
//     ESP_ERROR_CHECK(led_strip_clear(strip));
//     // while(val_cntr < col_0_sum)
//     // {
//     //     ESP_ERROR_CHECK(led_strip_set_pixel_hsv(strip, neo_idx_col_0, 0, 255, 100));
//     //     ESP_ERROR_CHECK(led_strip_refresh_async(strip));
//     //     ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));
//     //     vTaskDelay(pdMS_TO_TICKS(10));


//     //     idx_cntr_0 += 1;
//     //     if (neo_idx_col_0 > 8)
//     //     {
//     //         neo_idx_col_0 -= 8;
//     //     }
//     //     else 
//     //     {
//     //         neo_idx_col_0 = 0;
//     //     }
//     //     val_cntr += 0.10f;
//     // }
// }


void decoder_task(void *pvParameters)
{
    size_t packet_len; 
    // uint8_t pcm[4096];                    /* off the BTC_TASK stack */  
    // uint8_t pcm_copy[4096];                    /* off the BTC_TASK stack */  
    
    ssize_t framelen = 0; 
    size_t written = 0;
    //ESP_LOGE("DECODER TASK: ", "Entered Decoder Task");

    //led_strip_handle_t strip = (led_strip_handle_t)pvParameters;
    uint8_t pckt_rec_cnt = 0;
    while(1)
    {
        //uint64_t start = esp_timer_get_time();
        //ESP_LOGI("DECODER TASK", "Running on Core %d", xPortGetCoreID());
        
        uint8_t *sbc_pack = xRingbufferReceive(buf_handle, &packet_len, portMAX_DELAY);
        //ESP_LOGI("DECODER TASK: ", "%X %X %X %X", sbc_pack[0], sbc_pack[10], sbc_pack[11], sbc_pack[12]);
        //ESP_LOGI("DECODER TASK: ", "Received: %d", (int)packet_len);

        //Check received item
        if (sbc_pack != NULL) {
            //Print item
        //ESP_LOGI("DECODER TASK: ", "%X %X %X", sbc_pack[0], sbc_pack[119], sbc_pack[238]);
        uint16_t pos = 0, pcm_pos = 0;  

        while (pos < packet_len) 
        {
            written = 0;  
            framelen = sbc_decode(&sbc,  
                                sbc_pack + pos, packet_len - pos,  
                                pcm + pcm_pos, sizeof(pcm) - pcm_pos,  
                                &written);  
            if (framelen <= 0)  
                break;  
            
            pos     += framelen;  
            pcm_pos += written;
        }    
        //ESP_LOGI("DECODER TASK: ", " %X %X", pcm[4], pcm[5]);  
        //ESP_LOGI("DECODER TASK: ", " Bytes In %d  Bytes Out %d", framelen, written);  

            //Return Item
        vRingbufferReturnItem(buf_handle, sbc_pack);

        //xSemaphoreTake(pcm_mutex, portMAX_DELAY);
        memcpy(pcm_copy, pcm, sizeof(pcm));
        //xSemaphoreGive(pcm_mutex);

        esp_err_t ret = i2s_channel_write(tx_handle, pcm_copy, sizeof(pcm_copy), NULL, 1000);
        } else {
            //Failed to receive item
            ESP_LOGE("DECODER TASK: ", "Failed to create ring buffer");
        }

        //write_fft_leds(pcm_copy, strip);

        if (pckt_rec_cnt == 4)
        {
        xTaskNotifyGive(fft_led_task_handle);
        pckt_rec_cnt = 0;
        }
    
        //int64_t now = esp_timer_get_time();
        //ESP_LOGI("Decoder Task", "Time of decoder task loop: %" PRId64 " us", now - start);
        //pass pcm copy into fft generator
        //call write led function to send fft to neopixels
        // convert byte array to 16 bit array

        //memcpy(y_cf, pcm, sizeof(pcm));        
        //bit reverse
        
        //ESP_LOGI("DECODER TASK: ", "Result: %s", esp_err_to_name(ret));  
        pckt_rec_cnt += 1;
    }
}



void write_led_task(void *pvParameters)
{
    led_task_params_t* fft_led_struct = (led_task_params_t *)pvParameters;

    //Have to do this to pass it into the write led functions for some reason or else it will crash
    led_strip_handle_t strip = fft_led_struct->led_strip;

    ESP_LOGI("LED Task", "Struct handle: %p", fft_led_struct);
    ESP_LOGI("LED Task", "Strip handle: %p", fft_led_struct->led_strip);
    ESP_LOGI("LED Task", "Strip handle declared variable: %p", strip);
    ESP_LOGI("LED Task", "PCM Copy FFT handle: %p", &fft_led_struct->pcm_copy_for_fft);
    
    fft_led_struct->pcm_copy_for_fft = pcm_copy;
    uint8_t pcm_local[4096];


    //memcpy(pcm_local, pcm_copy, sizeof(pcm_local));
    //pcm_local = fft_led_struct->pcm_copy_for_fft;

    ESP_LOGI("LED Task", "PCM Copy FFT PTR to PCM_COPY handle: %p", pcm_local);


    uint8_t col_7_idx_arr[5] = {0, 8, 16, 24, 32};
    uint8_t col_6_idx_arr[5] = {1, 9, 17, 25, 33};
    uint8_t col_5_idx_arr[5] = {2, 10, 18, 26, 34};
    uint8_t col_4_idx_arr[5] = {3, 11, 19, 27, 35};
    uint8_t col_3_idx_arr[5] = {4, 12, 20, 28, 36};
    uint8_t col_2_idx_arr[5] = {5, 13, 21, 29, 37};
    uint8_t col_1_idx_arr[5] = {6, 14, 22, 30, 38};
    uint8_t col_0_idx_arr[5] = {7, 15, 23, 31, 39};
    
    //uint8_t col_0_idx_arr[5] = {32, 24, 16, 8, 0};

    uint8_t prev_idx_cntr_0 = 0;
    uint8_t diff = 0;
    uint8_t loop_idx = 0;

    uint8_t brightness_full = 200;
    uint8_t brightness_mid = 100;
    uint8_t brightness_low = 50;
    uint8_t brightness_off = 0;

    while(1)
    {
    ESP_LOGI("FFT LED TASK", "Running on Core %d", xPortGetCoreID());
        //uint64_t start = esp_timer_get_time();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //xSemaphoreTake(pcm_mutex, portMAX_DELAY);

    memcpy(pcm_local, fft_led_struct->pcm_copy_for_fft, sizeof(pcm_local));

    //xSemaphoreGive(pcm_mutex);
    //ESP_LOGI("LED Task", "Struct handle: %p", fft_led_struct);
    //ESP_LOGI("LED Task", "Strip handle: %p", fft_led_struct->led_strip);
    //ESP_LOGI("LED Task", "PCM Copy handle: %p", (void *)fft_led_struct->pcm_copy_for_fft);
    int16_t sample_16_b;
    float sample_32_b;
    float column_0[128];
    float column_1[128];
    float column_2[128];
    float column_3[128];
    float column_4[128];
    float column_5[128];
    float column_6[128];
    float column_7[128];
    
    float col_0_sum = 0;
    float col_1_sum = 0;
    float col_2_sum = 0;
    float col_3_sum = 0;
    float col_4_sum = 0;
    float col_5_sum = 0;
    float col_6_sum = 0;
    float col_7_sum = 0;

    float col_0_avg = 0;
    float col_1_avg = 0;
    float col_2_avg = 0;
    float col_3_avg = 0;
    float col_4_avg = 0;
    float col_5_avg = 0;
    float col_6_avg = 0;
    float col_7_avg = 0;

    // ESP_LOGI("Write LED Functions", "Audio PCM Input: %d %d %d %d %d %d %d %d", pcm_fft[0], pcm_fft[1], pcm_fft[2], pcm_fft[3], 
    //     pcm_fft[4], pcm_fft[5], pcm_fft[6], pcm_fft[7]);    

//     //i*4 for left channel only, skips 2 and 3 offset indices which are the right channel
    for (int i = 0; i < 1024; i++)
    {
        sample_16_b = (int16_t)((uint16_t)(pcm_local[i*4]) | (uint16_t)(pcm_local[(i*4)+1] << 8));
        sample_32_b = (float)sample_16_b/32768.0f;
        audio_sig_input[i] = sample_32_b;
    }
//     //ESP_LOGI("Write LED Functions: ", "Sample 16 bits %d", sample_16_b);
//     //ESP_LOGI("Write LED Functions: ", "Sample 32 bits %f", sample_32_b);

    //ESP_LOGI("Write LED Functions", "Audio Input: %f %f %f %f", audio_sig_input[0], audio_sig_input[1], audio_sig_input[2], audio_sig_input[3]);    


//     //audio_sig_cf constain N_SAMPLES*2 bc of the complex element
    for (int i = 0; i < N_SAMPLES; i++)
    {
        audio_sig_cf[i*2] = audio_sig_input[i];
        audio_sig_cf[(i*2) + 1] = 0.0f;
    }


// //     ESP_LOGI(TAG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
// //     y_cf[0], y_cf[1], y_cf[2], y_cf[3], y_cf[4],
// //     y_cf[5], y_cf[6], y_cf[7], y_cf[8], y_cf[9],
// //     y_cf[10], y_cf[11], y_cf[12], y_cf[13], y_cf[14],
// //     y_cf[15], y_cf[16], y_cf[17], y_cf[18], y_cf[19]);

    ESP_ERROR_CHECK(dsps_fft2r_fc32(audio_sig_cf, N_SAMPLES));
    ESP_ERROR_CHECK(dsps_bit_rev_fc32(audio_sig_cf, N_SAMPLES));


// //     //this is the final real output of fft from x1

    for (int i = 0; i < N_SAMPLES; i++)
    {
        magnitude[i] = (sqrtf((audio_sig_cf[i*2]*audio_sig_cf[i*2]) + (audio_sig_cf[((i*2)+1)]*audio_sig_cf[((i*2)+1)]))/N_SAMPLES);
    }

//     //ESP_LOGI("Write LED Functions", "FFT Magn. Values: %f %f %f %f %f", magnitude[0], magnitude[1], magnitude[2], magnitude[3], magnitude[4]);

    for (int i = 0; i < 1024; i++)
    {
        if (i < 8)
        {
            col_0_sum += magnitude[i];
        }
        else if (i >= 8 && i < 16)
        {
            col_1_sum += magnitude[i];
        }
        else if (i >= 16 && i < 24)
        {
            col_2_sum += magnitude[i];
        }        
        else if (i >= 24 && i < 32)
        {
            col_3_sum += magnitude[i];
        }
        else if (i >= 32 && i < 40)
        {
            col_4_sum += magnitude[i];
        }
        else if (i >= 40 && i < 48)
        {
            col_5_sum += magnitude[i];
        }
        else if (i >= 48 && i < 56)
        {
            col_6_sum += magnitude[i];
        }
        else if (i >= 56 && i < 64)
        {
            col_7_sum += magnitude[i];
            break;
        }
    }

    // col_0_avg = col_0_sum/(128.0f);
    // col_1_avg = col_1_sum/(128.0f);
    // col_2_avg = col_2_sum/(128.0f);
    // col_3_avg = col_3_sum/(128.0f);
    // col_4_avg = col_4_sum/(128.0f);
    // col_5_avg = col_5_sum/(128.0f);
    // col_6_avg = col_6_sum/(128.0f);
    // col_7_avg = col_7_sum/(128.0f);
    ESP_LOGI("Write LED Functions", "Sum Vals: %f %f %f %f %f %f %f %f ", col_0_sum, col_1_sum, col_2_sum, col_3_sum, col_4_sum, 
    col_5_sum, col_6_sum, col_7_sum);

    //ESP_LOGI("Write LED Functions", "Avg Vals: %f %f %f %f %f", col_0_avg, col_1_avg, col_2_avg, col_3_avg, col_4_avg);
    uint8_t neo_idx_col_0 = LED_STRIP_LED_COUNT;
    uint8_t neo_idx_col_0_off = 0;
    //uint8_t idx_cntr_0 = 0;

    int idx_cntr_0 = (int)((col_0_sum*4) / 0.2f);
    int idx_cntr_1 = (int)((col_1_sum*5) / 0.2f);
    int idx_cntr_2 = (int)((col_2_sum*6) / 0.2f);
    int idx_cntr_3 = (int)((col_3_sum*7) / 0.2f);    
    int idx_cntr_4 = (int)((col_4_sum*8) / 0.2f);
    int idx_cntr_5 = (int)((col_5_sum*9) / 0.2f);
    int idx_cntr_6 = (int)((col_6_sum*10) / 0.2f);
    int idx_cntr_7 = (int)((col_7_sum*11) / 0.2f);    

    if (idx_cntr_0 < 0)
        idx_cntr_0 = 0;

    if (idx_cntr_0 > 5)
        idx_cntr_0 = 5;

    if (idx_cntr_1 < 0)
        idx_cntr_1 = 0;

    if (idx_cntr_1 > 5)
        idx_cntr_1 = 5;

    if (idx_cntr_2 < 0)
        idx_cntr_2 = 0;

    if (idx_cntr_2 > 5)
        idx_cntr_2 = 5;

    if (idx_cntr_3 < 0)
        idx_cntr_3 = 0;

    if (idx_cntr_3 > 5)
        idx_cntr_3 = 5;

    if (idx_cntr_4 < 0)
        idx_cntr_4 = 0;

    if (idx_cntr_4 > 5)
        idx_cntr_4 = 5;

    if (idx_cntr_5 < 0)
        idx_cntr_5 = 0;

    if (idx_cntr_5 > 5)
        idx_cntr_5 = 5;

    if (idx_cntr_6 < 0)
        idx_cntr_6 = 0;

    if (idx_cntr_6 > 5)
        idx_cntr_6 = 5;

    if (idx_cntr_7 < 0)
        idx_cntr_7 = 0;

    if (idx_cntr_7 > 5)
        idx_cntr_7 = 5;
    
    //ESP_ERROR_CHECK(led_strip_set_pixel_hsv(strip, 32, 0, 255, 100));
    // for (int j = 5; j > 0; j--)
    // {
    //     for (int i = 0; i < 5; i++)
    //     {
    //         if (i < idx_cntr_0)
    //         {
    //             if ((i == (idx_cntr_0 - 1)) && (i != 0))
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_0_idx_arr[i], 0, 255, 50));
    //             }  
    //             else
    //             {                 
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_0_idx_arr[i], 0, 255, 100));
    //             }
    //         }

    //         else
    //         {
    //             if (i == idx_cntr_0)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_0_idx_arr[i], 0, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_0_idx_arr[i], 0, 255, 0)); 
    //             }
    //         }
    //         if (i < idx_cntr_1)
    //         {
    //             if (i == (idx_cntr_1 - 1) && (i != 0))
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                     strip, col_1_idx_arr[i], 45, 255, j*20));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                     strip, col_1_idx_arr[i], 45, 255, 100));
    //             }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_1)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_1_idx_arr[i], 45, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_1_idx_arr[i], 45, 255, 0)); 
    //             }
    //         }

    //         if (i < idx_cntr_2)
    //         {
    //         if (i == (idx_cntr_2 - 1))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_2_idx_arr[i], 90, 255, j*20));
    //         }
    //         else 
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_2_idx_arr[i], 90, 255, 100));
    //         }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_2)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_2_idx_arr[i], 90, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_2_idx_arr[i], 90, 255, 0)); 
    //             }
    //         }

    //         if (i < idx_cntr_3)
    //         {
    //         if (i == (idx_cntr_3 - 1) && (i != 0))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_3_idx_arr[i], 135, 255, j*20));
    //         }
    //         else
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_3_idx_arr[i], 135, 255, 100));
    //         }
    //         }           
    //         else
    //         {
    //             if (i == idx_cntr_3)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_3_idx_arr[i], 135, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_3_idx_arr[i], 135, 255, 0)); 
    //             }
    //         }

    //         if (i < idx_cntr_4)
    //         {
    //         if (i == (idx_cntr_4 - 1) && (i != 0))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_4_idx_arr[i], 180, 255, j*20));
    //         }
    //         else
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_4_idx_arr[i], 180, 255, 100));
    //         }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_4)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_4_idx_arr[i], 180, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_4_idx_arr[i], 180, 255, 0)); 
    //             }
    //         }

    //         if (i < idx_cntr_5)
    //         {
    //         if (i == (idx_cntr_5 - 1) && (i != 0))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_5_idx_arr[i], 225, 255, j*20));
    //         }
    //         else
    //         {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                     strip, col_5_idx_arr[i], 225, 255, 100));
    //         }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_5)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_5_idx_arr[i], 225, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_5_idx_arr[i], 225, 255, 0)); 
    //             }
    //         }

    //         if (i < idx_cntr_6)
    //         {
    //         if (i == (idx_cntr_6 - 1) && (i != 0))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_6_idx_arr[i], 270, 255, j*20));
    //         }
    //         else
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_6_idx_arr[i], 270, 255, 100));
    //         }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_6 )
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_6_idx_arr[i], 270, 255, j*10));
    //             }
    //             else
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_6_idx_arr[i], 270, 255, 0));
    //             } 
    //         }
    //         if (i < idx_cntr_7)
    //         {
    //         if (i == (idx_cntr_7 - 1) && (i != 0))
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_7_idx_arr[i], 315, 255, j*20));
    //         }
    //         else
    //         {
    //             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_6_idx_arr[i], 315, 255, 100));
    //         }
    //         }
    //         else
    //         {
    //             if (i == idx_cntr_7)
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_7_idx_arr[i], 315, 255, j*10));
    //             }
    //             else 
    //             {
    //                 ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
    //                 strip, col_7_idx_arr[i], 315, 255, 0)); 
    //             }
    //         }

    //         //ESP_ERROR_CHECK(led_strip_refresh_async(strip));
    //         //ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));
        
    //     //ESP_ERROR_CHECK(led_strip_set_pixel_hsv(strip, 32, 0, 255, 100));
    //     }
    //     ESP_ERROR_CHECK(led_strip_refresh_async(strip));
    //     ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }



    for (int i = 0; i < 5; i++)
    {
        if (i < idx_cntr_0)
        {
            if ((i == (idx_cntr_0 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_0_idx_arr[i], 0, 255, brightness_low));
            }
            else if ((i == (idx_cntr_0 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_0_idx_arr[i], 0, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_0_idx_arr[i], 0, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_0)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_0_idx_arr[i], 0, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_0_idx_arr[i], 0, 255, brightness_off));
            }
        }

        if (i < idx_cntr_1)
        {
            if ((i == (idx_cntr_1 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_1_idx_arr[i], 45, 255, brightness_low));
            }
            else if ((i == (idx_cntr_1 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_1_idx_arr[i], 45, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_1_idx_arr[i], 45, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_1)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_1_idx_arr[i], 45, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_1_idx_arr[i], 45, 255, brightness_off));
            }
        }

        if (i < idx_cntr_2)
        {
            if ((i == (idx_cntr_2 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_2_idx_arr[i], 90, 255, brightness_low));
            }
            else if ((i == (idx_cntr_2 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_2_idx_arr[i], 90, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_2_idx_arr[i], 90, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_2)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_2_idx_arr[i], 90, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_2_idx_arr[i], 90, 255, brightness_off));
            }
        }

        if (i < idx_cntr_3)
        {
            if ((i == (idx_cntr_3 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_3_idx_arr[i], 135, 255, brightness_low));
            }
            else if ((i == (idx_cntr_3 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_3_idx_arr[i], 135, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_3_idx_arr[i], 135, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_3)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_3_idx_arr[i], 135, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_3_idx_arr[i], 135, 255, brightness_off));
            }
        }

        if (i < idx_cntr_4)
        {
            if ((i == (idx_cntr_4 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_4_idx_arr[i], 180, 255, brightness_low));
            }
            else if ((i == (idx_cntr_4 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_4_idx_arr[i], 180, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_4_idx_arr[i], 180, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_4)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_4_idx_arr[i], 180, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_4_idx_arr[i], 180, 255, brightness_off));
            }
        }

        if (i < idx_cntr_5)
        {
            if ((i == (idx_cntr_5 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_5_idx_arr[i], 225, 255, brightness_low));
            }
            else if ((i == (idx_cntr_5 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_5_idx_arr[i], 225, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_5_idx_arr[i], 225, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_5)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_5_idx_arr[i], 225, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_5_idx_arr[i], 225, 255, brightness_off));
            }
        }

        if (i < idx_cntr_6)
        {
            if ((i == (idx_cntr_6 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_6_idx_arr[i], 270, 255, brightness_low));
            }
            else if ((i == (idx_cntr_6 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_6_idx_arr[i], 270, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_6_idx_arr[i], 270, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_6)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_6_idx_arr[i], 270, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_6_idx_arr[i], 270, 255, brightness_off));
            }
        }

        if (i < idx_cntr_7)
        {
            if ((i == (idx_cntr_7 - 1)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_7_idx_arr[i], 315, 255, brightness_low));
            }
            else if ((i == (idx_cntr_7 - 2)) && (i != 0))
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_7_idx_arr[i], 315, 255, brightness_mid));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_7_idx_arr[i], 315, 255, brightness_full));
            }
        }
        else
        {
            if (i == idx_cntr_7)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_7_idx_arr[i], 315, 255, brightness_low));
            }
            else
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, col_7_idx_arr[i], 315, 255, brightness_off));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));

    }

        ESP_ERROR_CHECK(led_strip_refresh_async(strip));
        ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
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
// static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl,  
//                                      esp_a2d_audio_buff_t *audio_buf)  
// {  
//     //13 - 16 milliseconds to finsih callback, 13000 - 16000 us
//     //uint64_t start = esp_timer_get_time();

//     //Callback frequency, 10-30 ms, time since prvious callback, mainly around 20000 us (20 ms)
//     // static int64_t last_time = 0;

//     // int64_t now = esp_timer_get_time();

//     // if (last_time != 0)
//     // {
//     //     ESP_LOGI("TIMING", "Time since previous callback: %" PRId64 " us",
//     //              now - last_time);
//     // }

//     // last_time = now;
    
    
//     //int64_t t0 = esp_timer_get_time();

//     bt_a2d_audio_data_hdl(conn_hdl, audio_buf);  
  
//     if (!sbc_inited) {  
//         sbc_init(&sbc, 0L);  
//         sbc.endian = SBC_LE;                 /* match your audio pipeline */  
//         sbc_inited = true;  
//     }  
  
//     size_t packet_len = audio_buf->data_len;  
//     uint16_t pos = 0, pcm_pos = 0;  
//     ssize_t framelen = 0; 
//     size_t written = 0;  
    
//     //119 SBC bytes in a frame
//     //ESP_LOG_BUFFER_HEX("RAW SBC Bytes At Top of Callback", audio_buf->data, 32);

//     //int64_t t0 = esp_timer_get_time();
//     //ESP_LOGI("BYTES Gen:", "%d", audio_buf->data_len);

//     while (pos < packet_len) 
//     {
//         written = 0;  
//         framelen = sbc_decode(&sbc,  
//                               audio_buf->data + pos, packet_len - pos,  
//                               pcm + pcm_pos, sizeof(pcm) - pcm_pos,  
//                               &written);  
//         if (framelen <= 0)  
//             break;  
        
//         pos     += framelen;  
//         pcm_pos += written;
//     }     

//     memcpy(pcm_copy, pcm, sizeof(pcm));
//     //ESP_LOG_BUFFER_HEX("RAW SBC Bytes At End of Callback", audio_buf->data, 32);
//     //i2s_channel_write(tx_handle, pcm_copy, sizeof(4096), NULL, 1000);
//     //esp_err_t ret = i2s_channel_write(tx_handle, pcm_copy, sizeof(pcm_copy), NULL, 1000);

//     //int64_t t1 = esp_timer_get_time();
//     //ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);

//     //ESP_LOG_BUFFER_HEX("PCM_CALLBACK", pcm_copy, 16);
//     //uint32_t sum = checksum(pcm_copy, 4096);
//     //ESP_LOGI("CALLBACK", "PCM checksum: %" PRIu32, sum);
//     //ESP_LOGI("TIMING", "PCM Value in callback: %02X %02X %02X %02X %02X %02X",pcm_copy[0], pcm_copy[1], pcm_copy[2], pcm_copy[3], pcm_copy[10], pcm_copy[30]);
//     //memmove(pcm_copy+4096, pcm_copy, sizeof(pcm));  

    
//     //int64_t t0 = esp_timer_get_time();
//     BaseType_t result = xQueueSend(work_queue, pcm_copy, 0);
    
//     //int64_t t1 = esp_timer_get_time();
//     //ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);
//     //ESP_LOGI("RESULT: ", "%d", result);
//     if (++s_pckt_cnt % 100 == 0)  
//     {
//     ESP_LOGI(BT_AV_TAG,  
//                  "pos=%u" " pcm=%u" " last_frame len:=%d" " Bytes of PCM Out:=%d" " in=%02X %02X %02X %02X %02X %02X %02X %02X" " out=%02X %02X %02X %02X %02X %02X %02X %02X" " %d",  
//                  pos, pcm_pos, (int)framelen, sizeof(pcm),
//                  audio_buf->data[0], audio_buf->data[1],  
//                  audio_buf->data[2], audio_buf->data[3],
//                 audio_buf->data[4], audio_buf->data[5],  
//                 audio_buf->data[6], audio_buf->data[7], 
//                 pcm[0], pcm[1], pcm[2], pcm[3],
//                 pcm[4], pcm[5], pcm[6], pcm[7], 
//                 sbc.frequency);  
//     }
//     esp_a2d_audio_buff_free(audio_buf); 
//     //int64_t end = esp_timer_get_time();
//     // ESP_LOGI("TIMING", "Callback execution: %" PRId64 " us",
//     //          end - start);
//     // int64_t t1 = esp_timer_get_time();
//     // ESP_LOGI("TIMING", "BT CB Exec. Time: %" PRId64 " us", t1-t0);


static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl,  
                                     esp_a2d_audio_buff_t *audio_buf)  
{  
    //13 - 16 milliseconds to finsih callback, 13000 - 16000 us
    //uint64_t start = esp_timer_get_time();

    //Callback frequency, 10-30 ms, time since prvious callback, mainly around 20000 us (20 ms)
    // static int64_t last_time = 0;

    bt_a2d_audio_data_hdl(conn_hdl, audio_buf);  
  
    if (!sbc_inited) {  
        sbc_init(&sbc, 0L);  
        sbc.endian = SBC_LE;                 /* match your audio pipeline */  
        sbc_inited = true;  
    }  
  
    size_t packet_len = audio_buf->data_len;  
    //ESP_LOGI("BT CB: ", "%X %X %X", audio_buf->data[0], audio_buf->data[119], audio_buf->data[238]);

    BaseType_t write_buf_res = xRingbufferSend(buf_handle, audio_buf->data, packet_len, 0);
    if (write_buf_res != pdTRUE) {
        ESP_LOGE("BT CB: ", "Failed to send item\n");
    }
    
    esp_a2d_audio_buff_free(audio_buf); 
    //int64_t now = esp_timer_get_time();
    //ESP_LOGI("BT CB Task", "Time of decoder task loop: %" PRId64 " us", now - start);
    //ESP_LOGI("BT CB", "Running on Core %d", xPortGetCoreID());
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

    SemaphoreHandle_t pcm_mutex;
    pcm_mutex = xSemaphoreCreateMutex();
    configASSERT(pcm_mutex != NULL);


    buf_handle = xRingbufferCreate(952*8, RINGBUF_TYPE_NOSPLIT);

    if (buf_handle == NULL) {
    ESP_LOGE("MAIN: ", "Failed to create ring buffer\n");
    }
    ESP_LOGE("MAIN: ", "Successfully created ring buffer\n");

    esp_err_t fft_ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (fft_ret  != ESP_OK) 
    {
        ESP_LOGE("Main: ", "Not possible to initialize FFT. Error = %i", fft_ret);
        return;
    }

    i2s_configure();
    int ret = sbc_init(&sbc, 0L);
    sbc.frequency = 0x00;
    ESP_LOGI(BT_AV_TAG, "SBC INIT RET VALUE:" "%d", ret);
    ESP_ERROR_CHECK(bredr_app_common_init());

    //decoder_queue = xQueueCreate(8, 4096);

    //configASSERT(decoder_queue != NULL);

    // xTaskCreate(
    //     worker_task,
    //     "worker_task",
    //     12288,
    //     NULL,
    //     5,
    //     NULL
    // );

    // typedef struct {
    //     led_strip_handle_t led_strip;
    //     uint16_t *pcm_copy;
    //     size_t pcm_length;
    // } led_task_params_t;

    
    fft_led_struct.led_strip = configure_led();
    led_strip_clear(fft_led_struct.led_strip);

    
    
    ESP_LOGI("LED Main", "Struct handle: %p", &fft_led_struct);
    ESP_LOGI("LED Main", "Strip handle: %p", fft_led_struct.led_strip);
    ESP_LOGI("LED Main", "PCM Copy FFT handle: %p", &fft_led_struct.pcm_copy_for_fft);
    ESP_LOGI("LED Main", "PCM Copy handle: %p", pcm_copy);

    
    xTaskCreate(
    decoder_task,
    "decoder_task",
    4096*4,
    NULL,
    5,
    NULL
    );

    xTaskCreatePinnedToCore(
    write_led_task, 
    "write_led_task",
    8192, 
    &fft_led_struct, 
    5, 
    &fft_led_task_handle,
    1
    );

    //led_strip_handle_t led_strip = configure_led();

    //ESP_LOGI("LED Main", "PCM Copy handle: %p", &fft_led_struct.pcm_copy_for_fft);


//     xTaskCreate(
//     write_led_task,
//     "write_led_task",
//     4096*4,
//     led_strip,
//     1,
//     NULL
// );

    // ESP_LOGI(FFT_TAG, "Start FFT Init.");
    // esp_err_t fft_ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    // if (fft_ret  != ESP_OK) 
    // {
    //     ESP_LOGE(FFT_TAG, "Not possible to initialize FFT. Error = %i", fft_ret);
    //     return;
    // }
    // dsps_wind_hann_f32(wind, N);

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL, NULL);
    // for (int i = 0; i < LED_STRIP_LED_COUNT; i++)
    // {
    //     ESP_ERROR_CHECK(led_strip_set_pixel_hsv(fft_led_struct.led_strip, i, 9*i, 255, 100));
    //     ESP_ERROR_CHECK(led_strip_refresh_async(fft_led_struct.led_strip));
    //     ESP_ERROR_CHECK(led_strip_refresh_async_done(fft_led_struct.led_strip));
    // }
    // vTaskDelay(pdMS_TO_TICKS(10));
    // ESP_ERROR_CHECK(led_strip_clear(fft_led_struct.led_strip));
    // while (1) 
    // {
    //     for (int i = 0; i < LED_STRIP_LED_COUNT; i++)
    //     {
    //         ESP_ERROR_CHECK(led_strip_set_pixel_hsv(led_strip, i, 9*i, 255, 100));
    //         ESP_ERROR_CHECK(led_strip_refresh_async(led_strip));
    //         ESP_ERROR_CHECK(led_strip_refresh_async_done(led_strip));
    //     }
    // }

//led_strip_handle_t strip = configure_led();
//uint8_t col_0_idx_arr[5] = {32, 24, 16, 8, 0};

// while (1)
// {
//     // Random number of illuminated LEDs: 0 to NUM_COLUMN_LEDS
//     int idx_cntr_0 = esp_random() % (5 + 1);

//     for (int i = 0; i < 5; i++)
//     {
//         if (i < idx_cntr_0)
//         {
//             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
//                 strip, col_0_idx_arr[i], 0, 255, 100));
//         }
//         else
//         {
//             ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
//                 strip, col_0_idx_arr[i], 0, 0, 0));
//         }
//     }

//     ESP_ERROR_CHECK(led_strip_refresh_async(strip));
//     ESP_ERROR_CHECK(led_strip_refresh_async_done(strip));

//     vTaskDelay(pdMS_TO_TICKS(250));
// }
}




// // void app_main(void)
// // {
// //     led_strip_handle_t led_strip = configure_led();
// //     ESP_ERROR_CHECK(led_strip_clear(led_strip));

// //     // /* Refresh the strip to send data */
// //     //ESP_ERROR_CHECK(led_strip_refresh(led_strip));
// //     ESP_LOGI(TAG, "Start blinking LED strip");
// //     while (1) 
// //     {
// //             /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
// //             // for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
// //             //      ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 200, 0, 0));
// //             // }
// //             // // /* Refresh the strip to send data */
// //             // ESP_ERROR_CHECK(led_strip_refresh(led_strip));

// //         // for (int i = 0; i < 8; i += 4)
// //         // {
// //         //     for (int j = 0; j < 5; j++)
// //         //     {
// //         //         ESP_ERROR_CHECK(led_strip_set_pixel_hsv(led_strip, i + 8*j, i+j, 200, 200));
// //         //         //ESP_ERROR_CHECK(led_strip_refresh(led_strip));
// //         //         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, (i+1) + 8*j, 120 + (i+j), 200, 0));
// //         //         //ESP_ERROR_CHECK(led_strip_refresh(led_strip));
// //         //         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, (i+2) + 8*j, 0, 0, 200));
// //         //         //ESP_ERROR_CHECK(led_strip_refresh(led_strip));
// //         //         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, (i+3) + 8*j, 200, 200, 0));
// //         //         ESP_ERROR_CHECK(led_strip_refresh_async(led_strip));
// //         //         ESP_ERROR_CHECK(led_strip_refresh_async_done(led_strip));

// //         //     }
// //         // }
// //     }
// // }

// Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "driver/spi_master.h"
// #include "soc/gpio_struct.h"
// #include "driver/gpio.h"
// #include "driver/uart.h"
// #include "soc/uart_struct.h"
// #include <math.h>

// #include "esp_dsp.h"
// #include "esp_heap_caps.h"

// static const char *TAG = "main";

// // This example shows how to use FFT from esp-dsp library

// #define N_SAMPLES 4096
// int N = N_SAMPLES;
// // Input test array
// //__attribute__((aligned(16)))
// //static float x1[N_SAMPLES];
// //__attribute__((aligned(16)))
// //float x2[N_SAMPLES];
// // Window coefficients
// //__attribute__((aligned(16)))
// // static float wind[N_SAMPLES];
// // working complex array
// //__attribute__((aligned(16)))
// float tone[N_SAMPLES];
// float y_cf[N_SAMPLES*2];
// // Pointers to result arrays
// // float *y1_cf = &y_cf[0];
// // float *y2_cf = &y_cf[N_SAMPLES];

// // Sum of y1 and y2
// //__attribute__((aligned(16)))
// //float sum_y[N_SAMPLES / 2];
// float magnitude[N_SAMPLES];


// void app_main()
// {
//     ESP_LOGI(TAG, "Start Example.");
//     esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
//     if (ret  != ESP_OK) {
//         ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
//         return;
//     }

//     // Generate hann window
//     //dsps_wind_hann_f32(wind, N);
//     // Generate input signal for x1 A=1 , F=0.1

//     //Using normalized frequency 
//     uint16_t sample_rate = 44100;
//     uint16_t tone_freq = 10;
//     uint16_t ampl = 1;
    
//     //generate signal from normalized freq
//     //100/44100 = 0.0022
//     float norm_freq = (float)(tone_freq)/(float)(sample_rate);


//     esp_err_t tone_ret = dsps_tone_gen_f32(tone, N_SAMPLES, 1.0, norm_freq, 0.0);
//     if (tone_ret != ESP_OK) 
//     {
//         printf("Tone generation failed: %d\n", ret);
//     }

//     for (int i = 0; i < N_SAMPLES; i++)
//     {
//         y_cf[i*2] = tone[i];
//         y_cf[(i*2) + 1] = 0.0f;
//     }


//     ESP_LOGI(TAG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//     y_cf[0], y_cf[1], y_cf[2], y_cf[3], y_cf[4],
//     y_cf[5], y_cf[6], y_cf[7], y_cf[8], y_cf[9],
//     y_cf[10], y_cf[11], y_cf[12], y_cf[13], y_cf[14],
//     y_cf[15], y_cf[16], y_cf[17], y_cf[18], y_cf[19]);


//     // // Generate hann window
//     // // Generate input signal for x1 A=1 , F=0.1
//     // dsps_tone_gen_f32(y_cf, N, 1.0, 0.16,  0);
//     // ESP_LOGI(TAG, "%d %d %d %d %d %d", y_cf[0], y_cf[1], y_cf[2], y_cf[3], y_cf[4], y_cf[5]);


//     ESP_ERROR_CHECK(dsps_fft2r_fc32(y_cf, N_SAMPLES));
//     ESP_ERROR_CHECK(dsps_bit_rev_fc32(y_cf, N_SAMPLES));

//     ESP_LOGI(TAG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//     y_cf[0], y_cf[1], y_cf[2], y_cf[3], y_cf[4],
//     y_cf[5], y_cf[6], y_cf[7], y_cf[8], y_cf[9],
//     y_cf[10], y_cf[11], y_cf[12], y_cf[13], y_cf[14],
//     y_cf[15], y_cf[16], y_cf[17], y_cf[18], y_cf[19]);

//     //this is the final real output of fft from x1
//     for (int i = 0; i <= N/2; i++)
//     {
//         magnitude[i] = (sqrtf((y_cf[i*2]*y_cf[i*2]) + (y_cf[((i*2)+1)]*y_cf[((i*2)+1)]))/N_SAMPLES);
//     }

//     ESP_LOGI(TAG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//     magnitude[0], magnitude[1], magnitude[2], magnitude[3], magnitude[4],
//     magnitude[5], magnitude[6], magnitude[7], magnitude[8], magnitude[9],
//     magnitude[10], magnitude[11], magnitude[12], magnitude[13], magnitude[14],
//     magnitude[15], magnitude[16], magnitude[17], magnitude[18], magnitude[19]);

//     // Generate input signal for x2 A=0.1,F=0.2

//     // Convert two input vectors to one complex vector
//     // for (int i = 0 ; i < N ; i++) {
//     //     y_cf[i * 2 + 0] = x1[i] * wind[i];
//     //     y_cf[i * 2 + 1] = x2[i] * wind[i];
//     // }

//     // for (int i = 0; i < N; i++)
//     // {
//     //     y_cf[i*2] = x1[i];   //real 
//     //     y_cf[(i*2) + 1] = 0;          //imag
//     // }
//     // FFT
//     //unsigned int start_b = dsp_get_cpu_cycle_count();
//     //dsps_fft2r_fc32(y_cf, N);
//     //unsigned int end_b = dsp_get_cpu_cycle_count();
//     // Bit reverse
//     //dsps_bit_rev_fc32(y_cf, N);
//     // Convert one complex vector to two complex vectors
//     //dsps_cplx2reC_fc32(y_cf, N);

//     //this is the final real output of fft from x1
//     // for (int i = 0; i < N; i++)
//     // {
//     //     magnitude[i] = y_cf[i*2];
//     // }

//     // float max_value = 0.0;
//     // for (int i = 0; i < N; i++)
//     // {
//     //     if (magnitude[i] > max_value)
//     //     {
//     //         max_value = magnitude[i];
//     //     }
//     // }

//     //ESP_LOGI(TAG, "Max %d", max_value);
// 	// for (int i = 0 ; i < N ; i++)
// 	// {
// 	// 	y_cf[i] = 20 * log10f((y_cf[(N/2)+i*2])/N);
// 	// }

    
//     // float max_val = 0;
//     // uint16_t max_idx = 0;
//     // for (int i = 0; i < N; i++)
//     // {
//     //     if (y_cf[i])
//     // }
//     // for (int i = 0; i < N/2; i++)
//     // {

//     // }
//     //ESP_LOGI(TAG, "%d %d %d %d %d %d", magnitude[0], magnitude[1], magnitude[2], magnitude[3], magnitude[4], magnitude[5]);
//     // for (int i = 0 ; i < N / 2 ; i++) {
//     //     y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);
//     //     y2_cf[i] = 10 * log10f((y2_cf[i * 2 + 0] * y2_cf[i * 2 + 0] + y2_cf[i * 2 + 1] * y2_cf[i * 2 + 1]) / N);
//     //     // Simple way to show two power spectrums as one plot
//     //     sum_y[i] = fmax(y1_cf[i], y2_cf[i]);
//     // }

//     // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
//     // ESP_LOGW(TAG, "Signal x1");
//     // dsps_view(y1_cf, N / 2, 64, 10,  -60, 40, '|');
//     // ESP_LOGW(TAG, "Signal x2");
//     // dsps_view(y2_cf, N / 2, 64, 10,  -60, 40, '|');
//     // ESP_LOGW(TAG, "Signals x1 and x2 on one plot");
//     //dsps_view(y_cf, N / 2, 128, 10,  -60, 40, '|');
//     //ESP_LOGI(TAG, "FFT for %i complex points take %i cycles", N, end_b - start_b);

//     //ESP_LOGI(TAG, "End Example.");
// }



// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_err.h"
// #include "led_strip.h"

// #define LED_GPIO  4       // Change to your DIN GPIO
// #define LED_COUNT 40      // 8x5 matrix

// void app_main(void)
// {
//     led_strip_handle_t strip;

//     led_strip_config_t strip_config = {
//         .strip_gpio_num = LED_GPIO,
//         .max_leds = LED_COUNT,
//         .led_model = LED_MODEL_WS2812,
//         .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
//     };

//     led_strip_rmt_config_t rmt_config = {
//         .resolution_hz = 10000000,
//         .flags.with_dma = false,
//     };

//     ESP_ERROR_CHECK(led_strip_new_rmt_device(
//         &strip_config, &rmt_config, &strip));

//     ESP_ERROR_CHECK(led_strip_clear(strip));

//     while (1)
//     {
//         // RED
//         for (int i = 0; i < LED_COUNT; i++)
//         {
//             ESP_ERROR_CHECK(led_strip_set_pixel(strip, i, 100, 0, 0));
//         }
//         ESP_ERROR_CHECK(led_strip_refresh(strip));
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
