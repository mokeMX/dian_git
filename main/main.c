#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <math.h>

#include "led.h"
#include "key.h"        // 包含板载 BOOT 按键驱动
#include "xl9555.h"     // 包含扩展按键 KEY0/1/2/3 驱动
#include "myi2s.h"      // 包含你写的 I2S 驱动

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 1. 定义波形模式 */
typedef enum {
    MODE_SAWTOOTH_500 = 0, // 500Hz 锯齿波
    MODE_BEAT_SINE,        // 1001Hz(L) + 999Hz(R) 拍频
    MODE_STEREO_SINE       // 1kHz(L) + 4kHz(R) 立体声
} wave_mode_t;

/* 2. 定义声道屏蔽模式 */
typedef enum {
    CHAN_R_ONLY = 0,       // 仅右声道 (默认)
    CHAN_L_R,              // 左右声道混合 (L+R)
    CHAN_L_ONLY            // 仅左声道
} chan_mode_t;

/* 全局状态变量 (默认：右声道 500Hz 锯齿波) */
volatile wave_mode_t g_wave_mode = MODE_SAWTOOTH_500;
volatile chan_mode_t g_chan_mode = CHAN_R_ONLY;

/**
 * @brief 音频生成与发送任务
 */
void audio_task(void *pvParameters) {
    int16_t samples[256];  // 128个采样点 * 2个声道 (左右) = 256
    double t = 0;
    const double dt = 1.0 / I2S_SAMPLE_RATE;
    const int16_t amplitude = 12000; // 振幅 (控制音量，最大32767)

    while (1) {
        for (int i = 0; i < 128; i++) {
            int16_t left_val = 0;
            int16_t right_val = 0;

            // 1. 根据模式生成对应的数学波形
            switch (g_wave_mode) {
                case MODE_SAWTOOTH_500:
                    // 500Hz 锯齿波：fmod(t*f, 1) * 2 - 1 将数值映射到 -1.0 ~ 1.0 之间
                    left_val = 0;
                    right_val = (int16_t)(amplitude * (fmod(t * 500.0, 1.0) * 2.0 - 1.0));
                    break;
                    
                case MODE_BEAT_SINE:
                    // 左 1001Hz, 右 999Hz
                    left_val  = (int16_t)(amplitude * sin(2.0 * M_PI * 1001.0 * t));
                    right_val = (int16_t)(amplitude * sin(2.0 * M_PI * 999.0 * t));
                    break;
                    
                case MODE_STEREO_SINE:
                    // 左 1000Hz, 右 4000Hz
                    left_val  = (int16_t)(amplitude * sin(2.0 * M_PI * 1000.0 * t));
                    right_val = (int16_t)(amplitude * sin(2.0 * M_PI * 4000.0 * t));
                    break;
            }

            // 2. 根据声道模式进行软屏蔽
            if (g_chan_mode == CHAN_L_ONLY) right_val = 0;
            if (g_chan_mode == CHAN_R_ONLY) left_val = 0;

            // 3. 存入 I2S 缓冲区 (交替存储：L, R, L, R...)
            samples[i * 2]     = left_val;
            samples[i * 2 + 1] = right_val;
            
            t += dt;
        }
        
        // 防止时间 t 无限累加导致浮点精度丢失
        if (t > 1.0) t -= 1.0; 

        // 调用 myi2s.c 中的发送函数 (sizeof(samples) 即 512 字节)
        i2s_tx_write((uint8_t *)samples, sizeof(samples));
    }
}

/**
 * @brief 程序入口
 */
void app_main(void)
{
    esp_err_t ret;
    uint8_t key_xl = 0; // XL9555 按键值
    uint8_t key_bt = 0; // 板载 BOOT 按键值

    // 初始化 NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 硬件外设初始化
    led_init();
    key_init();             // 初始化板载 BOOT 键
    xl9555_init();          // 初始化 XL9555 扩展 IO
    myi2s_init();           // 初始化 I2S

    printf("I2S Audio Lab Started!\n");
    printf("Default: 500Hz Sawtooth (Right Only)\n");

    // 创建音频生成任务（固定在核心1，优先级设高，防止音频卡顿）
    xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 10, NULL, 1);

    // 主循环：按键扫描与状态机控制
    while(1)
    {
        key_xl = xl9555_key_scan(0); // 扫描扩展按键
        key_bt = key_scan(0);        // 扫描 BOOT 键

        // KEY0：触发拍频模式 (强制设为 L+R)
        if (key_xl == KEY0_PRES) {
            g_wave_mode = MODE_BEAT_SINE;
            g_chan_mode = CHAN_L_R;
            printf("KEY0 Pressed: Beating (L:1001Hz, R:999Hz, L+R)\n");
        }
        
        // KEY1：触发 1kHz/4kHz 模式 (强制设为 L+R)
        else if (key_xl == KEY1_PRES) {
            g_wave_mode = MODE_STEREO_SINE;
            g_chan_mode = CHAN_L_R;
            printf("KEY1 Pressed: Stereo (L:1kHz, R:4kHz, L+R)\n");
        }
        
        // KEY3：触发 500Hz 锯齿波模式 (强制设为仅右声道)
        else if (key_xl == KEY3_PRES) {
            g_wave_mode = MODE_SAWTOOTH_500;
            g_chan_mode = CHAN_R_ONLY;
            printf("KEY3 Pressed: Sawtooth 500Hz (Right Only)\n");
        }
        
        // BOOT：循环切换声道模式 (右 -> 左右 -> 左)
        if (key_bt == BOOT_PRES) {
            LED0_TOGGLE(); 
            g_chan_mode = (g_chan_mode + 1) % 3; 
            
            printf("BOOT Pressed: Channel Switch -> ");
            if (g_chan_mode == CHAN_R_ONLY) printf("Right Only\n");
            else if (g_chan_mode == CHAN_L_R) printf("L + R\n");
            else if (g_chan_mode == CHAN_L_ONLY) printf("Left Only\n");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 让出 CPU 控制权
    }
}