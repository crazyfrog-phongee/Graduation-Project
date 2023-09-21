#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "common.h"
#include "main.h"

#define handle_error_en(en, msg) \
    do                           \
    {                            \
        errno = en;              \
        perror(msg);             \
        exit(EXIT_FAILURE);      \
    } while (0)

static const char *TAG = "FREERTOS";
sx1278_node_t sx1278_node;

void *peripheral_task(void *arg)
{
    /* USER CODE BEGIN StartDefaultTask */
    static int count_to_break;
    static uint32_t adc_val[22] = {0};
    static uint16_t ADC_VREF_mV = 3300;
    static float battery = 0;
    char data_log[40];

    memset(data_log, 0, 40 * sizeof(char));
    sprintf(data_log, "Peripheral task");
    LOG(TAG, data_log);

    volatile clock_t time_keeper_1 = clock();
    /* Infinite loop */
    for (;;)
    {

        for (uint8_t i = 0; i < 20; i++)
        {
            adc_val[i] = rand() % 100 + 3900;
        }

        time_keeper_1 = clock();
        printf("%ld\n", time_keeper_1);
        while ((clock() - time_keeper_1) <= 3000)
        {
            adc_val[20] = 0;
            adc_val[21] = 0;
            for (uint8_t i = 0; i < 20; i += 2)
            {
                adc_val[20] += adc_val[i];
                adc_val[21] += adc_val[i + 1];
            }
            adc_val[20] /= 10;
            adc_val[21] /= 10;

            ADC_VREF_mV = (uint16_t)(VREFINT * ADC_RESOLUTION * 1000 / adc_val[21]) - 130;
            battery = (float)(((float)adc_val[20] * RATIO * ADC_VREF_mV / ADC_RESOLUTION) / 1000);
            ftoa(battery, sx1278_node.battery, 2);

            /* Logging data */
            sprintf(data_log, "Vref: %d, Vbat: %s", ADC_VREF_mV, sx1278_node.battery);
            LOG(TAG, data_log);
            count_to_break++;
            printf("In while (): %ld\n", clock());
            sleep(1);
        }
        
        if (count_to_break == 30)
        {
            pthread_exit(NULL);
        }
        sleep(2);
    }
    
    /* USER CODE END StartDefaultTask */
}

int main(int argc, char const *argv[])
{
    pthread_t peripheral_thread_id;
    int ret;

    sx1278_node.node_id = 10;

    if ((ret = pthread_create(&peripheral_thread_id, NULL, peripheral_task, NULL)))
    {
        handle_error_en(ret, "pthread_create");
    }

    pthread_join(peripheral_thread_id, NULL);

    return 0;
}
