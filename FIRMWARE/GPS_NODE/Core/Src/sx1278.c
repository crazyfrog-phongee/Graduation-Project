/*
 * sx1278.c
 *
 * Created on: Oct 4, 2023
 * Author: PhongEE
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"
#include "gpio.h"

#include "sx1278.h"
#include "common.h"
#include "event_groups.h"

extern SPI_HandleTypeDef hspi2;
static const char *TAG = "SX1278";
extern EventGroupHandle_t sx1278_evt_group;

uint8_t sx1278_read_reg(uint8_t reg)
{
    uint8_t txByte = reg & 0x7f;
    uint8_t rxByte = 0x00;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &txByte, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    HAL_SPI_Receive(&hspi2, &rxByte, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    return rxByte;
}

void sx1278_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t ureg = reg | 0x80;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &ureg, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&hspi2, &val, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void sx1278_reset(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(10);
}

void sx1278_sleep(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void sx1278_standby(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx1278_rx_contiuous(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx1278_rx_single(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
}

void sx1278_tx(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void sx1278_cad(void)
{
    sx1278_set_irq(0x80);
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void sx1278_set_tx_power(uint8_t output_power)
{
    if (output_power > 15)
    {
        LOG(TAG, "Invalid output power");
        return;
    }
    // PA output pin: PA_BOOST pin
    sx1278_write_reg(REG_PA_CONFIG, PA_BOOST | output_power);
}

void sx1278_set_LNA_gain(uint8_t gain)
{
    if (gain > 6)
    {
        LOG(TAG, "Invalid gain");
        return;
    }

    if (gain == 0)
    {
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x04);
    }
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x00);
        sx1278_write_reg(REG_LNA, sx1278_read_reg(REG_LNA) | (gain << 5));
    }
}

void sx1278_set_freq(uint64_t freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    sx1278_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1278_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1278_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void sx1278_set_bandwidth(long band)
{
    int bw;
    if (band <= 7.8E3)
        bw = 0;
    else if (band <= 10.4E3)
        bw = 1;
    else if (band <= 15.6E3)
        bw = 2;
    else if (band <= 20.8E3)
        bw = 3;
    else if (band <= 31.25E3)
        bw = 4;
    else if (band <= 41.7E3)
        bw = 5;
    else if (band <= 62.5E3)
        bw = 6;
    else if (band <= 125E3)
        bw = 7;
    else if (band <= 250E3)
        bw = 8;
    else
        bw = 9;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void sx1278_set_sf(uint8_t sf)
{
    if (sf < 6 || sf > 12)
    {
        LOG(TAG, "Invalid spreading factor");
        return;
    }

    if (sf == 6)
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }
    sx1278_write_reg(REG_MODEM_CONFIG_2, (sx1278_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void sx1278_set_cr(uint8_t cr)
{
    if (cr < 5 || cr > 8)
    {
        LOG(TAG, "Invalid coding rate");
        return;
    }

    cr = cr - 4;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx1278_set_header(bool en, uint32_t size)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) | 0x01);
        sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    }
}

void sx1278_set_crc(bool en)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) | 0x04);
    else
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void sx1278_set_preamble(int len)
{
    sx1278_write_reg(REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
    sx1278_write_reg(REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

int sx1278_get_rssi(void)
{
    return (sx1278_read_reg(REG_PKT_RSSI_VALUE) - 164);
}

float sx1278_get_snr(void)
{
    return ((int8_t)sx1278_read_reg(REG_PKT_SNR_VALUE) * 0.25);
}

void sx1278_set_irq(uint8_t val)
{
    sx1278_write_reg(REG_DIO_MAPPING_1, val);
}

void sx1278_init(void)
{
    sx1278_reset();
    uint8_t ver = sx1278_read_reg(REG_VERSION);
    char data_log[20];
    sprintf(data_log, "Version: 0x%02x", (char)ver);
    LOG(TAG, data_log);
    sx1278_sleep();
    sx1278_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    sx1278_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1278_set_LNA_gain(0);
    sx1278_set_tx_power(15); // Pout = 10 dBm (10 mW)
    sx1278_set_freq(434000000);
    sx1278_set_bandwidth(125000); // Bandwidth: 250 kHz
    sx1278_set_sf(7);
    sx1278_set_cr(5);
    sx1278_set_preamble(12);
    sx1278_set_header(true, 0);
    sx1278_set_crc(true);
    //    sx1278_set_irq(0x00);
    sx1278_standby();
}

void sx1278_send_data(uint8_t *data_send, int size)
{
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, 0);
    sx1278_write_reg(REG_PAYLOAD_LENGTH, 0);
    for (int index = 0; index < size; index++)
    {
        sx1278_write_reg(REG_FIFO, data_send[index]);
    }
    sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    // Start transmission and wait for conclusion
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while (!(sx1278_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
    {
        HAL_Delay(10);
    }
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    //    sx1278_sleep();
}

void sx1278_start_recv_data(void)
{
    sx1278_set_irq(0x00);
    sx1278_rx_contiuous();
}

sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_t *node)
{
    sx1278_packet_t packet;
    int packet_len = strlen((char *)packet_data);
    packet.crc = get_crc_value(packet_data, packet_len - 1);
    if (packet.crc != packet_data[packet_len - 1])
    {
        LOG(TAG, "ERROR CRC");
        return SX1278_NOT_OK;
    }
    else
    {
        int res = sscanf((char *)packet_data, "$,%[^,],%[^,],%[^,],%[^,],%[^,],*", packet.opcode, packet.node_id, packet.gate_id, packet.period, packet.threshold);
        if (res < 5)
        {
            LOG(TAG, "ERROR PACKET");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.node_id) != node->node_id)
        {
            LOG(TAG, "ERROR NODE_ID");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.opcode) != DOWNLINK_RX_REQUEST_OPCODE)
        {
            LOG(TAG, "ERROR OPCODE");
            return SX1278_NOT_OK;
        }
        //        if (atoi(packet.gate_id) != sx1278_network.gate_id)
        //        {
        //        	LOG(TAG, "ERROR GATE_ID");
        //            return SX1278_NOT_OK;
        //        }
        node->gate_id = atoi(packet.gate_id);
        node->period = atoi(packet.period);
        strcpy(node->threshold, packet.threshold);
        char data_log[100];
        sprintf(data_log, "RECV PACKET: opcode:%s, period:%s, threshold:%s", packet.opcode, packet.period, packet.threshold);
        LOG(TAG, data_log);
        return SX1278_OK;
    }
}
sx1278_err_t sx1278_recv_data(uint8_t *data_recv, int *rssi, float *snr, sx1278_node_t *node)
{
    memset((char *)data_recv, '\0', strlen((char *)data_recv));
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);

    if (!(irq & IRQ_RX_DONE_MASK))
    {
        LOG(TAG, "Invalid RxDone Interrupt");
        return SX1278_INVALID_RX_DONE;
    }

    if (!(irq & IRQ_VALID_HEADER_MASK))
    {
        LOG(TAG, "Invalid Header Interrupt");
        return SX1278_INVALID_HEADER;
    }

    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
    {
        LOG(TAG, "Payload Crc Error Interrupt");
        return SX1278_PAYLOAD_CRC_ERROR;
    }

    int len = sx1278_read_reg(REG_RX_NB_BYTES);
    *rssi = sx1278_get_rssi();
    *snr = sx1278_get_snr();
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, sx1278_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    for (int index = 0; index < len; index++)
    {
        data_recv[index] = sx1278_read_reg(REG_FIFO);
    }
    sx1278_standby();
    return parse_packet(data_recv, node);
}

int get_random_value(int min, int max)
{
    srand(time(NULL));
    return (rand() % max) + min;
}

uint8_t get_crc_value(uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

bool listen_before_talk(void)
{
    EventBits_t evt_bits;
    while (1)
    {
        xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);
        sx1278_cad();
        evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (evt_bits & SX1278_DIO0_BIT)
        {
            int irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            LOG(TAG, "CAD timeout");
            sx1278_standby();
            return true;
        }
        else
        {
            int irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            sx1278_standby();
            TickType_t time_delay = (TickType_t)get_random_value(0, 50);
            LOG(TAG, "CAD detect");
            HAL_Delay(time_delay);
        }
    }
}

void send_respond(sx1278_opcode_type_t opcode, sx1278_node_t node, uint8_t *packet)
{
    memset((char *)packet, '\0', strlen((char *)packet));
    if (opcode == UPLINK_TX_RESPOND_OPCODE)
    {
        sprintf((char *)packet, "$,%d,%d,%d,%s,%s,%d,%s,*", opcode, node.node_id, node.gate_id, node.temp, node.battery, node.period, node.threshold);
        int size = strlen((char *)packet);
        packet[size] = get_crc_value(packet, size);
    }
    sx1278_send_data(packet, strlen((char *)packet));
}
