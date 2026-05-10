#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/multicore.h"

#define IBUS_UART uart1
#define CRSF_UART uart0
#define IBUS_TIMEOUT_US 25000 // If iBUS stalls, output failsafe
#define CRSF_UART_TX_PIN 0
#define IBUS_UART_RX_PIN 5

volatile uint16_t channel_data[16] = {992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992};

uint8_t crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
    }
    return crc;
}

// CORE 1: Handle iBUS Input
void core1_entry()
{
    uint8_t buffer[32];
    while (1)
    {
        if (uart_getc(IBUS_UART) == 0x20)
        {
            buffer[0] = 0x20;
            if (uart_getc(IBUS_UART) == 0x40)
            {
                buffer[1] = 0x40;
                for (int i = 2; i < 32; i++)
                    buffer[i] = uart_getc(IBUS_UART);

                // Checksum validation
                uint16_t sum = 0xFFFF;
                for (int i = 0; i < 30; i++)
                    sum -= buffer[i];
                if (sum == (buffer[30] | (buffer[31] << 8)))
                {
                    for (int i = 0; i < 14; i++)
                    {
                        uint16_t raw = buffer[2 + (i * 2)] | (buffer[3 + (i * 2)] << 8);
                        // Scale 1000-2000 to 172-1811
                        channel_data[i] = ((raw - 1000) * 1639 / 1000) + 172;
                    }
                }
            }
        }
    }
}

// CORE 0: CRSF Output Loop
void send_crsf_packet()
{
    uint8_t frame[25] = {0xEC, 24, 0x16};
    uint8_t *p = &frame[3];
    uint16_t *ch = (uint16_t *)channel_data;

    // Bit packing logic
    uint32_t val[16];
    for (int i = 0; i < 16; i++)
        val[i] = (ch[i] < 172) ? 172 : (ch[i] > 1811 ? 1811 : ch[i]);

    p[0] = (val[0] & 0x07FF) >> 0;
    p[1] = (val[0] >> 8) | (val[1] << 3);
    p[2] = (val[1] >> 5) | (val[2] << 6);
    p[3] = (val[2] >> 2);
    p[4] = (val[2] >> 10) | (val[3] << 1);
    p[5] = (val[3] >> 7) | (val[4] << 4);
    p[6] = (val[4] >> 4) | (val[5] << 7);
    p[7] = (val[5] >> 1);
    p[8] = (val[5] >> 9) | (val[6] << 2);
    p[9] = (val[6] >> 6) | (val[7] << 5);
    p[10] = (val[7] >> 3);
    p[11] = (val[8] & 0x07FF);
    p[12] = (val[8] >> 8) | (val[9] << 3);
    p[13] = (val[9] >> 5) | (val[10] << 6);
    p[14] = (val[10] >> 2);
    p[15] = (val[10] >> 10) | (val[11] << 1);
    p[16] = (val[11] >> 7) | (val[12] << 4);
    p[17] = (val[12] >> 4) | (val[13] << 7);
    p[18] = (val[13] >> 1);
    p[19] = (val[13] >> 9) | (val[14] << 2);
    p[20] = (val[14] >> 6) | (val[15] << 5);
    p[21] = (val[15] >> 3);

    frame[24] = crc8(&frame[2], 23);
    uart_write_blocking(CRSF_UART, frame, 25);
}

int main()
{
    uart_init(IBUS_UART, 115200);
    gpio_set_function(IBUS_UART_RX_PIN, GPIO_FUNC_UART);

    uart_init(CRSF_UART, 420000);
    gpio_set_function(CRSF_UART_TX_PIN, GPIO_FUNC_UART);

    multicore_launch_core1(core1_entry);

    while (1)
    {
        send_crsf_packet();
        sleep_ms(10); // 50Hz update rate
    }
}
