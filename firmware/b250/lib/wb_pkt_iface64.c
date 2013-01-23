// Copyright 2012 Ettus Research LLC

#include <wb_pkt_iface64.h>
#include <wb_utils.h>
#include <printf.h>

#define NUM_BYTES_MASK 0x1fff

static uint32_t get_status(wb_pkt_iface64_config_t *config)
{
    return wb_peek32(config->base + 0x1ffc);
}

static void set_control(wb_pkt_iface64_config_t *config)
{
    wb_poke32(config->base + 0x1ffc, config->ctrl);
}

wb_pkt_iface64_config_t wb_pkt_iface64_init(const uint32_t base)
{
    wb_pkt_iface64_config_t config;
    config.base = base;
    config.ctrl = 0;
    set_control(&config);
    return config;
}

void *wb_pkt_iface64_rx_try_claim(wb_pkt_iface64_config_t *config, size_t *num_bytes)
{
    const uint32_t status = get_status(config);
    const uint32_t rx_state_flag = (status >> 31) & 0x1;
    *num_bytes = (status & NUM_BYTES_MASK);
    if (rx_state_flag == 0) return NULL;
    return (void *)config->base;
}

void wb_pkt_iface64_rx_release(wb_pkt_iface64_config_t *config)
{
    config->ctrl |= 1ul << 31; //does a release
    set_control(config);
    while (true)
    {
        const uint32_t status = get_status(config);
        const uint32_t rx_state_flag = (status >> 31) & 0x1;
        if (rx_state_flag == 0)
        {
            config->ctrl &= ~(1ul << 31); //allows for next claim
            set_control(config);
            return;
        }
    }
}

void wb_pkt_iface64_tx_submit(wb_pkt_iface64_config_t *config, const void *buff, const size_t num_bytes)
{
    while (true)
    {
        const uint32_t status = get_status(config);
        const uint32_t tx_state_flag = (status >> 30) & 0x1;
        if (tx_state_flag == 1) break;
    }

    const size_t num_lines = (num_bytes + 3)/4; //round up
    const uint32_t *buff32 = (const uint32_t *)buff;
    for (size_t i = 0; i < num_lines; i++)
    {
        wb_poke32(config->base + i, buff32[i]);
        printf("buff[%u] = 0x%x\n", (unsigned)i, buff32[i]);
    }

    config->ctrl |= (1ul << 30); //allows for next claim
    config->ctrl &= ~(NUM_BYTES_MASK); //clear num bytes
    config->ctrl |= num_bytes & NUM_BYTES_MASK; //set num bytes
    set_control(config);

    //wait for state machine to transition
    while (true)
    {
        const uint32_t status = get_status(config);
        const uint32_t tx_state_flag = (status >> 30) & 0x1;
        if (tx_state_flag == 0) break;
    }

    config->ctrl &= ~(1ul << 30); //release
    set_control(config);

}
