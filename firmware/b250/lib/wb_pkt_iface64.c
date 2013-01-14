// Copyright 2012 Ettus Research LLC

#include <wb_pkt_iface64.h>
#include <wb_utils.h>

static uint32_t get_status(wb_pkt_iface64_config_t *config)
{
    return wb_peek32(config->base);
}

static void set_control(wb_pkt_iface64_config_t *config)
{
    wb_poke32(config->base, config->ctrl);
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
    *num_bytes = (status >> 16) & 0xff;
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
