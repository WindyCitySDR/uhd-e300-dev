
// Copyright 2013 Ettus Research LLC

#include <link_state_route_proto.h>
#include <u3_net_stack.h>
#include <string.h>


/***********************************************************************
 * global constants
 **********************************************************************/
#define LS_ID_DISCOVER_BCAST 0
#define LS_ID_DISCOVER_DIRECT 1
#define LS_ID_INFORM_DIRECT 2

#define LS_MAX_NUM_NBORS 8
#define LS_MAX_TABLE_SIZE 64


/***********************************************************************
 * shared structures
 **********************************************************************/
typedef struct
{
    bool valid;
    uint16_t seq;
    struct ip_addr node;
} ls_node_entry_t;

typedef struct
{
    struct ip_addr node;
    struct ip_addr nbors[LS_MAX_NUM_NBORS];
} ls_data_t;


/***********************************************************************
 * sequence monitor
 **********************************************************************/
static uint16_t current_seq = 0;

static bool is_seq_expired(const uint16_t seq)
{
    const uint16_t delta = current_seq - seq;
    return delta > 1024; //basically made up metric, TODO look at this closer
}

/***********************************************************************
 * neighbor table
 **********************************************************************/
static ls_node_entry_t ls_neighbors[LS_MAX_NUM_NBORS];

static void update_neighbor_entry(const size_t i, const uint16_t seq, const struct ip_addr *node)
{
    ls_neighbors[i].valid = true;
    ls_neighbors[i].seq = seq;
    ls_neighbors[i].node.addr = node->addr;
}

static void register_neighbor(const uint16_t seq, const struct ip_addr *node)
{
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if (!ls_neighbors[i].valid || is_seq_expired(ls_neighbors[i].seq))
        {
            update_neighbor_entry(i, seq, node);
            return;
        }

        if (ls_neighbors[i].valid && ls_neighbors[i].node.addr == node->addr)
        {
            const uint16_t reply_delta = current_seq - seq;
            const uint16_t entry_delta = current_seq - ls_neighbors[i].seq;
            if (entry_delta > reply_delta) //reply more recent
            {
                update_neighbor_entry(i, seq, node);
            }
            return;
        }
    }

    //no space, shift the table down and take entry 0
    memmove(ls_neighbors+1, ls_neighbors, (LS_MAX_NUM_NBORS-1)*sizeof(ls_node_entry_t));
    update_neighbor_entry(0, seq, node);
}

static void send_link_state_data_to_all_neighbors(const uint8_t ethno, const uint16_t seq, const void *buff, const size_t num_bytes)
{
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if (ls_neighbors[i].valid && !is_seq_expired(ls_neighbors[i].seq))
        {
            u3_net_stack_send_icmp_pkt(
                ethno,
                ICMP_IRQ, 0,
                LS_ID_INFORM_DIRECT, seq,
                &(ls_neighbors[i].node), buff, num_bytes
            );
        }
    }
}

/***********************************************************************
 * route table structures
 **********************************************************************/
typedef struct
{
    ls_node_entry_t node;
    struct ip_addr nbors[LS_MAX_NUM_NBORS];
} ls_table_entry_t;

static ls_table_entry_t ls_entries[LS_MAX_TABLE_SIZE];

void update_table_entry(const size_t i, const uint16_t seq, const ls_data_t *ls_data)
{
    ls_entries[i].node.valid = true;
    ls_entries[i].node.seq = seq;
    ls_entries[i].node.node.addr = ls_data->node.addr;
    memcpy(ls_entries[i].nbors, ls_data->nbors, LS_MAX_NUM_NBORS*sizeof(struct ip_addr));
}

bool update_table(const uint16_t seq, const void *buff, const size_t num_bytes)
{
    const ls_data_t *ls_data = (const ls_data_t *)buff;

    for (size_t i = 0; i < LS_MAX_TABLE_SIZE; i++)
    {
        if (!ls_entries[i].node.valid || is_seq_expired(ls_entries[i].node.seq))
        {
            update_table_entry(i, seq, ls_data);
            return true;
        }

        if (ls_entries[i].node.valid && ls_entries[i].node.node.addr == ls_data->node.addr)
        {
            const uint16_t reply_delta = current_seq - seq;
            const uint16_t entry_delta = current_seq - ls_entries[i].node.seq;
            if (entry_delta > reply_delta) //reply more recent
            {
                update_table_entry(i, seq, ls_data);
                return true;
            }
            return false; //old, throw reply out
        }
    }

    //no space, shift the table down and take entry 0
    memmove(ls_entries+1, ls_entries, (LS_MAX_TABLE_SIZE-1)*sizeof(ls_table_entry_t));
    update_table_entry(0, seq, ls_data);
    return true;
}

/***********************************************************************
 * handler for information reply
 **********************************************************************/
static void handle_icmp_ir(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t id, const uint16_t seq,
    const void *buff, const size_t num_bytes
){
    switch (id)
    {
    //received a reply to a broadcast discovery, now attempt to talk directly
    case LS_ID_DISCOVER_BCAST:
        u3_net_stack_send_icmp_pkt(
            ethno, ICMP_IRQ, 0, LS_ID_DISCOVER_DIRECT, current_seq++, src, buff, num_bytes
        );
        break;

    //received a reply directly from the neighbor, add to neighbor list
    case LS_ID_DISCOVER_DIRECT:
        register_neighbor(seq, src);
        break;
    }
}

/***********************************************************************
 * handler for information request
 **********************************************************************/
static void handle_icmp_irq(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t id, const uint16_t seq,
    const void *buff, const size_t num_bytes
){
    switch (id)
    {
    //replies to discovery packets
    case LS_ID_DISCOVER_BCAST:
    case LS_ID_DISCOVER_DIRECT:
        u3_net_stack_send_icmp_pkt(
            ethno, ICMP_IR, 0, id, seq, src, buff, num_bytes
        );
        break;

    //handle information and forward if new
    case LS_ID_INFORM_DIRECT:
        if (update_table(seq, buff, num_bytes))
        {
            //table was updated -- send this info to all neighbors
            send_link_state_data_to_all_neighbors(ethno, seq, buff, num_bytes);
        }
        break;
    };
}

/***********************************************************************
 * init and registration code
 **********************************************************************/
void link_state_route_proto_init(void)
{
    u3_net_stack_register_icmp_handler(ICMP_IRQ, 0, &handle_icmp_irq);
    u3_net_stack_register_icmp_handler(ICMP_IR, 0, &handle_icmp_ir);
}

/***********************************************************************
 * initiate a periodic update to the table
 **********************************************************************/
void link_state_route_proto_update(const uint8_t ethno)
{
    //send a discovery packet
    u3_net_stack_send_icmp_pkt(
        ethno,
        ICMP_IRQ, 0,
        LS_ID_DISCOVER_BCAST, current_seq++,
        u3_net_stack_get_bcast(ethno), NULL, 0
    );

    //fill link state data buffer
    ls_data_t ls_data = {};
    ls_data.node.addr = u3_net_stack_get_ip_addr(ethno)->addr;
    size_t nbor_count = 0;
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if (ls_neighbors[i].valid && !is_seq_expired(ls_neighbors[i].seq))
        {
            ls_data.nbors[nbor_count++].addr = ls_neighbors[i].node.addr;
        }
    }

    //send this data to all neighbors
    send_link_state_data_to_all_neighbors(ethno, current_seq++, &ls_data, sizeof(ls_data)); //TODO can be less than sizeof lsdata
}
