
// Copyright 2013 Ettus Research LLC

#include <link_state_route_proto.h>
#include <u3_net_stack.h>
#include <ethernet.h>
#include <string.h>


/***********************************************************************
 * global constants
 **********************************************************************/
#define LS_PROTO_VERSION 5

//shift the proto version into the ID so only matching fw responds
#define LS_ID_DISCOVER  (0 | (8 << LS_PROTO_VERSION))
#define LS_ID_INFORM    (1 | (8 << LS_PROTO_VERSION))

#define LS_PAYLOAD_MTU 1024
#define LS_MAX_NUM_NBORS 16
#define LS_MAX_TABLE_SIZE 64

/***********************************************************************
 * wire format for table communication
 **********************************************************************/
typedef struct
{
    uint32_t num_nbors;
    struct ip_addr node;
    struct ip_addr nbors[];
} ls_data_t;

static inline size_t sizeof_ls_data(const ls_data_t *ls_data)
{
    return 4/*num neighbors*/ + 4/*source node*/ + 4*ls_data->num_nbors;
}

/***********************************************************************
 * sequence monitor
 **********************************************************************/
static uint16_t current_seq = 0;

static inline bool is_seq_expired(const uint16_t seq)
{
    const uint16_t delta = current_seq - seq;
    return delta > 30; //have not talked in a while, you are deaf to me
}

/***********************************************************************
 * neighbor table
 **********************************************************************/
typedef struct
{
    bool valid;
    uint16_t seq;
    uint8_t ethno;
    struct ip_addr node;
} ls_node_entry_t;

static bool ls_node_entry_valid(const ls_node_entry_t *entry)
{
    return entry->valid && !is_seq_expired(entry->seq);
}

static ls_node_entry_t ls_neighbors[LS_MAX_NUM_NBORS];

static void update_neighbor_entry(const size_t i, const int8_t ethno, const uint16_t seq, const struct ip_addr *node)
{
    ls_neighbors[i].valid = true;
    ls_neighbors[i].seq = seq;
    ls_neighbors[i].ethno = ethno;
    ls_neighbors[i].node.addr = node->addr;
}

static bool register_neighbor(const int8_t ethno, const uint16_t seq, const struct ip_addr *node)
{
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if (!ls_node_entry_valid(&ls_neighbors[i]))
        {
            update_neighbor_entry(i, ethno, seq, node);
            return true;
        }

        if (ls_neighbors[i].node.addr == node->addr && ls_neighbors[i].ethno == ethno)
        {
            const uint16_t reply_delta = current_seq - seq;
            const uint16_t entry_delta = current_seq - ls_neighbors[i].seq;
            if (entry_delta > reply_delta) //reply more recent
            {
                update_neighbor_entry(i, ethno, seq, node);
                return true;
            }
            return false;
        }
    }

    //no space, shift the table down and take entry 0
    memmove(ls_neighbors+1, ls_neighbors, (LS_MAX_NUM_NBORS-1)*sizeof(ls_node_entry_t));
    update_neighbor_entry(0, ethno, seq, node);
    return true;
}

static void send_link_state_data_to_all_neighbors(const uint8_t ethno, const uint16_t seq, const void *buff, const size_t num_bytes)
{
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if (ls_node_entry_valid(&ls_neighbors[i]) && ls_neighbors[i].ethno == ethno)
        {
            u3_net_stack_send_icmp_pkt(
                ethno, ICMP_IRQ, 0,
                LS_ID_INFORM, seq,
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

void update_table_entry(const size_t i, const uint8_t ethno, const uint16_t seq, const ls_data_t *ls_data)
{
    ls_entries[i].node.valid = true;
    ls_entries[i].node.seq = seq;
    ls_entries[i].node.ethno = ethno;
    ls_entries[i].node.node.addr = ls_data->node.addr;
    #define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
    const size_t num = MIN(ls_data->num_nbors, LS_MAX_NUM_NBORS);
    memset(ls_entries[i].nbors, 0, LS_MAX_TABLE_SIZE*sizeof(struct ip_addr));
    memcpy(ls_entries[i].nbors, ls_data->nbors, num*sizeof(struct ip_addr));
}

bool update_table(const uint8_t ethno, const uint16_t seq, const ls_data_t *ls_data)
{
    for (size_t i = 0; i < LS_MAX_TABLE_SIZE; i++)
    {
        if (!ls_node_entry_valid(&ls_entries[i].node))
        {
            update_table_entry(i, ethno, seq, ls_data);
            return true;
        }

        if (ls_entries[i].node.node.addr == ls_data->node.addr && ls_entries[i].node.ethno == ethno)
        {
            const uint16_t reply_delta = current_seq - seq;
            const uint16_t entry_delta = current_seq - ls_entries[i].node.seq;
            if (entry_delta > reply_delta) //reply more recent
            {
                update_table_entry(i, ethno, seq, ls_data);
                return true;
            }
            return false; //old, throw reply out
        }
    }

    //no space, shift the table down and take entry 0
    memmove(ls_entries+1, ls_entries, (LS_MAX_TABLE_SIZE-1)*sizeof(ls_table_entry_t));
    update_table_entry(0, ethno, seq, ls_data);
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
    //received a reply directly from the neighbor, add to neighbor list
    case LS_ID_DISCOVER:
        register_neighbor(ethno, seq, src);
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
    const ls_data_t *ls_data = (const ls_data_t *)buff;

    //ignore packets that originated from this unit
    for (size_t e = 0; e < ethernet_ninterfaces(); e++)
    {
        if (ls_data->node.addr == u3_net_stack_get_ip_addr(e)->addr) return;
    }

    switch (id)
    {
    //replies to discovery packets
    case LS_ID_DISCOVER:
        u3_net_stack_send_icmp_pkt(ethno, ICMP_IR, 0, id, seq, src, buff, num_bytes);
        break;

    //handle information and forward if new
    case LS_ID_INFORM:
        if (update_table(ethno, seq, ls_data))
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
void link_state_route_proto_neighbor_discovery(const uint8_t ethno)
{
    //send a discovery packet
    u3_net_stack_send_icmp_pkt(
        ethno, ICMP_IRQ, 0,
        LS_ID_DISCOVER, current_seq++,
        u3_net_stack_get_bcast(ethno), NULL, 0
    );
}

void link_state_route_proto_flood(const uint8_t ethno)
{
    //fill link state data buffer
    uint8_t buff[LS_PAYLOAD_MTU] = {};
    ls_data_t *ls_data = (ls_data_t *)buff;
    ls_data->node.addr = u3_net_stack_get_ip_addr(ethno)->addr;
    ls_data->num_nbors = 0;
    for (size_t i = 0; i < LS_MAX_NUM_NBORS; i++)
    {
        if ((sizeof_ls_data(ls_data) + 4) >= LS_PAYLOAD_MTU) break;
        if (ls_node_entry_valid(&ls_neighbors[i]) && ls_neighbors[i].ethno == ethno)
        {
            ls_data->nbors[ls_data->num_nbors++].addr = ls_neighbors[i].node.addr;
        }
    }

    //send this data to all neighbors
    send_link_state_data_to_all_neighbors(ethno, current_seq++, ls_data, sizeof_ls_data(ls_data));
}
