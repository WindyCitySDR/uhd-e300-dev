
// Copyright 2013 Ettus Research LLC

#ifndef INCLUDED_LINK_STATE_ROUTE_PROTO_H
#define INCLUDED_LINK_STATE_ROUTE_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//http://en.wikipedia.org/wiki/Link-state_routing_protocol

//! Initialize internals and handler registration
void link_state_route_proto_init(void);

//! Initiate a periodic update to the table
void link_state_route_proto_update(const uint8_t ethno);

//TODO a way to get the map

#endif /* INCLUDED_LINK_STATE_ROUTE_PROTO_H */
