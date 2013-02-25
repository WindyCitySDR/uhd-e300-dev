
// Copyright 2012 Ettus Research LLC

#ifndef INCLUDED_B250_INIT_H
#define INCLUDED_B250_INIT_H

void b250_init(void);

void b250_serial_loader_run1(void);

typedef enum
  {
    DCO_156p25,
    DCO_125,
    DCO_10
  } dco_freq_t;

#endif /* INCLUDED_B250_INIT_H */
