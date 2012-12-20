#include "b250_init.h"
#include "b250_defs.h"

int main(void)
{
    b250_init();
    while(1)
    {
        b250_serial_loader_run1();
    }
    return 0;
}
