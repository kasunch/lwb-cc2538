#include <stdio.h>
#include <inttypes.h>

#include "contiki.h"
#include "dev/leds.h"
#include "deployment.h"


PROCESS(deployment_test, "Deployment test");
AUTOSTART_PROCESSES(&deployment_test);

/*------------------------------------------------------------------------------------------------*/
PROCESS_THREAD(deployment_test, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();

  while(1) {
    etimer_set(&et, CLOCK_SECOND * 3);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    deployment_load_ieee_addr();
    deployment_set_node_id_ieee_addr();
    deployment_print_id_info();
  }

  PROCESS_END();
}

