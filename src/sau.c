/* ntnu.c */
#include <modem/lte_lc.h>
#include <net/cloud.h>
#include <modem/location.h>

#include "sau.h"

void initLte()
{
    int err;
    err = lte_lc_init_and_connect();
    if (err)
    {
        LOG_INF("Error lte_lc_init_and_connect: %d", err);
        err = 0;
    }
}

void setLteModeNbiot()
{
    int err;
    err = lte_lc_system_mode_set(LTE_LC_SYSTEM_MODE_NBIOT_GPS, LTE_LC_SYSTEM_MODE_PREFER_NBIOT);
    if (err)
    {
        LOG_INF("Error lte_lc_system_mode_set: %d", err);
        err = 0;
    }
}

void requestLtePsm()
{
    err = lte_lc_psm_req(true);
    if (err)
    {
        LOG_INF("Error lte_lc_psm_req: %d", err);
        err = 0;
    }
}

void initLocation()
{
    int err;
    err = location_init(location_event_handler_t event_handler);
    if (err)
    {
        printk("Initializing the Location library failed, error: %d\n", err);
        return -1;
    }
}

void cloudInit()
{
    err = cloud_init(cloud_backend, cloud_event_handler);
    if (err)
    {
        LOG_ERR("Cloud backend could not be initialized, error: %d",
                err);
    }
}

void cloudConnect(){
int err;

	if (cloud_connected) {
		return;
	}

	err = cloud_connect(cloud_backend);
	if (err) {
		LOG_ERR("cloud_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds",
	       CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work,
		K_SECONDS(CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

