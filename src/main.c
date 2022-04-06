
#include <string.h>
#include <zephyr.h>
#include <nrf_modem_at.h>
#include <modem/lte_lc.h>
#include <modem/location.h>
#include <date_time.h>
#include <net/cloud.h>
#include <net/socket.h>
#include <dk_buttons_and_leds.h>


//--------------- Funksjoner og declares fra location sample
static K_SEM_DEFINE(location_event, 0, 1);

static void antenna_configure(void)
{
	int err;

	if (strlen(CONFIG_LOCATION_SAMPLE_GNSS_AT_MAGPIO) > 0)
	{
		err = nrf_modem_at_printf("%s", CONFIG_LOCATION_SAMPLE_GNSS_AT_MAGPIO);
		if (err)
		{
			printk("Failed to set MAGPIO configuration\n");
		}
	}

	if (strlen(CONFIG_LOCATION_SAMPLE_GNSS_AT_COEX0) > 0)
	{
		err = nrf_modem_at_printf("%s", CONFIG_LOCATION_SAMPLE_GNSS_AT_COEX0);
		if (err)
		{
			printk("Failed to set COEX0 configuration\n");
		}
	}
}

static K_SEM_DEFINE(time_update_finished, 0, 1);

static void date_time_evt_handler(const struct date_time_evt *evt)
{
	k_sem_give(&time_update_finished);
}

static void location_event_handler(const struct location_event_data *event_data)
{
	switch (event_data->id)
	{
	case LOCATION_EVT_LOCATION:
		printk("Got location:\n");
		printk("  method: %s\n", location_method_str(event_data->location.method));
		printk("  latitude: %.06f\n", event_data->location.latitude);
		printk("  longitude: %.06f\n", event_data->location.longitude);
		printk("  accuracy: %.01f m\n", event_data->location.accuracy);
		if (event_data->location.datetime.valid)
		{
			printk("  date: %04d-%02d-%02d\n",
				   event_data->location.datetime.year,
				   event_data->location.datetime.month,
				   event_data->location.datetime.day);
			printk("  time: %02d:%02d:%02d.%03d UTC\n",
				   event_data->location.datetime.hour,
				   event_data->location.datetime.minute,
				   event_data->location.datetime.second,
				   event_data->location.datetime.ms);
		}
		printk("  Google maps URL: https://maps.google.com/?q=%.06f,%.06f\n\n",
			   event_data->location.latitude, event_data->location.longitude);
		break;

	case LOCATION_EVT_TIMEOUT:
		printk("Getting location timed out\n\n");
		break;

	case LOCATION_EVT_ERROR:
		printk("Getting location failed\n\n");
		break;

	case LOCATION_EVT_GNSS_ASSISTANCE_REQUEST:
		printk("Getting location assistance requested. Not doing anything.\n\n");
		break;

	default:
		printk("Getting location: Unknown event\n\n");
		break;
	}

	k_sem_give(&location_event);
}

static void location_event_wait(void)
{
	k_sem_take(&location_event, K_FOREVER);
}

/**
 * @brief Retrieve location so that fallback is applied.
 *
 * @details This is achieved by setting GNSS as first priority method and giving it too short
 * timeout. Then a fallback to next method, which is cellular in this example, occurs.
 */
static void location_with_fallback_get(void)
{
	int err;
	struct location_config config;
	enum location_method methods[] = {LOCATION_METHOD_GNSS, LOCATION_METHOD_CELLULAR};

	location_config_defaults_set(&config, sizeof(methods), methods);
	/* GNSS timeout is set to 1 second to force a failure. */
	config.methods[0].gnss.timeout = 1;
	/* Default cellular configuration may be overridden here. */
	config.methods[1].cellular.timeout = 30;

	printk("Requesting location with short GNSS timeout to trigger fallback to cellular...\n");

	err = location_request(&config);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}

	location_event_wait();
}

/**
 * @brief Retrieve location with default configuration.
 *
 * @details This is achieved by not passing configuration at all to location_request().
 */
static void location_default_get(void)
{
	int err;

	printk("Requesting location with the default configuration...\n");

	err = location_request(NULL);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}

	location_event_wait();
}

/**
 * @brief Retrieve location with GNSS low accuracy.
 */
static void location_gnss_low_accuracy_get(void)
{
	int err;
	struct location_config config;
	enum location_method methods[] = {LOCATION_METHOD_GNSS};

	location_config_defaults_set(&config, sizeof(methods), methods);
	config.methods[0].gnss.accuracy = LOCATION_ACCURACY_LOW;

	printk("Requesting low accuracy GNSS location...\n");

	err = location_request(&config);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}

	location_event_wait();
}

/**
 * @brief Retrieve location with GNSS high accuracy.
 */
static void location_gnss_high_accuracy_get(void)
{
	int err;
	struct location_config config;
	enum location_method methods[] = {LOCATION_METHOD_GNSS};

	location_config_defaults_set(&config, sizeof(methods), methods);
	config.methods[0].gnss.accuracy = LOCATION_ACCURACY_HIGH;

	printk("Requesting high accuracy GNSS location...\n");

	err = location_request(&config);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}

	location_event_wait();
}

#if defined(CONFIG_LOCATION_METHOD_WIFI)
/**
 * @brief Retrieve location with Wi-Fi positioning as first priority, GNSS as second
 * and cellular as third.
 */
static void location_wifi_get(void)
{
	int err;
	struct location_config config;
	enum location_method methods[] = {
		LOCATION_METHOD_WIFI,
		LOCATION_METHOD_GNSS,
		LOCATION_METHOD_CELLULAR};

	location_config_defaults_set(&config, sizeof(methods), methods);

	printk("Requesting Wi-Fi location with GNSS and cellular fallback...\n");

	err = location_request(&config);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}

	location_event_wait();
}
#endif

/**
 * @brief Retrieve location periodically with GNSS as first priority and cellular as second.
 */
static void location_gnss_periodic_get(void)
{
	int err;
	struct location_config config;
	enum location_method methods[] = {LOCATION_METHOD_GNSS, LOCATION_METHOD_CELLULAR};

	location_config_defaults_set(&config, sizeof(methods), methods);
	config.interval = 30;

	printk("Requesting 30s periodic GNSS location with cellular fallback...\n");

	err = location_request(&config);
	if (err)
	{
		printk("Requesting location failed, error: %d\n", err);
		return;
	}
}

//--------------- Funksjoner og declares fra cloud client sample

LOG_MODULE_REGISTER(cloud_client, CONFIG_CLOUD_CLIENT_LOG_LEVEL);

static struct cloud_backend *cloud_backend;
static struct k_work_delayable cloud_update_work;
static struct k_work_delayable connect_work;

static K_SEM_DEFINE(lte_connected, 0, 1);

/* Flag to signify if the cloud client is connected or not connected to cloud,
 * used to abort/allow cloud publications.
 */
static bool cloud_connected;

static void connect_work_fn(struct k_work *work)
{
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

static void cloud_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected) {
		LOG_INF("Not connected to cloud, abort cloud publication");
		return;
	}

	LOG_INF("Publishing message: %s", log_strdup(CONFIG_CLOUD_MESSAGE));

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.buf = CONFIG_CLOUD_MESSAGE,
		.len = strlen(CONFIG_CLOUD_MESSAGE)
	};

	/* When using the nRF Cloud backend data is sent to the message topic.
	 * This is in order to visualize the data in the web UI terminal.
	 * For Azure IoT Hub and AWS IoT, messages are addressed directly to the
	 * device twin (Azure) or device shadow (AWS).
	 */
	if (strcmp(CONFIG_CLOUD_BACKEND, "NRF_CLOUD") == 0) {
		msg.endpoint.type = CLOUD_EP_MSG;
	} else {
		msg.endpoint.type = CLOUD_EP_STATE;
	}

	err = cloud_send(cloud_backend, &msg);
	if (err) {
		LOG_ERR("cloud_send failed, error: %d", err);
	}

#if defined(CONFIG_CLOUD_PUBLICATION_SEQUENTIAL)
	k_work_schedule(&cloud_update_work,
			K_SECONDS(CONFIG_CLOUD_MESSAGE_PUBLICATION_INTERVAL));
#endif
}

void cloud_event_handler(const struct cloud_backend *const backend,
			 const struct cloud_event *const evt,
			 void *user_data)
{
	ARG_UNUSED(user_data);
	ARG_UNUSED(backend);

	switch (evt->type) {
	case CLOUD_EVT_CONNECTING:
		LOG_INF("CLOUD_EVT_CONNECTING");
		break;
	case CLOUD_EVT_CONNECTED:
		LOG_INF("CLOUD_EVT_CONNECTED");
		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);
		break;
	case CLOUD_EVT_READY:
		LOG_INF("CLOUD_EVT_READY");
#if defined(CONFIG_CLOUD_PUBLICATION_SEQUENTIAL)
		k_work_reschedule(&cloud_update_work, K_NO_WAIT);
#endif
		break;
	case CLOUD_EVT_DISCONNECTED:
		LOG_INF("CLOUD_EVT_DISCONNECTED");
		cloud_connected = false;
		k_work_reschedule(&connect_work, K_NO_WAIT);
		break;
	case CLOUD_EVT_ERROR:
		LOG_INF("CLOUD_EVT_ERROR");
		break;
	case CLOUD_EVT_DATA_SENT:
		LOG_INF("CLOUD_EVT_DATA_SENT");
		break;
	case CLOUD_EVT_DATA_RECEIVED:
		LOG_INF("CLOUD_EVT_DATA_RECEIVED");
		LOG_INF("Data received from cloud: %.*s",
			evt->data.msg.len,
			log_strdup(evt->data.msg.buf));
		break;
	case CLOUD_EVT_PAIR_REQUEST:
		LOG_INF("CLOUD_EVT_PAIR_REQUEST");
		break;
	case CLOUD_EVT_PAIR_DONE:
		LOG_INF("CLOUD_EVT_PAIR_DONE");
		break;
	case CLOUD_EVT_FOTA_DONE:
		LOG_INF("CLOUD_EVT_FOTA_DONE");
		break;
	case CLOUD_EVT_FOTA_ERROR:
		LOG_INF("CLOUD_EVT_FOTA_ERROR");
		break;
	default:
		LOG_INF("Unknown cloud event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_work_init_delayable(&cloud_update_work, cloud_update_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		LOG_INF("Network registration status: %s",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_DBG("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_DBG("%s", log_strdup(log_buf));
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_DBG("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		LOG_DBG("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);
		break;
	case LTE_LC_EVT_LTE_MODE_UPDATE:
		LOG_INF("Active LTE mode changed: %s",
			evt->lte_mode == LTE_LC_LTE_MODE_NONE ? "None" :
			evt->lte_mode == LTE_LC_LTE_MODE_LTEM ? "LTE-M" :
			evt->lte_mode == LTE_LC_LTE_MODE_NBIOT ? "NB-IoT" :
			"Unknown");
		break;
	case LTE_LC_EVT_MODEM_EVENT:
		LOG_INF("Modem domain event, type: %s",
			evt->modem_evt == LTE_LC_MODEM_EVT_LIGHT_SEARCH_DONE ?
				"Light search done" :
			evt->modem_evt == LTE_LC_MODEM_EVT_SEARCH_DONE ?
				"Search done" :
			evt->modem_evt == LTE_LC_MODEM_EVT_RESET_LOOP ?
				"Reset loop detected" :
			evt->modem_evt == LTE_LC_MODEM_EVT_BATTERY_LOW ?
				"Low battery" :
			evt->modem_evt == LTE_LC_MODEM_EVT_OVERHEATED ?
				"Modem is overheated" :
				"Unknown");
		break;
	default:
		break;
	}
}

static void modem_configure(void)
{
#if defined(CONFIG_NRF_MODEM_LIB)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already configured and LTE connected. */
	} else {
		int err;

#if defined(CONFIG_POWER_SAVING_MODE_ENABLE)
		/* Requesting PSM before connecting allows the modem to inform
		 * the network about our wish for certain PSM configuration
		 * already in the connection procedure instead of in a separate
		 * request after the connection is in place, which may be
		 * rejected in some networks.
		 */
		err = lte_lc_psm_req(true);
		if (err) {
			LOG_ERR("Failed to set PSM parameters, error: %d",
				err);
		}

		LOG_INF("PSM mode requested");
#endif

		err = lte_lc_modem_events_enable();
		if (err) {
			LOG_ERR("lte_lc_modem_events_enable failed, error: %d",
				err);
			return;
		}

		err = lte_lc_init_and_connect_async(lte_handler);
		if (err) {
			LOG_ERR("Modem could not be configured, error: %d",
				err);
			return;
		}

		/* Check LTE events of type LTE_LC_EVT_NW_REG_STATUS in
		 * lte_handler() to determine when the LTE link is up.
		 */
	}
#endif
}

#if defined(CONFIG_CLOUD_PUBLICATION_BUTTON_PRESS)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	if (has_changed & button_states & DK_BTN1_MSK) {
		k_work_reschedule(&cloud_update_work, K_NO_WAIT);
	}
}
#endif




//---------------------- Eget

enum systemStates
{
	SET_FENCE, //Send inn terskel fra databasen
	SHEEP_IN_FENCE,
	SHEEP_OUT_OF_FENCE,
	SHEEP_DEAD,
	FIND_SHEEP
} systemState;

int main(void)
{
	// Setup :
	
	{ // loop
		switch (state)
		{
		SET_FENCE:
			//hent GPS-data, sett fence utifra hvor man er. +/- en gitt lon og lat.

		SHEEP_IN_FENCE:
			//Hent GPS-data. Først med P-GPS; deretter fallback til LTE.
			//Set opp NB-IoT connection
			//Send data til server
			//Etterspør evt tilstandsendring

			//Dette skal skje 4 gang i døgnet

		SHEEP_OUT_OF_FENCE:
			//Hent GPS-data. A-gps. 
			//Sett opp NB-IoT connection
			//Send data til server
			//Etterspør evt tilstandsendring

			//Dette skal skje 4 gang i timen

		SHEEP_DEAD:
			//Sett opp NB-IoT connection
			//Etterspør tilstandsendring

			//Dette skal skje en gang i døgnet

		FIND_SHEEP:
			//Set opp LTE-M connection
			//Continous tracking, AGPS.

		default:
			//Hent GPS-data. A-gps. 
			//Sett opp NB-IoT connection
			//Send data til server
			//Etterspør evt tilstandsendring


		}// End switch
			
	} // End main()
}

/* Sample code from "location"
int err;

printk("Location sample started\n\n");

antenna_configure();

if (IS_ENABLED(CONFIG_DATE_TIME)) {
	// Registering early for date_time event handler to avoid missing
	// the first event after LTE is connected.
	//
	date_time_register_handler(date_time_evt_handler);
}

printk("Connecting to LTE...\n");

lte_lc_init();
// Enable PSM.
lte_lc_psm_req(true);
lte_lc_connect();

printk("Connected to LTE\n");

// A-GPS/P-GPS needs to know the current time.
if (IS_ENABLED(CONFIG_DATE_TIME)) {
	printk("Waiting for current time\n");

	//Wait for an event from the Date Time library.
	k_sem_take(&time_update_finished, K_MINUTES(10));

	if (!date_time_is_valid()) {
		printk("Failed to get current time. Continuing anyway.\n");
	}
}

err = location_init(location_event_handler);
if (err) {
	printk("Initializing the Location library failed, error: %d\n", err);
	return -1;
}

// The fallback case is run first, otherwise GNSS might get a fix even with a 1 second
// timeout.

location_with_fallback_get();

location_default_get();

location_gnss_low_accuracy_get();

location_gnss_high_accuracy_get();

#if defined(CONFIG_LOCATION_METHOD_WIFI)
location_wifi_get();
#endif

location_gnss_periodic_get();

return 0;

}
*/
// End example code