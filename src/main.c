/*
 * Copyright (c) 2022 Circuit Dojo LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <modem/lte_lc.h>
#include <net/cloud.h>
#include <string.h>
#include <logging/log.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <date_time.h>
#include <net/cloud.h>
#include <net/socket.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "assistance.h"

// Initiates log module. you can then chose which type of log event you want to see (warning, error, info)
LOG_MODULE_REGISTER(SAU, CONFIG_SAU_LOG_LEVEL);

// AGPS related
#if !defined(CONFIG_SAU_ASSISTANCE_NONE)
static struct k_work_q gnss_work_q;
// Allocates stack and priority
#define GNSS_WORKQ_THREAD_STACK_SIZE 2304
#define GNSS_WORKQ_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(gnss_workq_stack_area, GNSS_WORKQ_THREAD_STACK_SIZE);

// agps related variables and threads
static struct nrf_modem_gnss_agps_data_frame last_agps; // For agps data
static struct k_work agps_data_get_work;				// function fot getting data
static volatile bool requesting_assistance;

/* requests agps data if needed*/
static void agps_data_get_work_fn(struct k_work *item)
{
	ARG_UNUSED(item);

	int err;

	requesting_assistance = true;

	printk("Assistance data needed, ephe 0x%08x, alm 0x%08x, flags 0x%02x",
		   last_agps.sv_mask_ephe,
		   last_agps.sv_mask_alm,
		   last_agps.data_flags);

// Following is only if LTE is disconnected between communication
#if defined(CONFIG_SAU_LTE_ON_DEMAND)
	lte_connect();
#endif /* CONFIG_SAU_LTE_ON_DEMAND */

	err = assistance_request(&last_agps);
	if (err)
	{
		printk("Failed to request assistance data\n");
	}

// Following is only if LTE is disconnected between communication (Its not in our application. LTE PSM FTW.)
#if defined(CONFIG_SAU_LTE_ON_DEMAND)
	lte_disconnect();
#endif /* CONFIG_SAU_LTE_ON_DEMAND */

	requesting_assistance = false;
}

#endif //! defined(CONFIG_SAU_ASSISTANCE_NONE)

// Cloud related variables
static struct cloud_backend *cloud_backend;
// static struct k_work_delayable cloud_update_work;
static struct k_work_delayable connect_work;

// Unused, but really cool. Might use in a future version
// static const char update_indicator[] = {'\\', '|', '/', '-'};

// For fix data
static struct nrf_modem_gnss_pvt_data_frame last_pvt;

bool cloud_connected; // Flags if we are connected to the cloud. Used for error handling.

static K_SEM_DEFINE(pvt_data_sem, 0, 1);  // This flags pvt data without a fix
static K_SEM_DEFINE(pvt_fix_sem, 0, 1);	  // This flags pvt data with a fix
static K_SEM_DEFINE(lte_connected, 0, 1); // This flags a successful connection

#define PVT_FIX_SEM 0
#define PVT_DATA_SEM 1

static struct k_poll_event events[2] = {

	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&pvt_fix_sem, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&pvt_data_sem, 0),
};

struct cloud_data
{
	int developer;
	int fix_retry;
	int fix_interval;
	double lat_max;
	double lat_min;
	double lon_max;
	double lon_min;
};

struct json
{
	double longitude;
	double latitude;
	float altitude;
	float accuracy;
	int developer;
};

struct json package;
struct cloud_data from_cloud;

/* Thread for connecting to the cloud. Will always retry if unsuccessful */
static void connect_work_fn(struct k_work *work)
{
	int err;

	if (cloud_connected)
	{
		return;
	}

	err = cloud_connect(cloud_backend);
	if (err)
	{
		printk("cloud_connect error: %d", err);
	}

	printk("Next connection retry in %d seconds",
		   CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work, K_SECONDS(CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

/* Unused thread for sending stuff to the cloud. Kept for reference. */
/*
static void cloud_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected)
	{
		printk("Not connected to cloud, abort cloud publication\n");
		return;
	}

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.buf = CONFIG_CLOUD_MESSAGE,
		.len = strlen(CONFIG_CLOUD_MESSAGE)};
*/
/* When using the nRF Cloud backend data is sent to the message topic.
 * This is in order to visualize the data in the web UI terminal.
 * For Azure IoT Hub and AWS IoT, messages are addressed directly to the
 * device twin (Azure) or device shadow (AWS).
 */
/*
if (strcmp(CONFIG_CLOUD_BACKEND, "NRF_CLOUD\n") == 0)
{
	msg.endpoint.type = CLOUD_EP_MSG;
}
else
{
	msg.endpoint.type = CLOUD_EP_STATE;
}

err = cloud_send(cloud_backend, &msg);
if (err)
{
	printk("cloud_send failed, error: %d", err);
}
}
*/
// Cloud event handler

/* This function analyzes a string sent from the cloud. Its used to changing interval, retry time or enabling developer mode. */
int cloud_message_parser(char *json_from_cloud)
{

	const cJSON *var = NULL;

	cJSON *json = cJSON_Parse(json_from_cloud);
	if (json == NULL)
	{
		printk("empty JSON string in json_parser()");
	}

	var = cJSON_GetObjectItemCaseSensitive(json, "Developer");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives developer update from cloud
		from_cloud.developer = atoi(var->valuestring);
		printk("Developer mode changed to %d", from_cloud.developer);
	}

	var = cJSON_GetObjectItemCaseSensitive(json, "fix_rates");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_rates update from cloud
		from_cloud.fix_retry = atoi(var->valuestring);
		nrf_modem_gnss_fix_retry_set(atoi(var->valuestring));
	}

	var = cJSON_GetObjectItemCaseSensitive(json, "fix_interval");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_interval update from cloud
		from_cloud.fix_interval = atoi(var->valuestring);
		nrf_modem_gnss_fix_interval_set(atoi(var->valuestring));
	}

	var = cJSON_GetObjectItemCaseSensitive(json, "lat_max");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_interval update from cloud
		from_cloud.lat_max = atoi(var->valuestring);
	}
	var = cJSON_GetObjectItemCaseSensitive(json, "lat_min");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_interval update from cloud
		from_cloud.lat_min = atoi(var->valuestring);
	}
	var = cJSON_GetObjectItemCaseSensitive(json, "lon_max");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_interval update from cloud
		from_cloud.lon_max = atoi(var->valuestring);
	}
	var = cJSON_GetObjectItemCaseSensitive(json, "lon_min");
	if (cJSON_IsString(var))
	{ // Add wanted functionality when it receives fix_interval update from cloud
		from_cloud.lon_min = atoi(var->valuestring);
	}

	// printk("Developer from cloud parsed json: %d \n", output);
	cJSON_Delete(json);

	return 1;
}

/* 	This function is used for checking if the sheep is inside a fixed geofence.
	If its still inside, we won't have to send the data.
	Meant for future expansions.	*/
bool in_geofence()
{
	if (package.latitude < from_cloud.lat_max &&
		package.latitude > from_cloud.lat_min &&
		package.longitude < from_cloud.lon_max &&
		package.longitude > from_cloud.lon_min)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* Handler for cloud based events. Used to flag if we are connected, and reconnect if connection is lost. */
void cloud_event_handler(const struct cloud_backend *const backend,
						 const struct cloud_event *const evt,
						 void *user_data)
{
	ARG_UNUSED(user_data);
	ARG_UNUSED(backend);

	switch (evt->type)
	{
	case CLOUD_EVT_CONNECTING:
		printk("CLOUD_EVT_CONNECTING\n");
		break;
	case CLOUD_EVT_CONNECTED:
		printk("CLOUD_EVT_CONNECTED\n");
		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);
		break;
	case CLOUD_EVT_READY:
		printk("CLOUD_EVT_READY\n");

		break;
	case CLOUD_EVT_DISCONNECTED:
		printk("CLOUD_EVT_DISCONNECTED\n");
		cloud_connected = false;
		k_work_reschedule(&connect_work, K_NO_WAIT);
		break;
	case CLOUD_EVT_ERROR:
		printk("CLOUD_EVT_ERROR\n");
		break;
	case CLOUD_EVT_DATA_SENT:
		printk("CLOUD_EVT_DATA_SENT\n");
		break;
	case CLOUD_EVT_DATA_RECEIVED:

		printk("CLOUD_EVT_DATA_RECEIVED\n");
		printk("Data received from cloud: %.*s",
			   evt->data.msg.len,
			   log_strdup(evt->data.msg.buf));
		cloud_message_parser(evt->data.msg.buf); // This cloud message parser does the magic! Acts on given keyword from the cloud.
		break;

	case CLOUD_EVT_PAIR_REQUEST:
		printk("CLOUD_EVT_PAIR_REQUEST\n");
		break;
	case CLOUD_EVT_PAIR_DONE:
		printk("CLOUD_EVT_PAIR_DONE\n");
		break;
	case CLOUD_EVT_FOTA_DONE:
		printk("CLOUD_EVT_FOTA_DONE\n");
		break;
	case CLOUD_EVT_FOTA_ERROR:
		printk("CLOUD_EVT_FOTA_ERROR\n");
		break;
	default:
		printk("Unknown cloud event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	// k_work_init_delayable(&cloud_update_work, cloud_update_work_fn); //Kept for reference
	k_work_init_delayable(&connect_work, connect_work_fn);
}

// GNSS related
/* Handler for modem errors. HOPEFULLY never used */
void nrf_modem_recoverable_error_handler(uint32_t error)
{
	printk("Modem library recoverable error: %u", error);
}

/* GNSS event handler. Flags new pvt-data and valid pvt-data. */
static void gnss_event_handler(int event)
{
	int retval;

	switch (event)
	{
	case NRF_MODEM_GNSS_EVT_PVT:
		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);

		if (retval == 0)
		{
			k_sem_give(&pvt_data_sem);
		}
		break;

	case NRF_MODEM_GNSS_EVT_FIX:

		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);

		if (retval == 0)
		{
			k_sem_give(&pvt_fix_sem); // This flags K_POLL_STATE_SEM_AVAILABLE
		}
		break;
	case NRF_MODEM_GNSS_EVT_NMEA:
		// We chose to not use this, but kept it for future reference.
		/* nmea_data = k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));
		if (nmea_data == NULL)
		{
			printk("Failed to allocate memory for NMEA\n");
			break;
		}

		retval = nrf_modem_gnss_read(nmea_data,
									 sizeof(struct nrf_modem_gnss_nmea_data_frame),
									 NRF_MODEM_GNSS_DATA_NMEA);
		if (retval == 0)
		{
			retval = k_msgq_put(&nmea_queue, &nmea_data, K_NO_WAIT);
			// printk("nmea data was put in nmea queue \n");
		}

		if (retval != 0)
		{
			k_free(nmea_data);
			// printk("nmea data was freed \n");
		}
		*/
		break;

	case NRF_MODEM_GNSS_EVT_AGPS_REQ:
		// case for handling AGPS requests. This case is called when the modem needs a-gps data. We chose to handle it or ignore it based on the konfig.
#if !defined(CONFIG_SAU_ASSISTANCE_NONE)
		retval = nrf_modem_gnss_read(&last_agps,
									 sizeof(last_agps),
									 NRF_MODEM_GNSS_DATA_AGPS_REQ);
		if (retval == 0)
		{
			printk("agps_data_get_work submitted to gnss_work_q\n");
			k_work_submit_to_queue(&gnss_work_q, &agps_data_get_work);
		}
#endif // !CONFIG_SAU_ASSISTANCE_NONE
		break;
	default:
		break;
	}
}
/*  LTE Event handler. This is only used for info when testing. Its not needed in the final product. */
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type)
	{
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
			(evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
		{
			break;
		}

		printk("Network registration status: %s",
			   evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming\n");
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d, Active time: %d",
			   evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE:
	{
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
					   "eDRX parameter update: eDRX: %f, PTW: %f",
					   evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0)
		{
			printk("%s", log_strdup(log_buf));
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s",
			   evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle\n");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d",
			   evt->cell.id, evt->cell.tac);
		break;
	case LTE_LC_EVT_LTE_MODE_UPDATE:
		printk("Active LTE mode changed: %s",
			   evt->lte_mode == LTE_LC_LTE_MODE_NONE ? "None" : evt->lte_mode == LTE_LC_LTE_MODE_LTEM ? "LTE-M"
															: evt->lte_mode == LTE_LC_LTE_MODE_NBIOT  ? "NB-IoT"
																									  : "Unknown\n");
		break;
	case LTE_LC_EVT_MODEM_EVENT:
		printk("Modem domain event, type: %s",
			   evt->modem_evt == LTE_LC_MODEM_EVT_LIGHT_SEARCH_DONE ? "Light search done" : evt->modem_evt == LTE_LC_MODEM_EVT_SEARCH_DONE ? "Search done"
																						: evt->modem_evt == LTE_LC_MODEM_EVT_RESET_LOOP	   ? "Reset loop detected"
																						: evt->modem_evt == LTE_LC_MODEM_EVT_BATTERY_LOW   ? "Low battery"
																						: evt->modem_evt == LTE_LC_MODEM_EVT_OVERHEATED	   ? "Modem is overheated"
																																		   : "Unknown\n");
		break;
	default:
		break;
	}
}

/* Connects to LTE, sets mode,*/
static int modem_init(void)
{
	// Configures the antenna and LNA
	if (strlen(CONFIG_SAU_AT_MAGPIO) > 0)
	{
		if (nrf_modem_at_printf("%s", CONFIG_SAU_AT_MAGPIO) != 0)
		{
			printk("Failed to set MAGPIO configuration\n");
			return -1;
		}
	}
	// Configures the antenna and LNA
	if (strlen(CONFIG_SAU_AT_COEX0) > 0)
	{
		if (nrf_modem_at_printf("%s", CONFIG_SAU_AT_COEX0) != 0)
		{
			printk("Failed to set COEX0 configuration\n");
			return -1;
		}
	}

	int err = lte_lc_modem_events_enable();
	if (err)
	{
		LOG_ERR("lte_lc_modem_events_enable failed, error: %d",
				err);
		return 0;
	}

	lte_lc_init_and_connect_async(lte_handler);

	lte_lc_psm_req(true);	   // Requesting LTM PSM
	if (lte_lc_connect() != 0) // The actual connection
	{
		printk("Failed to connect to LTE network\n");
		return -1;
	}
	printk("Connected to LTE network, psm true \n");

	printk("Modem init done \n");
	return 0;
}

/* Initiates the work queue for the assistance module*/
static int sample_init(void)
{

	int err = 0;
#if !defined(CONFIG_SAU_ASSISTANCE_NONE) // This only happens if we want to use AGPS
	struct k_work_queue_config cfg = {
		.name = "gnss_work_q",
		.no_yield = false};

	k_work_queue_start(
		&gnss_work_q,
		gnss_workq_stack_area,
		K_THREAD_STACK_SIZEOF(gnss_workq_stack_area),
		GNSS_WORKQ_THREAD_PRIORITY,
		&cfg);
	printk("work_queue_start done \n");
	k_work_init(&agps_data_get_work, agps_data_get_work_fn);

	err = assistance_init(&gnss_work_q);
	printk("Assistance init done \n");
#endif /* !CONFIG_SAU_ASSISTANCE_NONE */
	return err;
}

static int gnss_init_and_start(void)
{

	/* Configure GNSS. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0)
	{
		printk("Failed to set GNSS event handler\n");
		return -1;
	}
	printk("Gnss handler set! \n");

	/* Enable all supported NMEA messages. */
	/* // We're not using the nmea masks, but kept if for future reference
	uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
						 NRF_MODEM_GNSS_NMEA_GGA_MASK |
						 NRF_MODEM_GNSS_NMEA_GLL_MASK |
						 NRF_MODEM_GNSS_NMEA_GSA_MASK |
						 NRF_MODEM_GNSS_NMEA_GSV_MASK;

	if (nrf_modem_gnss_nmea_mask_set(nmea_mask) != 0)
	{
		printk("Failed to set GNSS NMEA mask\n");
		return -1;
	}
	printk("NMEA masks set \n");
*/

	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	/*// We can save some data by setting this use case. However, we might be without a-gps data, which would cost more.
	  // This can be used if we have very long time between getting fixes.
		if (!IS_ENABLED(CONFIG_SAU_ASSISTANCE_NONE))
		{
			use_case |= NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;
		}
	*/

	if (IS_ENABLED(CONFIG_SAU_LOW_ACCURACY)) // Allows low accuracy fixes with only 3 satelites
	{
		use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
	}

	if (nrf_modem_gnss_use_case_set(use_case) != 0)
	{
		printk("Failed to set GNSS use case\n");
	}
	else
	{
		printk("Use case set \n");
	}

	uint16_t fix_retry = 180;
	uint16_t fix_interval = 60 * 5;

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0)
	{
		printk("Failed to set GNSS fix retry\n");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0) // Would this make the modem get a new fix every fix_interval minutes
	{
		printk("Failed to set GNSS fix interval\n");
		return -1;
	}
	printk("Fix interval and retry set.");

	if (nrf_modem_gnss_start() != 0)
	{
		printk("Failed to start GNSS\n");
		return -1;
	}
	printk("gnss modem started! \n");

	return 0;
}

/* Prints satellite data; number of healthy satellites.*/
static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked = 0;
	uint8_t in_fix = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i)
	{
		if (pvt_data->sv[i].sv > 0)
		{
			tracked++;
			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX)
			{
				in_fix++;
			}
			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY)
			{
				unhealthy++;
			}
		}
	}

	printk("Tracking: %2d Using: %2d Unhealthy: %d \n", tracked, in_fix, unhealthy);
}

/* Prints location PVT data */
static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printk("Latitude:       %.06f \n", pvt_data->latitude);
	printk("Longitude:      %.06f \n", pvt_data->longitude);
	printk("Altitude:       %.01f m \n", pvt_data->altitude);
	printk("Accuracy:       %.01f m \n", pvt_data->accuracy);
	printk("Speed:          %.01f m/s \n", pvt_data->speed);
	printk("Speed accuracy: %.01f m/s \n", pvt_data->speed_accuracy);
	printk("Heading:        %.01f deg \n", pvt_data->heading);
	printk("Date:           %04u-%02u-%02u \n",
		   pvt_data->datetime.year,
		   pvt_data->datetime.month,
		   pvt_data->datetime.day);
	printk("Time (UTC):     %02u:%02u:%02u.%03u \n",
		   pvt_data->datetime.hour,
		   pvt_data->datetime.minute,
		   pvt_data->datetime.seconds,
		   pvt_data->datetime.ms);
	printk("PDOP:           %.01f \n", pvt_data->pdop);
	printk("HDOP:           %.01f \n", pvt_data->hdop);
	printk("VDOP:           %.01f \n", pvt_data->vdop);
	printk("TDOP:           %.01f \n", pvt_data->tdop);
}

/* Function for transmitting message to the cloud*/
void transmit_to_cloud(char *message)
{
	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE, // B??r evalueres
		.buf = message,
		.len = strlen(message)};

	int err = cloud_send(cloud_backend, &msg);
	if (err)
	{
		printk("cloud_send failed, error: %d", err);
	}
}

/* Function for converting the pvt data to our own package data */
void pvt_to_package(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	package.longitude = pvt_data->longitude;
	package.latitude = pvt_data->latitude;
	package.altitude = pvt_data->altitude;
	package.accuracy = pvt_data->accuracy;
}

/*  */
static void send_struct(struct json package)
{
	char *out;
	char lon_buf[128];
	char lat_buf[128];
	char alt_buf[128];
	char acc_buf[128];
	char dev_buf[128];
	double lon = package.longitude;
	double lat = package.latitude;
	float alt = package.altitude;
	float acc = package.accuracy;
	int dev = package.developer;

	snprintf(lon_buf, sizeof(lon_buf), "%.6f", lon);
	snprintf(lat_buf, sizeof(lat_buf), "%.6f", lat);
	snprintf(alt_buf, sizeof(alt_buf), "%.6f", alt);
	snprintf(acc_buf, sizeof(alt_buf), "%.6f", acc);
	snprintf(dev_buf, sizeof(alt_buf), "%d", dev);

	if (dev == 0)
	{
		printk("Latitude : %s\n", lon_buf);
		printk("Longitude: %s\n", lat_buf);
		printk("Altitude : %s\n", alt_buf);
		printk("DevMode  : %d\n", package.developer);
	}

	cJSON *root;

	// create root node and array
	root = cJSON_CreateObject();

	// add sensor data
	cJSON_AddItemToObject(root, "Latitude", cJSON_CreateString(lon_buf));
	cJSON_AddItemToObject(root, "Longitude", cJSON_CreateString(lat_buf));
	cJSON_AddItemToObject(root, "Altitude", cJSON_CreateString(alt_buf));
	cJSON_AddItemToObject(root, "Accuracy", cJSON_CreateString(acc_buf));
	cJSON_AddItemToObject(root, "Developer", cJSON_CreateString(dev_buf));

	// print everything
	out = cJSON_Print(root);
	if (out == NULL)
	{
		printk("Failed print...\n");
	}
	printk("%s\n", out);

	// CLOUD MAGIC

	transmit_to_cloud(out); // sender JSON til cloud

	// Publish with MQTT

	// data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
	// out, strlen(out));
	cJSON_free(out);
	free(out);

	// free all objects under root and root itself
	cJSON_Delete(root);

	return;
}

void checkForSem(void)
{
	k_poll(events, 2, K_FOREVER); // Looking at semaphores for gnss
	if (events[PVT_DATA_SEM].state == K_POLL_STATE_SEM_AVAILABLE &&
		k_sem_take(events[PVT_DATA_SEM].sem, K_NO_WAIT) == 0) // While searching for fix, this is called every second
	{
		// printk("\033[1;1H\n"); // These two lines clears the console between printing
		// printk("\033[2J\n");
		print_satellite_stats(&last_pvt); // Prints sat stats
	}
	// If there is new, valid PVT data:
	if (events[PVT_FIX_SEM].state == K_POLL_STATE_SEM_AVAILABLE &&
		k_sem_take(events[PVT_FIX_SEM].sem, K_NO_WAIT) == 0)
	{
		printk("Fix available!\n");
		print_fix_data(&last_pvt); // Prints the fix data
								   // if (events[CLOUD_CONNECTED_SEM].state == K_POLL_STATE_SEM_AVAILABLE && k_sem_take(events[CLOUD_CONNECTED_SEM].sem, K_NO_WAIT)) // Nested semchecks! Future is now.
		//{
		if (cloud_connected)
		{
			pvt_to_package(&last_pvt);
			send_struct(package);
			printk("Data sent!");
		}
		else
		{
			printk("Fix available, but we're not connected to the cloud");
		}
	}

	events[PVT_FIX_SEM].state = K_POLL_STATE_NOT_READY;
	events[PVT_DATA_SEM].state = K_POLL_STATE_NOT_READY;
}

void main(void)
{
	work_init(); // Cloud related work fn
	modem_init();
	sample_init(); // Assistance_init is called here
	gnss_init_and_start();

	cloud_backend = cloud_get_binding(CONFIG_CLOUD_BACKEND);
	__ASSERT(cloud_backend != NULL, "%s backend not found",
			 CONFIG_CLOUD_BACKEND);
	int err = cloud_init(cloud_backend, cloud_event_handler);
	if (err)
	{
		printk("Cloud backend could not be initialized, error: %d",
			   err);
	}
//	work_init(); //Why was this double? 

	k_work_schedule(&connect_work, K_NO_WAIT); // This thread connects to the cloud
	for (;;)
	{
		checkForSem(); // Polling function
	}
}
