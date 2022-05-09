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
#include <cJSON.h>
#include "assistance.h"

// Initiates log module. you can then chose which type of log event you want to see (warning, error, info)
LOG_MODULE_REGISTER(SAU, 3);

// creates the gnss_work_q variable of k_work_q structure
static struct k_work_q gnss_work_q;

// Cloud related variables
static struct cloud_backend *cloud_backend;
static struct k_work_delayable cloud_update_work;
static struct k_work_delayable connect_work;

// Allocates stack and priority
#define GNSS_WORKQ_THREAD_STACK_SIZE 2304
#define GNSS_WORKQ_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(gnss_workq_stack_area, GNSS_WORKQ_THREAD_STACK_SIZE);

// agps related variables and threads
static struct nrf_modem_gnss_agps_data_frame last_agps; // For agps data
static struct k_work agps_data_get_work;				// function fot getting data
static volatile bool requesting_assistance;

// Unused, but really cool. Might use in a future version
// static const char update_indicator[] = {'\\', '|', '/', '-'};

// For fix data
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static uint64_t fix_timestamp;

bool cloud_connected; // Flags if we are connected to the cloud. Used for error handling.

K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);

static K_SEM_DEFINE(time_sem, 0, 1);
static K_SEM_DEFINE(lte_ready, 0, 1);
static K_SEM_DEFINE(pvt_data_sem, 0, 1);  // This flags pvt data without a fix
static K_SEM_DEFINE(pvt_fix_sem, 0, 1);	  // This flags pvt data with a fix
static K_SEM_DEFINE(lte_connected, 0, 1); // This flags a successful connection

static struct k_poll_event events[3] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&pvt_data_sem, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&nmea_queue, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&pvt_fix_sem, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
									K_POLL_MODE_NOTIFY_ONLY,
									&lte_connected, 0),
};

// To assert the build.
BUILD_ASSERT(IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_GPS) ||
				 IS_ENABLED(CONFIG_LTE_NETWORK_MODE_NBIOT_GPS) ||
				 IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS),
			 "CONFIG_LTE_NETWORK_MODE_LTE_M_GPS, "
			 "CONFIG_LTE_NETWORK_MODE_NBIOT_GPS or "
			 "CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS must be enabled");

// Cloud related
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
		printk("cloud_connect, error: %d", err);
	}

	printk("Next connection retry in %d seconds",
			CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work,
					K_SECONDS(CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void cloud_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected)
	{
		printk("Not connected to cloud, abort cloud publication");
		return;
	}

	printk("Publishing message: %s", log_strdup(CONFIG_CLOUD_MESSAGE));

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.buf = CONFIG_CLOUD_MESSAGE,
		.len = strlen(CONFIG_CLOUD_MESSAGE)};

	/* When using the nRF Cloud backend data is sent to the message topic.
	 * This is in order to visualize the data in the web UI terminal.
	 * For Azure IoT Hub and AWS IoT, messages are addressed directly to the
	 * device twin (Azure) or device shadow (AWS).
	 */
	if (strcmp(CONFIG_CLOUD_BACKEND, "NRF_CLOUD") == 0)
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
	/*
	#if defined(CONFIG_CLOUD_PUBLICATION_SEQUENTIAL)
		k_work_schedule(&cloud_update_work,
				K_SECONDS(CONFIG_CLOUD_MESSAGE_PUBLICATION_INTERVAL));
	#endif
	*/
}

// Cloud event handler

void cloud_event_handler(const struct cloud_backend *const backend,
						 const struct cloud_event *const evt,
						 void *user_data)
{
	ARG_UNUSED(user_data);
	ARG_UNUSED(backend);

	switch (evt->type)
	{
	case CLOUD_EVT_CONNECTING:
		printk("CLOUD_EVT_CONNECTING");
		break;
	case CLOUD_EVT_CONNECTED:
		printk("CLOUD_EVT_CONNECTED");
		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);
		break;
	case CLOUD_EVT_READY:
		printk("CLOUD_EVT_READY");
#if defined(CONFIG_CLOUD_PUBLICATION_SEQUENTIAL)
		k_work_reschedule(&cloud_update_work, K_NO_WAIT);
#endif
		break;
	case CLOUD_EVT_DISCONNECTED:
		printk("CLOUD_EVT_DISCONNECTED");
		cloud_connected = false;
		k_work_reschedule(&connect_work, K_NO_WAIT);
		break;
	case CLOUD_EVT_ERROR:
		printk("CLOUD_EVT_ERROR");
		break;
	case CLOUD_EVT_DATA_SENT:
		printk("CLOUD_EVT_DATA_SENT");
		break;
	case CLOUD_EVT_DATA_RECEIVED:

		printk("CLOUD_EVT_DATA_RECEIVED");
		printk("Data received from cloud: %.*s",
				evt->data.msg.len,
				log_strdup(evt->data.msg.buf));
		// Her må vi lagre dataen, og agere på den!
		break;

	case CLOUD_EVT_PAIR_REQUEST:
		printk("CLOUD_EVT_PAIR_REQUEST");
		break;
	case CLOUD_EVT_PAIR_DONE:
		printk("CLOUD_EVT_PAIR_DONE");
		break;
	case CLOUD_EVT_FOTA_DONE:
		printk("CLOUD_EVT_FOTA_DONE");
		break;
	case CLOUD_EVT_FOTA_ERROR:
		printk("CLOUD_EVT_FOTA_ERROR");
		break;
	default:
		printk("Unknown cloud event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_work_init_delayable(&cloud_update_work, cloud_update_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
}

// GNSS related
/* Handler for modem errors. HOPEFULLY never used */
void nrf_modem_recoverable_error_handler(uint32_t error)
{
	printk("Modem library recoverable error: %u", error);
}

/* Handler for date time library */
static void date_time_evt_handler(const struct date_time_evt *evt) // How does this work? Where does it get time?
{
	k_sem_give(&time_sem); // Flags that time is set? how?
}

/* GNSS event handler*/
static void gnss_event_handler(int event)
{
	int retval;
	struct nrf_modem_gnss_nmea_data_frame *nmea_data;

	switch (event)
	{
	case NRF_MODEM_GNSS_EVT_PVT:
		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);

		if (retval == 0)
		{
			k_sem_give(&pvt_data_sem); // This flags K_POLL_STATE_SEM_AVAILABLE ?
		}
		break;

	case NRF_MODEM_GNSS_EVT_FIX:

		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);

		if (retval == 0)
		{
			k_sem_give(&pvt_fix_sem); // This flags K_POLL_STATE_SEM_AVAILABLE ?
		}
		break;
	case NRF_MODEM_GNSS_EVT_NMEA:
		// We chose to not use this, but keep it for future reference.
		/* nmea_data = k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));
		if (nmea_data == NULL)
		{
			printk("Failed to allocate memory for NMEA");
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
			printk("agps_data_get_work submitted to gnss_work_q");
			k_work_submit_to_queue(&gnss_work_q, &agps_data_get_work);
		}
#endif // !CONFIG_SAU_ASSISTANCE_NONE

		break;
	default:
		break;
	}
}
// LTE Event handler
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
				evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
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
				evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d",
				evt->cell.id, evt->cell.tac);
		break;
	case LTE_LC_EVT_LTE_MODE_UPDATE:
		printk("Active LTE mode changed: %s",
				evt->lte_mode == LTE_LC_LTE_MODE_NONE ? "None" : evt->lte_mode == LTE_LC_LTE_MODE_LTEM ? "LTE-M"
															 : evt->lte_mode == LTE_LC_LTE_MODE_NBIOT  ? "NB-IoT"
																									   : "Unknown");
		break;
	case LTE_LC_EVT_MODEM_EVENT:
		printk("Modem domain event, type: %s",
				evt->modem_evt == LTE_LC_MODEM_EVT_LIGHT_SEARCH_DONE ? "Light search done" : evt->modem_evt == LTE_LC_MODEM_EVT_SEARCH_DONE ? "Search done"
																						 : evt->modem_evt == LTE_LC_MODEM_EVT_RESET_LOOP	? "Reset loop detected"
																						 : evt->modem_evt == LTE_LC_MODEM_EVT_BATTERY_LOW	? "Low battery"
																						 : evt->modem_evt == LTE_LC_MODEM_EVT_OVERHEATED	? "Modem is overheated"
																																			: "Unknown");
		break;
	default:
		break;
	}
}
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
		printk("Failed to request assistance data");
	}

// Following is only if LTE is disconnected between communication (Its not in our application. LTE PSM FTW.)
#if defined(CONFIG_SAU_LTE_ON_DEMAND)
	lte_disconnect();
#endif /* CONFIG_SAU_LTE_ON_DEMAND */

	requesting_assistance = false;
}

/* Connects to LTE, sets mode, initiates date/time*/
static int modem_init(void)
{
	// Configuring SAU_MAGPOI for nrd9160dk
	if (strlen(CONFIG_SAU_AT_MAGPIO) > 0)
	{
		if (nrf_modem_at_printf("%s", CONFIG_SAU_AT_MAGPIO) != 0) // What are these?
		{
			printk("Failed to set MAGPIO configuration");
			return -1;
		}
	}
	// Configuring SAU_COEX0 for nrd9160dk
	if (strlen(CONFIG_SAU_AT_COEX0) > 0)
	{
		if (nrf_modem_at_printf("%s", CONFIG_SAU_AT_COEX0) != 0) // What are these?
		{
			printk("Failed to set COEX0 configuration");
			return -1;
		}
	}

	if (IS_ENABLED(CONFIG_DATE_TIME))
	{
		date_time_register_handler(date_time_evt_handler);
	}

	if (lte_lc_init() != 0)
	{
		printk("Failed to initialize LTE link controller");
		return -1;
	}
	lte_lc_register_handler(lte_handler); // Using the LTE event handler
	lte_lc_psm_req(true);				  // Requesting LTM PSM
	if (lte_lc_connect() != 0)			  // The actual connection
	{
		printk("Failed to connect to LTE network");
		return -1;
	}
	printk("Connected to LTE network, psm true \n");
	if (IS_ENABLED(CONFIG_DATE_TIME))
	{
		printk("Waiting for current time \n");
		/* Wait for an event from the Date Time library. */
		k_sem_take(&time_sem, K_MINUTES(10));
		if (!date_time_is_valid())
		{
			printk("Failed to get current time, continuing anyway");
		}
	}
	printk("Modem init done \n");
	return 0;
}

/* Initiates the work queue AND the assistance module*/
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

	err = assistance_init(&gnss_work_q); // See the Cmake file from the sample!
	printk("Assistance init done \n");
#endif /* !CONFIG_SAU_ASSISTANCE_NONE */
	return err;
}
// PGPS TING

static int gnss_init_and_start(void)
{
/*#if defined(CONFIG_SAU_ASSISTANCE_NONE)
	// Enable GNSS.
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) // This activates GNSS without changing LTE.
	{
		printk("Failed to activate GNSS functional mode");
		return -1;
	}
endif // CONFIG_SAU_ASSISTANCE_NONE 
*/
	/* Configure GNSS. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0)
	{
		printk("Failed to set GNSS event handler");
		return -1;
	}
	printk("Gnss handler set! \n");

	/* Enable all supported NMEA messages. */
	uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
						 NRF_MODEM_GNSS_NMEA_GGA_MASK |
						 NRF_MODEM_GNSS_NMEA_GLL_MASK |
						 NRF_MODEM_GNSS_NMEA_GSA_MASK |
						 NRF_MODEM_GNSS_NMEA_GSV_MASK;

	if (nrf_modem_gnss_nmea_mask_set(nmea_mask) != 0)
	{
		printk("Failed to set GNSS NMEA mask");
		return -1;
	}
	printk("NMEA masks set \n");

	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	if (IS_ENABLED(CONFIG_SAU_MODE_PERIODIC) &&
		!IS_ENABLED(CONFIG_SAU_ASSISTANCE_NONE))
	{
		/* Disable GNSS scheduled downloads when assistance is used. */ // What does this mean? Why wont we use
		use_case |= NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;
	}

	if (IS_ENABLED(CONFIG_SAU_LOW_ACCURACY)) // We're not using this now/yet, but seems simple enough
	{
		use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
	}

	if (nrf_modem_gnss_use_case_set(use_case) != 0)
	{
		printk("Failed to set GNSS use case");
	}
	printk("Use case set \n");

	uint16_t fix_retry = 180;
	uint16_t fix_interval = 60 * 15;

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0)
	{
		printk("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0) // K_MINUTES(15) why not? Would this make the modem get a new fix every 15 minutes?
	{
		printk("Failed to set GNSS fix interval");
		return -1;
	}
	printk("Set fix reset 180 and fix interval to 15 min \n");

	if (nrf_modem_gnss_start() != 0)
	{
		printk("Failed to start GNSS");
		return -1;
	}
	printk("gnss modem started! \n");

	return 0;
}

/* Function to connect to LTE without interfering with GNSS. Used only if we want to turn off LTE.*/
void lte_connect(void)
{
	int err;

	printk("Connecting to LTE network");

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
	if (err)
	{
		printk("Failed to activate LTE, error: %d", err);
		return;
	}

	k_sem_take(&lte_ready, K_FOREVER);

	/* Wait for a while, because with IPv4v6 PDN the IPv6 activation takes a bit more time. */
	k_sleep(K_SECONDS(1));
}

/* Function to disconnect from LTE without interfering with GNSS. Used only if we want to turn off LTE.*/
void lte_disconnect(void)
{
	int err;

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
	if (err)
	{
		printk("Failed to deactivate LTE, error: %d", err);
		return;
	}

	printk("LTE disconnected");
}

/* This function reads the data from the modem, stores it in &last_pvt, then shuts down the gnss modem.*/
void get_gnss_fix(void)
{
	int err;
	int retval;
	retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
	if (retval == 0)
	{
		k_sem_give(&pvt_data_sem);
		err = nrf_modem_gnss_stop(); // Stops the gnss modem after the fix is pulled from it
	}
	// k_sleep(6 * 3600 * 1000)
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

	printk("Tracking: %2d Using: %2d Unhealthy: %d\n", tracked, in_fix, unhealthy);
}

/* Prints location PVT data*/
static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printk("Latitude:       %.06f\n", pvt_data->latitude);
	printk("Longitude:      %.06f\n", pvt_data->longitude);
	printk("Altitude:       %.01f m\n", pvt_data->altitude);
	printk("Accuracy:       %.01f m\n", pvt_data->accuracy);
	printk("Speed:          %.01f m/s\n", pvt_data->speed);
	printk("Speed accuracy: %.01f m/s\n", pvt_data->speed_accuracy);
	printk("Heading:        %.01f deg\n", pvt_data->heading);
	printk("Date:           %04u-%02u-%02u\n",
		   pvt_data->datetime.year,
		   pvt_data->datetime.month,
		   pvt_data->datetime.day);
	printk("Time (UTC):     %02u:%02u:%02u.%03u\n",
		   pvt_data->datetime.hour,
		   pvt_data->datetime.minute,
		   pvt_data->datetime.seconds,
		   pvt_data->datetime.ms);
	printk("PDOP:           %.01f\n", pvt_data->pdop);
	printk("HDOP:           %.01f\n", pvt_data->hdop);
	printk("VDOP:           %.01f\n", pvt_data->vdop);
	printk("TDOP:           %.01f\n", pvt_data->tdop);
}

void checkForSem(void)
{
	k_poll(events, 3, K_FOREVER); //
	// If there is new PVT data, regardless of its validity
	if (events[0].state == K_POLL_STATE_SEM_AVAILABLE &&
		k_sem_take(events[0].sem, K_NO_WAIT) == 0)
	{
		// printk("\033[1;1H"); // These two lines clears the console between printing
		// printk("\033[2J");
		print_satellite_stats(&last_pvt); // Prints sat stats

		// If there is new, valid PVT data:
		if (events[2].state == K_POLL_STATE_SEM_AVAILABLE &&
			k_sem_take(events[2].sem, K_NO_WAIT) == 0)
		{
			printk("Fix available!");
			print_fix_data(&last_pvt); // Prints the fix data
			//cloud_send(cloud_backend, &last_pvt);
			
									   // Her skal vi sende data!
		}
		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;
		events[2].state = K_POLL_STATE_NOT_READY;
	}
}

int16_t changeIntervalAndRetryTime(int fix_interval, int fix_retry)
{

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0)
	{
		printk("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval * 60) != 0) // K_MINUTES(15) why not? Would this make the modem get a new fix every 15 minutes?
	{
		printk("Failed to set GNSS fix interval");
		return -1;
	}
	printk("Set fix reset to %d sec and fix interval to %d min \n", fix_interval, fix_retry);
	return 0;
}

/* Håvard's experimentelle lekestue */

struct json {
   double longitude;
   double latitude;
   float  altitude;
   float  accuracy;
   int    developer;
};

struct json package;

void transmit_to_cloud(char* message) {

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE, //Bør evalueres
		.buf = message,
		.len = strlen(message)
	};

	
	int err = cloud_send(cloud_backend, &msg);
	if (err) {
		LOG_ERR("cloud_send failed, error: %d", err);
	}
}

/* work in progress

void  pvt_to_package(struct nrf_modem_gnss_pvt_data_frame *pvt_data){
	package.longitude = pvt_data->longitude;
	package.latitude = pvt_data->latitude;
	package.altitude = pvt_data->altitude;
	package.accuracy = pvt_data->accuracy;
}

static void send_struct(struct json package)
{
	char* out;
	char lon_buf[6];
	char lat_buf[6];
	char alt_buf[6];
	char acc_buf[6];
	char dev_buf[1];
	double lon = package.longitude;
	double lat = package.latitude;
	float alt = package.altitude;
	float acc = package.accuracy;
	int dev = package.developer;

	snprintf(lon_buf, sizeof(lon_buf), "%.6f", lon);
	snprintf(lat_buf, sizeof(lat_buf), "%.6f", lat);
	snprintf(alt_buf, sizeof(alt_buf), "%.6f", alt);
	snprintf(acc_buf, sizeof(alt_buf), "%.6f", acc);
	snprintf(dev_buf, sizeof(alt_buf), "%d"  , dev);

	if (dev == 0) {
		printk("Latitude : %s\n", lon_buf);
		printk("Longitude: %s\n", lat_buf);
		printk("Altitude : %s\n", alt_buf);
		printk("DevMode  : %s\n", package.developer);
	}

	cJSON *root;

	//create root node and array
	root = cJSON_CreateObject();	

	//add sensor data 
	cJSON_AddItemToObject(root, "Latitude" , cJSON_CreateString(lon_buf));
	cJSON_AddItemToObject(root, "Longitude", cJSON_CreateString(lat_buf));
	cJSON_AddItemToObject(root, "Altitude" , cJSON_CreateString(alt_buf));
	cJSON_AddItemToObject(root, "Accuracy" , cJSON_CreateString(acc_buf));
	cJSON_AddItemToObject(root, "Developer", cJSON_CreateString(dev_buf));

	// print everything
	out = cJSON_Print(root);
	if (out == NULL) {
		printk("Failed print...\n");
	}
    printk("%s\n", out);

	//CLOUD MAGIC

	transmit_to_cloud(out); //sender JSON til cloud

	// Publish with MQTT

	//data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
   			//out, strlen(out));

	free(out);

	// free all objects under root and root itself
	cJSON_Delete(root);

	return;
}


*/

void main(void)
{
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
	work_init(); // Cloud related work fn 

	for (;;)
	{
		checkForSem(); // Polli'n'g function
		//send_struct(package); // sender json til cloud
		if (cloud_connected) 
		{
			transmit_to_cloud("This in ground control to major Too..rje.");
		}
		
	}
}
