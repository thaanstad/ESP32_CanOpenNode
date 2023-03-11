/*
CANOpenNode Demo
trevorH 2.19.23

This code demonstrates the use of CANOpenNode on the ESP32 Wrover Processor.  It utilizes the Arduino core for ESP32
provided by espressif: https://github.com/espressif/arduino-esp32

The code utilizes PDO messages to display data with intervals of 1.5seconds and heart beat messages with an interval
of 1 second.  And SDO messages are read repeatedly with large intervals of 100ms and also as quickly as possible for several times
during that read session.

The node IDs for the devices that are tested are 9 and 10 but additional nodes could easily be added with different IDs.  Node
9 will request SDO data from Node 10.

The system utilizes freeRTOS to generate the tasks for implementation of CANOpenNode (as described in their documentation) as well
as a CAN_ctrl_task for setup of the TWAI CAN hardware within the ESP32 and processing messages two and from CANOpenNode.  This
function utilizes to queues for CAN messages that should be received and those that should be transmitted.

Basic error recovery has been implemented for CANtimeouts, errors and disconnection but this should be updated for better robustness

*/

#include <CANopen.h>
#include "OD.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h" // For TWAI_TX_IDX

#define TX_GPIO_NUM                     GPIO_NUM_21
#define RX_GPIO_NUM                     GPIO_NUM_22
#define STBY_GPIO_NUM					GPIO_NUM_26

#define CAN_MSGPRINT_TASK_PRIO              4
#define CAN_MSGGEN_TASK_PRIO                3
#define CAN_CTRL_TASK_PRIO                  1

#define CANOPEN_TASK_PRIO					3
#define CANOPEN_TMR_TASK_PRIO				4
#define CANOPEN_INTERUPT_TASK_PRIO			2
#define CANOPEN_SDO_TASK_PRIO               9

#define STARTUP_TAG					 "STARTUP"
#define TWAI_TAG                     "TWAI"
#define CANMSGPRINT_TAG				 "CAN_PRINT"
#define CANMSGGEN_TAG				 "CAN_GEN"
#define CANCTRL_TAG					 "CAN_CTRL"
#define CANOPEN_TAG                  "CANOPEN"

#define SERIALBAUD      1000000

//need a device ID of 9 and 10 on the bus
#define CO_NODE_ID 10

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_general_config_t g_config = { .mode = TWAI_MODE_NORMAL, .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,        \
																	.clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      \
																	.tx_queue_len = 20, .rx_queue_len = 20,                           \
																	.alerts_enabled = TWAI_ALERT_NONE,  .clkout_divider = 0,        \
																	.intr_flags = ESP_INTR_FLAG_LEVEL1 };
static  twai_message_t tx_msg = { .extd = 1, .identifier = 0xAABB, .data_length_code = 8,
											.data = {0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7} };
typedef struct CANMessages {
	twai_message_t message;
} TX_Messages, RX_Messages;

static SemaphoreHandle_t CAN_ctrl_task_sem;

static const int CAN_TX_queue_len = 10;     // Size of TX_queue
static const int CAN_RX_queue_len = 10;     // Size of RX_queue

QueueHandle_t CAN_TX_queue;
QueueHandle_t CAN_RX_queue;

/* default values for CO_CANopenInit() */
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 1000
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL
/* Global variables and objects */
CO_t* CO = NULL; /* CANopen object */
// Global variables
uint32_t time_old, time_current;
CO_ReturnError_t err;


//This function sets up the CAN bus using ESP32 TWAI and provides some error handling for CAN bus errors.  TX and RX queues
//are used to receive and transmit data from the CAN controller in the ESP32
static void CAN_ctrl_task(void* arg)
{
	CANMessages tx_msg;
	twai_message_t rx_message;
	xSemaphoreTake(CAN_ctrl_task_sem, portMAX_DELAY);


	//Install TWAI driver
	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(TWAI_TAG, "Driver installed");
	ESP_ERROR_CHECK(twai_start());
	vTaskDelay(pdMS_TO_TICKS(50));
	ESP_LOGI(TWAI_TAG, "Driver started");

	pinMode(STBY_GPIO_NUM, OUTPUT);
	digitalWrite(STBY_GPIO_NUM, LOW);

	//Prepare to trigger errors, reconfigure alerts to detect change in error state
	twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

	uint32_t alerts;
	uint32_t alertsPrev;

	//clear out any old messages on bus
	unsigned long currentTime = millis();
	while ((millis() - currentTime) < 100)
	{
		twai_read_alerts(&alerts, 0);
		if (alerts & TWAI_ALERT_RX_DATA)
		{
			twai_receive(&rx_message, 0);
		}
		vTaskDelay(1);
	}

	while (1) {
		twai_read_alerts(&alerts, 0);
		if (alertsPrev != alerts)
		{
			//ESP_LOGI(TWAI_TAG, "%X", alerts);
			alertsPrev = alerts;
		}
		if (alerts & TWAI_ALERT_RX_DATA)
		{
			while (twai_receive(&rx_message, 0) == ESP_OK) {
				//Process received messages.  It is important this is read till empty since this
				//only seems to report when there is initially data in RX buffer
				
				//ESP_LOGI(TWAI_TAG, "CAN received");
				if (uxQueueSpacesAvailable(CAN_RX_queue))
				{
					xQueueSend(CAN_RX_queue, (void*)&rx_message, 10);
				}
				else
				{
					xQueueSend(CAN_RX_queue, (void*)&rx_message, 10);
					ESP_LOGE(TWAI_TAG, "RX Queue full");
				}
			}
		}

		if (xQueueReceive(CAN_TX_queue, (void*)&tx_msg, 0) == pdTRUE)
		{
			//ESP_LOGI(TWAI_TAG, "transmitting...");
			if (twai_transmit(&tx_msg.message, 0) != ESP_OK) {
				ESP_LOGE(TWAI_TAG, "TX_ERROR_INVLD");
			}
		}


		if (alerts & TWAI_ALERT_ERR_ACTIVE)
		{
			ESP_LOGI(TWAI_TAG, "Entered Error Active");
		}
		if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
			ESP_LOGI(TWAI_TAG, "Surpassed Error Warning Limit");
		}
		if (alerts & TWAI_ALERT_ERR_PASS) {
			ESP_LOGI(TWAI_TAG, "Entered Error Passive");
		}
		if (alerts & TWAI_ALERT_BUS_OFF || alerts & TWAI_ALERT_BUS_ERROR) {
			ESP_LOGI(TWAI_TAG, "Bus Off/Error state");
			//Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
			for (int i = 10; i > 0; i--) {
				ESP_LOGW(TWAI_TAG, "Initiate bus recovery in %d", i);
				vTaskDelay(pdMS_TO_TICKS(10));
			}
			ESP_LOGI(TWAI_TAG, "Initiate bus recovery");
			twai_initiate_recovery();
			ESP_LOGI(TWAI_TAG, "TWAI STOP");
			twai_stop();
			ESP_LOGI(TWAI_TAG, "Driver Uninstall");
			twai_driver_uninstall();
			ESP_LOGI(TWAI_TAG, "Driver install");
			twai_driver_install(&g_config, &t_config, &f_config);
			ESP_LOGI(TWAI_TAG, "TWAI Start");
			twai_start();
			ESP_LOGI(TWAI_TAG, "Completed Bus recovery");
			twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
		}
		if (alerts & TWAI_ALERT_BUS_RECOVERED) {
			//Bus recovery was successful
			ESP_LOGI(TWAI_TAG, "Bus Recovered");
		}
		vTaskDelay(1);
	}
	xSemaphoreGive(CAN_ctrl_task_sem);
	vTaskDelete(NULL);
}

/*
Tasks for generating and consuming messages (by printing to the terminal are provided for reference

static void CAN_msg_gen_task(void* arg)
{
	CANMessages msg;
	while (1)
	{
		msg.message.identifier = 0xBABC;
		msg.message.data_length_code = 8;
		msg.message.data[0] = 0x01;
		msg.message.data[1] = 0x02;
		msg.message.data[2] = 0x03;
		msg.message.data[3] = 0x04;
		msg.message.data[4] = 0x05;
		msg.message.data[5] = 0x06;
		msg.message.data[6] = 0x07;
		msg.message.data[7] = 0x08;
		msg.message.extd = 1;
		msg.message.rtr = 0;
		msg.message.ss = 0;
		msg.message.self = 0;
		msg.message.dlc_non_comp = 1;

		xQueueSend(CAN_TX_queue, (void*)&msg, 5);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}

static void CAN_msg_print_task(void* arg)
{
	CANMessages rx_msg;
	while (1)
	{
		if (xQueueReceive(CAN_RX_queue, (void*)&rx_msg, 0) == pdTRUE)
		{
			if (rx_msg.message.flags & TWAI_MSG_FLAG_EXTD) {
				//printf("Message is in Extended Format\n");
			}
			else {
				//printf("Message is in Standard Format\n");
			}
			if (!(rx_msg.message.flags & TWAI_MSG_FLAG_RTR)) {
				uint64_t msgData = 0;
				for (int i = 0; i < rx_msg.message.data_length_code; i++) {
					msgData |= ((uint64_t)rx_msg.message.data[i] << (8 * i));
				}
				ESP_LOGI(CANMSGPRINT_TAG, "%lu,CAN:%X,%llX", millis(), rx_msg.message.identifier, msgData);
			}
		}
		vTaskDelay(1);
	}
	vTaskDelete(NULL);
}

*/



//Function for reading SDOs provided by CANOpenNode within CO_SDOclient.h.
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId,
	uint16_t index, uint8_t subIndex,
	uint8_t* buf, size_t bufSize, size_t* readSize)
{
	CO_SDO_return_t SDO_ret;
	const unsigned int retryAttempts = 5;

	// setup client (this can be skipped, if remote device don't change)
	SDO_ret = CO_SDOclient_setup(SDO_C,
		CO_CAN_ID_SDO_CLI + nodeId,
		CO_CAN_ID_SDO_SRV + nodeId,
		nodeId);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_GENERAL;
	}



	// initiate upload
	SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_GENERAL;
	}

	//ESP_LOGI("CANSDO", "SDO init");
	// upload data

	do {
		uint32_t timeDifference_us = 1000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientUpload(SDO_C,
			timeDifference_us,
			false,
			&abortCode,
			NULL, NULL, NULL);

		if (SDO_ret < 0) {
			CO_SDOclientClose(SDO_C);
			ESP_LOGE("CANSDO_Read", "Abort %d", SDO_ret);
			return abortCode;
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	} while (SDO_ret > 0);

	// copy data to the user buffer (for long data function must be called
	// several times inside the loop)
	*readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
	CO_SDOclientClose(SDO_C);
	return CO_SDO_AB_NONE;
}

//Function for writing SDOs provided by CANOpenNode within CO_SDOclient.h.  There is the potential for this function
//to hang if there is not a reply from the SDO server so a simple timemout procedure has also been implemented.
CO_SDO_abortCode_t write_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId,
	uint16_t index, uint8_t subIndex,
	uint8_t* data, size_t dataSize)
{
	CO_SDO_return_t SDO_ret;
	bool_t bufferPartial = false;

	// setup client (this can be skipped, if remote device is the same)
	SDO_ret = CO_SDOclient_setup(SDO_C,
		CO_CAN_ID_SDO_CLI + nodeId,
		CO_CAN_ID_SDO_SRV + nodeId,
		nodeId);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_NO_DATA;
	}

	// initiate download
	SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex,
		dataSize, 1000, false);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_NO_DATA;
	}

	// fill data
	size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
	if (nWritten < dataSize) {
		bufferPartial = true;
		// If SDO Fifo buffer is too small, data can be refilled in the loop.
	}

	//download data
	do {
		uint32_t timeDifference_us = 1000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientDownload(SDO_C,
			timeDifference_us,
			false,
			bufferPartial,
			&abortCode,
			NULL, NULL);

		if (SDO_ret < 0) {
			ESP_LOGE("CANSDO_Read", "Abort %d", SDO_ret);
			return abortCode;
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	} while (SDO_ret > 0);

	CO_SDOclientClose(SDO_C);
	return CO_SDO_AB_NONE;
}


//CO_main is also provided by CANopenNode in example documentation for basic implementation of CANopenNode.  This function
//has been modified to start the CO_tmr and CO_interrupt tasks that handle processing of messages and receiving of messages
static void CO_main(void* arg) {
	CO_ReturnError_t err;
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	uint32_t heapMemoryUsed;
	void* CANptr = NULL; /* CAN module address */
	uint8_t pendingNodeId = CO_NODE_ID; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
	uint8_t activeNodeId = CO_NODE_ID; /* Copied from CO_pendingNodeId in the communication reset section */
	uint16_t pendingBitRate = 500;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */


	/* Allocate memory */
	CO_config_t* config_ptr = NULL;
	CO = CO_new(config_ptr, &heapMemoryUsed);
	if (CO == NULL) {
		ESP_LOGI(CANOPEN_TAG, "Error: Can't allocate memory");
		return;
	}
	else {
		ESP_LOGI(CANOPEN_TAG, "Allocated %u bytes for CANopen objects", heapMemoryUsed);
	}


	while (reset != CO_RESET_APP) {
		/* CANopen communication reset - initialize CANopen objects *******************/
		ESP_LOGI(CANOPEN_TAG, "CANopenNode - Reset communication...");

		/* Wait rt_thread. */
		CO->CANmodule->CANnormal = false;

		/* Enter CAN configuration. */
		CO_CANsetConfigurationMode((void*)&CANptr);
		CO_CANmodule_disable(CO->CANmodule);

		/* initialize CANopen */
		err = CO_CANinit(CO, CANptr, pendingBitRate);

		if (err != CO_ERROR_NO) {
			ESP_LOGI(CANOPEN_TAG, "Error: CAN initialization failed: %d", err);
			return;
		}

		CO_LSS_address_t lssAddress = { .identity = {
			.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
			.productCode = OD_PERSIST_COMM.x1018_identity.productCode,
			.revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
			.serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
		} };
		err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
		if (err != CO_ERROR_NO) {
			ESP_LOGI(CANOPEN_TAG, "Error: LSS slave initialization failed: %d", err);
			return;
		}

		activeNodeId = pendingNodeId;
		uint32_t errInfo = 0;


		err = CO_CANopenInit(CO,								/* CANopen object */
			NULL,												/* alternate NMT */
			NULL,												/* alternate em */
			OD,													/* Object dictionary */
			OD_STATUS_BITS,										/* Optional OD_statusBits */
			static_cast <CO_NMT_control_t> (NMT_CONTROL),       /* CO_NMT_control_t */
			FIRST_HB_TIME,										/* firstHBTime_ms */
			SDO_SRV_TIMEOUT_TIME,								/* SDOserverTimeoutTime_ms */
			SDO_CLI_TIMEOUT_TIME,								/* SDOclientTimeoutTime_ms */
			SDO_CLI_BLOCK,										/* SDOclientBlockTransfer */
			activeNodeId,
			&errInfo);
		if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
			if (err == CO_ERROR_OD_PARAMETERS) {
				ESP_LOGI(CANOPEN_TAG, "Error: Object Dictionary entry 0x%X", errInfo);
			}
			else {
				ESP_LOGI(CANOPEN_TAG, "Error: CANopen initialization failed: %d", err);
			}
			return;
		}

		err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
		if (err != CO_ERROR_NO) {
			if (err == CO_ERROR_OD_PARAMETERS) {
				ESP_LOGI(CANOPEN_TAG, "Error: Object Dictionary entry 0x%X", errInfo);
			}
			else {
				ESP_LOGI(CANOPEN_TAG, "Error: PDO initialization failed: %d", err);
			}
			return;
		}

		/* Configure Timer interrupt function for execution every 1 millisecond */
		xTaskCreatePinnedToCore(CO_tmrTask, "CO_TMR", 4096, NULL, CANOPEN_TMR_TASK_PRIO, NULL, tskNO_AFFINITY);
		xTaskCreatePinnedToCore(CO_interruptTask, "CO_INT", 4096, NULL, CANOPEN_INTERUPT_TASK_PRIO, NULL, tskNO_AFFINITY);

		/* Configure CAN transmit and receive interrupt */
		// Configured in individual CAN CTRL tasks


		/* Configure CANopen callbacks, etc */
		if (!CO->nodeIdUnconfigured) {
		}
		else {
			ESP_LOGI(CANOPEN_TAG, "CANopenNode - Node-id not initialized");
		}


		/* start CAN */
		CO_CANsetNormalMode(CO->CANmodule);

		reset = CO_RESET_NOT;

		ESP_LOGI(CANOPEN_TAG, "CANopenNode - Running...");
		fflush(stdout);


		while (reset == CO_RESET_NOT) {
			/* loop for normal program execution ******************************************/
						/* get time difference since last function call */
			uint32_t timeDifference_us = 1000;

			/* CANopen process */
			reset = CO_process(CO, false, timeDifference_us, NULL);
			//LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
			//LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

			/* Nonblocking application code may go here. */

			/* Process automatic storage */

			/* optional sleep for short time */
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
	/* program exit ***************************************************************/
		/* stop threads */


		/* delete objects from memory */
	CO_CANsetConfigurationMode((void*)&CANptr);
	CO_delete(CO);

	ESP_LOGI(CANOPEN_TAG, "CANopenNode finished");
	vTaskDelete(NULL);
	/* reset */
}

//Timer thread executes in constant intervals and processes CANOpenNode messages
static void CO_tmrTask(void* arg) {
	static unsigned long prevTime = 0;
	for (;;) {
		if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
			bool_t syncWas = false;
			/* get time difference since last function call */
			uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
			syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
			CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
			CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

			/* Further I/O or nonblocking application code may go here. */
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	vTaskDelete(NULL);
}

//
static void CO_interruptTask(void* arg) {
	for (;;)
	{
		CO_CANinterrupt(CO->CANmodule);
		vTaskDelay(1);
	}

	vTaskDelete(NULL);
}

static void CO_SDOTask(void* arg) {
	unsigned long pastTime = 0;
	uint32_t counter = 0;
	bool count = false;
	for (;;)
	{
		unsigned long currentTime = millis();
		if ((currentTime - pastTime) > 100)
		{
			pastTime = currentTime;
			counter++;
			uint8_t result[4];
			size_t readSize;

			result[0] = (counter & 0x000000ff);
			result[1] = (counter & 0x0000ff00) >> 8;
			result[2] = (counter & 0x00ff0000) >> 16;
			result[3] = (counter & 0xff000000) >> 24;

			//ESP_LOGI("SDO_DAT", "start");

			//write_SDO(CO->SDOclient, 9, 0x6000, 0x0, result, 4);
			if (count)
			{
				read_SDO(CO->SDOclient, 9, 0x6000, 0x0, result, 4, &readSize);
				count = !count;
			}
			else
			{
				read_SDO(CO->SDOclient, 9, 0x6001, 0x0, result, 4, &readSize);
				count = !count;
			}
			//read_SDO(CO->SDOclient, 9, 0x6001, 0x0, result, 4, &readSize);
			//read_SDO(CO->SDOclient, 9, 0x6000, 0x0, result, 4, &readSize);
			//read_SDO(CO->SDOclient, 9, 0x6001, 0x0, result, 4, &readSize);
			//read_SDO(CO->SDOclient, 9, 0x6000, 0x0, result, 4, &readSize);


			ESP_LOGI("SDO_DAT", "%d", result[0]);
		}


		vTaskDelay(pdMS_TO_TICKS(1));
	}

	vTaskDelay(pdMS_TO_TICKS(100));
	vTaskDelete(NULL);
}

void setup()
{
	//delay(8000);
	Serial.begin(SERIALBAUD);
	ESP_LOGI(STARTUP_TAG, "BOOTING, Compiled " __DATE__ " " __TIME__);

	CAN_ctrl_task_sem = xSemaphoreCreateBinary();
	// Create CAN RX and TX queues
	CAN_TX_queue = xQueueCreate(CAN_TX_queue_len, sizeof(TX_Messages));
	CAN_RX_queue = xQueueCreate(CAN_RX_queue_len, sizeof(RX_Messages));


	//xTaskCreatePinnedToCore(CAN_msg_print_task, "MSG_print", 4096, NULL, CAN_MSGPRINT_TASK_PRIO, NULL, tskNO_AFFINITY);
	//xTaskCreatePinnedToCore(CAN_msg_gen_task, "MSG_gen", 4096, NULL, CAN_MSGGEN_TASK_PRIO, NULL, tskNO_AFFINITY);
	xSemaphoreGive(CAN_ctrl_task_sem);
	xTaskCreatePinnedToCore(CAN_ctrl_task, "TWAI_ctrl", 4096, NULL, CAN_CTRL_TASK_PRIO, NULL, tskNO_AFFINITY);
	vTaskDelay(pdMS_TO_TICKS(100));
	xTaskCreatePinnedToCore(CO_main, "CO_MAIN", 6000, NULL, CANOPEN_TASK_PRIO, NULL, tskNO_AFFINITY);
#if CO_NODE_ID == 10
	vTaskDelay(pdMS_TO_TICKS(500));
	xTaskCreatePinnedToCore(CO_SDOTask, "CO_SDO", 4096, NULL, CANOPEN_SDO_TASK_PRIO, NULL, tskNO_AFFINITY);
#endif

	vTaskDelay(pdMS_TO_TICKS(100));
}

//Seperate thread started by FreeRTOS.  This loop() accesses Node9 data by writing and reading SDO messages
void loop()
{
	unsigned long currentTime = millis();
	static unsigned long pastTime = 0;
	static const unsigned long interval = 1000;
	static uint8_t NMT_opState = 0;
	uint32_t val = 0;

	if ((currentTime - pastTime) > interval)
	{
		pastTime = currentTime;
		CO_LOCK_OD(CO->CANmodule);
		OD_get_u32(OD_find(OD, 0x6000), 0x00, &val, false);
		val++;
		OD_set_u32(OD_find(OD, 0x6000), 0x00, val, false);
		//ESP_LOGI(CANOPEN_TAG, "%d", OD_PERSIST_COMM.x6001_counterFromNode);
		CO_UNLOCK_OD(CO->CANmodule);
	}

	if (NMT_opState != CO->NMT->operatingState)
	{
		NMT_opState = CO->NMT->operatingState;
		ESP_LOGI(CANOPEN_TAG, "%d", CO->NMT->operatingState);
	}

	vTaskDelay(pdMS_TO_TICKS(100));
}
