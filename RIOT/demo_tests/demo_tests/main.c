
/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdio.h>
#include <inttypes.h>

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
// #include "ble_conn_params.h"
#include "ble_hci.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "board.h"
#include "thread.h"
#include "msg.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "ble-core.h"
#include "net/gnrc/netif.h"
// #include "periph_conf.h"
// #include "bsp.h"
// #include "app_timer.h"

#include "xtimer.h"
#include "board.h"

#include "periph/gpio.h"
#include "debug.h"

//#define EERO
#define BLE_PRIO (GNRC_NETIF_PRIO)

#define RCV_QUEUE_SIZE  (8)
//#define SLEEP (5 * MS_PER_SEC)
#define SLEEP (500 * 1000u)

#define BUT1		13
//#define BUT2		17
//#define BUT3		15
//#define BUT4		19


#define DEVICE_NAME						"neppi"
#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define APP_ADV_ADV_INTERVAL			MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define UPDATE_ACC						1
#define UPDATE_ENERGY						2
//#define ACC_DATA_READY					1
//#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
//#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
//#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
//#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
//#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
//#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
//#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
/*
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0
*/

// Value used as error code on stack dump, can be used to identify stack
// location on stack unwind.
#define DEAD_BEEF                                        0xDEADBEEF
// 128-bit base UUID
#define BLE_UUID_OUR_BASE_UUID                           {{ \
                                                            0x8D, 0x19, 0x7F, \
                                                            0x81, 0x08, 0x08, \
                                                            0x12, 0xE0, 0x2B, \
                                                            0x14, 0x95, 0x71, \
                                                            0x05, 0x06, 0x31, \
                                                            0xB1 \
                                                         }}
// Just a random, but recognizable value
#define BLE_UUID_OUR_SERVICE                             0xABDC
//#define BLE_UUID_OUR_CHARACTERISTC_UUID                  0xBBC9
//#define BLE_UUID_ACCELEROMETER_RAW_CHARACTERISTIC        0xBBCA
//#define BLE_UUID_ACCELEROMETER_MOVEMENT_CHARACTERISTIC   0xBBCB
//#define BLE_UUID_HUMIDITY_CHARACTERISTIC                 0xBBCC
//#define BLE_UUID_RGB_SENSOR_CHARACTERISTIC               0xBBCD
//#define BLE_UUID_BUTTONS_CHARACTERISTIC                  0xBBCE
#define BLE_UUID_ENERGY_CHARACTERISTIC                 	 0xBBCF
#define BLE_UUID_CONTROLS_CHARACTERISTIC                 0xBBD0

static kernel_pid_t main_pid;
static kernel_pid_t ble_thread_pid;

static msg_t rcv_queue[RCV_QUEUE_SIZE];
//static msg_t rcv_queue_main[RCV_QUEUE_SIZE];

// Parameters to be passed to the stack when starting advertising.
// static ble_gap_adv_params_t m_adv_params;
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
// static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

/*
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};
*/
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */

typedef struct
{
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
	uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t	char_handles[2];	/**< Handle of characteristic (as provided by the BLE stack). */
} ble_os_t;

static ble_os_t our_service;

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void add_characteristic(ble_os_t* p_our_service, uint16_t  characteristic, uint8_t char_index);


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void stack_init(void)
{
    uint32_t err_code;

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.attr_tab_size =
        BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed =
        IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);


	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);


	ble_gap_addr_t ble_addr;
	err_code = sd_ble_gap_address_get(&ble_addr);
	APP_ERROR_CHECK(err_code);

	ble_addr.addr[5] = 0x00;
	ble_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

	err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &ble_addr);
	APP_ERROR_CHECK(err_code);
}


static void services_init(void)
{
	ble_os_t* p_our_service = &our_service;

	uint32_t      err_code;
	ble_uuid_t    service_uuid;
	ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;

	service_uuid.uuid = BLE_UUID_OUR_SERVICE;
	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
		&service_uuid,
		&p_our_service->service_handle);
	APP_ERROR_CHECK(err_code);

	p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	
	add_characteristic(p_our_service, BLE_UUID_CONTROLS_CHARACTERISTIC, 0);
        add_characteristic(p_our_service, BLE_UUID_ENERGY_CHARACTERISTIC, 1);
        
}


void add_characteristic(ble_os_t* p_our_service, uint16_t  characteristic, uint8_t char_index)
{
    ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;
    uint32_t      err_code = 0;
    ble_uuid_t    char_uuid;

    char_uuid.uuid = characteristic;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    // Configuring Client Characteristic Configuration Descriptor metadata and
    // add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;
    char_md.char_props.notify = 1;

    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

	// Set characteristic length in number of bytes
    attr_char_value.max_len = 1;
    attr_char_value.init_len = 1;
    uint8_t value[1] = { 0x12 };
    attr_char_value.p_value = value;

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
            &char_md,
            &attr_char_value,
            &p_our_service->char_handles[char_index]);
    APP_ERROR_CHECK(err_code);
}


static void
on_ble_evt(ble_evt_t *p_ble_evt)
{
	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		//ble_gap_addr_print(&(p_ble_evt->evt.gap_evt.params.connected.peer_addr));
		/*
		sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle,
			BLE_GAP_RSSI_THRESHOLD_INVALID,
			0);
		*/
		// m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;'



		break;

	case BLE_GAP_EVT_DISCONNECTED:
		// m_conn_handle = BLE_CONN_HANDLE_INVALID;
		ble_advertising_start();
		break;
	default:
		break;
	}
}

static void on_ble_write(ble_os_t * p_our_service, ble_evt_t * p_ble_evt)
{
	// Declare buffer variable to hold received data. The data can only be 32 bit long.
	uint8_t data_buffer;
	// Populate ble_gatts_value_t structure to hold received data and metadata.
	ble_gatts_value_t rx_data;
	rx_data.len = sizeof(uint8_t);
	rx_data.offset = 0;
	rx_data.p_value = &data_buffer;

	// Check if write event is performed on our characteristic or the CCCD
	for (uint8_t i = 0; i < 2; i++)
	{
		if (p_ble_evt->evt.gatts_evt.params.write.handle ==
		p_our_service->char_handles[i].value_handle)
		{
			// Get data
			sd_ble_gatts_value_get(p_our_service->conn_handle,
		    p_our_service->char_handles[i].value_handle, &rx_data);
			// Print handle and value
			// printf("Value received on handle %#06x: %#010x\r\n",
		    // p_ble_evt->evt.gatts_evt.params.write.handle,
		    // (unsigned int)data_buffer);

			//Pyry was here
			if(((unsigned int)data_buffer) == 0x00000000){LED1_OFF;LED2_OFF;LED3_OFF;}
			else if(((unsigned int)data_buffer) == 0x00000002){LED1_ON;LED2_OFF;LED3_OFF;}
			else if(((unsigned int)data_buffer) == 0x00000003){LED1_OFF;LED2_ON;LED3_OFF;}
			else if(((unsigned int)data_buffer) == 0x00000004){LED1_OFF;LED2_OFF;LED3_ON;}
			puts("Value CCCD recv");
		}
		else if (p_ble_evt->evt.gatts_evt.params.write.handle ==
		p_our_service->char_handles[i].cccd_handle)
		{
			// Get data
			sd_ble_gatts_value_get(p_our_service->conn_handle,
		    p_our_service->char_handles[i].cccd_handle, &rx_data);
			// Print handle and value
			// printf("Value received on handle %#06x: %#06x\r\n",
		//     p_ble_evt->evt.gatts_evt.params.write.handle,
		//     (unsigned int)data_buffer);
			puts("Value recv");
	}
/*
        printf("data_buffer: %lx", data_buffer);
		if (data_buffer == 0x0001)
		{
			printf("Notification enabled\r\n");
		}
		else if (data_buffer == 0x0000)
		{
			printf("Notification disabled\r\n");
		}
*/
	}
}

void ble_our_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt)
{

	//Implement switch case handling BLE events related to our service.
	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GATTS_EVT_WRITE:
		on_ble_write(p_our_service, p_ble_evt);
		break;
	case BLE_GAP_EVT_CONNECTED:
		p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		puts("connected");
		break;
	case BLE_GAP_EVT_DISCONNECTED:
		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
		puts("disconnected");
		break;
	default:
		// No implementation needed.
		break;
	}
}

void acc_characteristic_update(ble_os_t *p_our_service, uint32_t *acc_value, uint8_t char_index)
{
	if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		uint16_t               len = 1;
		ble_gatts_hvx_params_t hvx_params;
		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_our_service->char_handles[char_index].value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &len;
		hvx_params.p_data = (uint8_t*)acc_value;

		sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
	}
	else
	{
		uint16_t          len = 1;
		ble_gatts_value_t tx_data;
		tx_data.len     = len;
		tx_data.offset  = 0;
		tx_data.p_value = (uint8_t*)acc_value;
		sd_ble_gatts_value_set(p_our_service->conn_handle, p_our_service->char_handles[char_index].value_handle, &tx_data);
	}
}

static void
ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
	//ble_conn_params_on_ble_evt(p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_our_service_on_ble_evt(&our_service, p_ble_evt);
}

void *ble_thread(void *arg)
{
	(void)arg;

	//printf("2nd thread started, pid: %" PRIkernel_pid "\n", thread_getpid());
	puts("2nd thread started");
	msg_t m;
	msg_init_queue(rcv_queue, RCV_QUEUE_SIZE);

	stack_init();
	// gap_params_init();
	services_init();
	ble_advertising_init(DEVICE_NAME);

	// Start execution.
	ble_advertising_start();
	while (1) {

		msg_receive(&m);
		switch (m.type)
		{
			case UPDATE_ACC:
				//puts("acc_message receiving");
				acc_characteristic_update(&our_service, &m.content.value, 0);
				puts("acc_message received");
				//printf("%d\n",(int)m.content.value);
				break;
			case UPDATE_ENERGY:
				//puts("acc_message receiving");
				acc_characteristic_update(&our_service, &m.content.value, 1);
				//puts("acc_message received");
				//printf("%d\n",(int)m.content.value);
				break;
			default:
				break;

		}

		/*
		printf("2nd: Got msg from %" PRIkernel_pid "\n", m.sender_pid);
		m.content.value++;
		msg_reply(&m, &m);
		*/
	}

	return NULL;
}

void mpu_interrupt_cb(void *arg)
{
	//msg_t intm;
	//intm.type = ACC_DATA_READY;
	//msg_send(&intm, main_pid);
	//gpio_set(13);
	//LED0_TOGGLE;
}

int main(void)
{
/*
I have altered this part, pray I shall not alter it further
-Pyry
*/
	main_pid = thread_getpid();
	msg_t main_message;
	uint8_t state0 = 1;
	uint8_t state1 = 1;
	uint8_t state2 = 1;
	uint8_t state3 = 1;
	char ble_thread_stack[(THREAD_STACKSIZE_DEFAULT/2)];

	gpio_init(BTN0_PIN, BTN0_MODE);
	gpio_init(BTN1_PIN, BTN1_MODE);
	gpio_init(BTN2_PIN, BTN2_MODE);
	gpio_init(BTN3_PIN, BTN3_MODE);

	ble_thread_pid = thread_create(ble_thread_stack, sizeof(ble_thread_stack),
		BLE_PRIO, 0/*THREAD_CREATE_STACKTEST*/, ble_thread, NULL, "BLE");
	LED0_TOGGLE;
	

    	puts("Entering main loop");
	//msg_init_queue(rcv_queue_main, RCV_QUEUE_SIZE);


    for (;; )
    {
		state0 = gpio_read(BTN0_PIN);
		state1 = gpio_read(BTN1_PIN);
		state2 = gpio_read(BTN2_PIN);
		state3 = gpio_read(BTN3_PIN);
		if(!state0 | !state1 | !state2 | !state3)
		{

			uint8_t dummyC = 0;
			uint8_t dummyE = 0;


			if(!state0){
			dummyC = 0;			
			dummyE = 0;
			
			}
			else if(!state1){

			dummyC = 0x01;			
			dummyE = 0x88;	
			
			}else if(!state2){

			dummyC = 0x02;			
			dummyE = 0xaa;	
			
			}else if(!state3){

			dummyC = 0x03;			
			dummyE = 0xff;	
			
			}
			char *ptr = 0xBBCF;//BLE_UUID_ENERGY_CHARACTERISTIC;

			puts(ptr);
			
			main_message.type = UPDATE_ACC;
			main_message.content.value = dummyC;
			msg_send(&main_message, ble_thread_pid);
			//main_message.type = UPDATE_ENERGY;
			//main_message.content.value = dummyE;
			//msg_send(&main_message, ble_thread_pid);

			//Left here for historical purposes. Rip in pieces.
			//xtimer_usleep(SLEEP);
		}
    }
}


/**
 * @}
 */
