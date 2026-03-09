/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_drv_ppi.h"
#include "nrf_sdh_soc.h"
#include "nrf_drv_timer.h"
#include "nrf_sdh_ble.h"
#include "nrf_drv_timer.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_uart.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "nrf_sdm.h"
#include "pstorage.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

//#define DEVICE_NAME                     "METER_READER_1"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                100                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                6000                                        /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                4096                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                4096                                         /**< UART RX buffer size. */



#define PUSH_BUTTON_DELAY	        3000 
#define PUSH_BUTTON_DELAY_TICKS_OFF	        APP_TIMER_TICKS(PUSH_BUTTON_DELAY)
#define PUSH_BUTTON_DELAY_1	        1000
#define PUSH_BUTTON_DELAY_TICKS_ONN	        APP_TIMER_TICKS(PUSH_BUTTON_DELAY_1)



static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);


/*adc requirements*/
#define SAMPLES_IN_BUFFER 5
#define ADV_TIME_BUFF   5
#define BAUDRATE_SIZE 10
#define APP_ADV_NAME   "METER_READER"
#define APP_ADV_TIME   "5"

static uint32_t              m_adc_evt_counter;
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;


#define ADV_NAME_BLOCK_SIZE	32
uint8_t app_Adv_Name[ADV_NAME_BLOCK_SIZE];
uint8_t adc_sampling_done = 0;
uint16_t adc_bat_buffer[10];
 uint8_t delyidel2_5min = 0;
 uint8_t delyidel2_5min1 = 5;
uint8_t my_meter = 0;
uint8_t baud300 = 0;
uint8_t baud600 = 0;
/*ended*/

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);     

                                            /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
static void advertising_init(void);

APP_TIMER_DEF(m_pow_off_timer_id);

#define TIMER_MAX_VALUE	0x00FFFFFF


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */

 static void send_Response_Data(uint8_t *buf,uint16_t length);
 void change_uart_baud(uint32_t new_baud);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) app_Adv_Name,
                                          strlen(app_Adv_Name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        adc_sampling_done = 1;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        memset(adc_bat_buffer, 0, sizeof(adc_bat_buffer));
        memcpy(adc_bat_buffer, p_event->data.done.p_buffer, sizeof(adc_bat_buffer));


        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        //for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        //{
        //    NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
        //}
        m_adc_evt_counter++;
    }
}
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}




void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,NRF_TIMER_CC_CHANNEL0,ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

//void print_softdevice_ram(void)
//{
//uint32_t ram_start = 0x20000000;
////uint32_t err_code = sd_softdevice_enable(NULL, &ram_start);

//    NRF_LOG_INFO("SoftDevice expected RAM start: 0x%08X", ram_start);
//    APP_ERROR_CHECK(err_code);
//}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
#define RX_BUF_SIZE	      2048


uint8_t cmd_buffer[RX_BUF_SIZE];
uint8_t CmdStart = 0;
uint8_t cmd_length = 0;
uint8_t cmd_index = 0;
uint8_t NoOfWrongCmd = 0;
uint8_t SomethingWoring = 0;

volatile uint16_t UART_TX_FLAG = 0;
volatile uint16_t UART_RX_FLAG = 0;
volatile uint32_t itr1 = 0;
volatile uint32_t itr2 = 0;

#define FLASH_HEADER	"DED"
#define FLASH_HEADER_LENGTH	4
#define BYTE_ALLIGNMENT(x)	((x % 4) == 0 ? x :(x + (4 - (x % 4))))

uint8_t flash_update;
uint8_t firmware_version_flag = 0;
uint8_t firmware_version[5] = "0.0.1";
uint8_t ble_Adv_Name[ADV_NAME_BLOCK_SIZE];
uint8_t ble_Adv_Duration[ADV_TIME_BUFF];
uint8_t requested_baudrate[BAUDRATE_SIZE];

pstorage_handle_t       				flash_handle;
pstorage_handle_t                                       flash_handle2;
uint8_t flag = 0; 

static void flash_event_handler(pstorage_handle_t *handle,uint8_t op_code,uint32_t result,uint8_t *p_data,uint32_t data_len)
{
	switch(op_code)
	{
		case PSTORAGE_STORE_OP_CODE:
					flag = 1;
					break;
		case PSTORAGE_LOAD_OP_CODE:
					flag = 1;
					break;
		case PSTORAGE_CLEAR_OP_CODE:
					flag = 1;
					break;
		case PSTORAGE_UPDATE_OP_CODE:
					break;
	}
}
void nrf_flash_Init(void)
{
	pstorage_module_param_t param;
        pstorage_module_param_t param1;
	uint32_t                retval;
	retval = pstorage_init();

	if(retval == NRF_SUCCESS)
	{
		param.block_size  = ADV_NAME_BLOCK_SIZE;
		param.block_count = 2;
		param.cb          = flash_event_handler;
		retval = pstorage_register(&param, &flash_handle);

                retval = pstorage_block_identifier_get(&flash_handle, 1, &flash_handle2);
                if (retval != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("pstorage_block_identifier_get failed: %d", retval);
                    return;
                }
	}
}



void get_device_name(void)
{
	uint32_t retval;
	uint8_t flash_read_data[4] = {0};
	uint8_t flash_write_data[ADV_NAME_BLOCK_SIZE];memset(flash_write_data,'\0',sizeof(flash_write_data));
	retval = pstorage_load(flash_read_data,&flash_handle,4,0);
	while(flag != 1);
	flag = 0;
	if(retval == NRF_SUCCESS && !memcmp(flash_read_data,FLASH_HEADER,(FLASH_HEADER_LENGTH - 1)))
	{
                memset(app_Adv_Name,'\0',sizeof(ADV_NAME_BLOCK_SIZE));
		retval = pstorage_load(app_Adv_Name,&flash_handle,flash_read_data[3],4);
		while(flag != 1);
		flag = 0;
	}
	else
	{
		memset(flash_write_data,'\0',sizeof(flash_write_data));
		memcpy(flash_write_data,FLASH_HEADER,strlen(FLASH_HEADER));
		flash_write_data[3] = BYTE_ALLIGNMENT(strlen(APP_ADV_NAME));
		memcpy(flash_write_data + FLASH_HEADER_LENGTH ,APP_ADV_NAME,strlen(APP_ADV_NAME));
		pstorage_clear(&flash_handle,ADV_NAME_BLOCK_SIZE);
		while(flag != 1);
		flag = 0;
		retval = pstorage_store(&flash_handle,flash_write_data,flash_write_data[3]+FLASH_HEADER_LENGTH,0);
		while(flag != 1);
		flag = 0;
	}
}

void set_device_name(uint8_t * name,uint8_t length)
{
	uint8_t flash_data[ADV_NAME_BLOCK_SIZE];
	memset(flash_data,'\0',sizeof(flash_data));
	memcpy(flash_data,FLASH_HEADER,strlen(FLASH_HEADER));
	flash_data[3] = BYTE_ALLIGNMENT(length);
	memcpy(flash_data + FLASH_HEADER_LENGTH ,name,length);
	pstorage_clear(&flash_handle,ADV_NAME_BLOCK_SIZE);
	while(flag != 1);
	flag = 0;
	pstorage_store(&flash_handle,flash_data,flash_data[3]+FLASH_HEADER_LENGTH,0);
	while(flag != 1);
	flag = 0;
}

void set_bleadv_duration(uint8_t * name,uint8_t length)
{
        if(length != 1)
        {
          return ;
        }
	uint8_t flash_data[ADV_TIME_BUFF];
	memset(flash_data,'\0',sizeof(flash_data));
	memcpy(flash_data,FLASH_HEADER,strlen(FLASH_HEADER));
	flash_data[3] = BYTE_ALLIGNMENT(length);
	memcpy(flash_data + FLASH_HEADER_LENGTH ,name,length);
        if(flash_data[4] < 0x30)
        {
          return;
        }
        if(flash_data[4] > 0x40)
        {
          return;
        }
	pstorage_clear(&flash_handle2,ADV_NAME_BLOCK_SIZE);
	while(flag != 1);
	flag = 0; 
	pstorage_store(&flash_handle2,flash_data,flash_data[3]+FLASH_HEADER_LENGTH,0);
	while(flag != 1);
	flag = 0;
}

void get_device_advtime(void)
{
	uint32_t retval;
        uint8_t temp = 0;
	uint8_t flash_read_data1[8] = {0};
	uint8_t flash_write_data1[8];
        memset(flash_write_data1,'\0',sizeof(flash_write_data1));
	retval = pstorage_load(flash_read_data1,&flash_handle2,4,0);
	while(flag != 1);
	flag = 0;
	if(retval == NRF_SUCCESS && !memcmp(flash_read_data1,FLASH_HEADER,(FLASH_HEADER_LENGTH - 1)))
	{
                temp = 0;
                retval = pstorage_load(ble_Adv_Duration,&flash_handle2,1,4);
                temp = atoi(&ble_Adv_Duration[0]);
                if(temp >0 && temp < 10)
                {
                  delyidel2_5min1 = temp;
                }
                else
                {
                  delyidel2_5min1 = 5;
                }
		while(flag != 1);
		flag = 0;
	}
	else
	{
		memset(flash_write_data1,'\0',sizeof(flash_write_data1));
		memcpy(flash_write_data1,FLASH_HEADER,strlen(FLASH_HEADER));
		flash_write_data1[3] = BYTE_ALLIGNMENT(strlen(APP_ADV_TIME));
		memcpy(flash_write_data1 + FLASH_HEADER_LENGTH ,APP_ADV_TIME,strlen(APP_ADV_TIME));
		pstorage_clear(&flash_handle2,ADV_NAME_BLOCK_SIZE);
		while(flag != 1);
		flag = 0;
		retval = pstorage_store(&flash_handle2,flash_write_data1,flash_write_data1[3]+FLASH_HEADER_LENGTH,0);
		while(flag != 1);
		flag = 0;
              //  memset(app_Adv_Name,'\0',sizeof(ADV_NAME_BLOCK_SIZE));
                retval = pstorage_load(ble_Adv_Duration,&flash_handle2,1,4);
                temp = atoi(&ble_Adv_Duration[0]);
                if(temp >0 && temp < 10)
                {
                  delyidel2_5min1 = temp;
                }
                else
                {
                  delyidel2_5min1 = 5;
                }
                while(flag != 1);
		flag = 0;
	}
}
#if 0
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
#define CMD_START_CODE 	0x7E
#define CMD_END_CODE	0x7E
#define CMD_STARTED		1
#define CMD_END			0


	int PendingLen = p_evt->params.rx_data.length;
	int cpy_len = 0;
	int prev_copied_len = 0;
	int ignore = 0;

	if(p_evt->params.rx_data.p_data[0] == 0x21 && p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == 0x21)
	{

		memset(ble_Adv_Name,'\0',ADV_NAME_BLOCK_SIZE);
		memcpy(ble_Adv_Name,(p_evt->params.rx_data.p_data +1),(p_evt->params.rx_data.length-2));
		flash_update = 1;
	}
        if(p_evt->params.rx_data.p_data[0] == 0x22 && p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == 0x22)
        { 
              if(p_evt->params.rx_data.length < ADV_TIME_BUFF)
              {
                   memset(ble_Adv_Duration,'\0',ADV_TIME_BUFF);
                   memcpy(ble_Adv_Duration,(p_evt->params.rx_data.p_data +1),(p_evt->params.rx_data.length-2));
                   flash_update = 2;

              }
            
        }
	else
	{

#if 1
		while (PendingLen > 0 && !ignore )
		 {
			 if ( CmdStart != CMD_STARTED) {
				 memset(&cmd_buffer,'\0',RX_BUF_SIZE);
				 cmd_index = 0;
				 cmd_length = p_evt->params.rx_data.p_data[prev_copied_len + 2];
				 CmdStart = CMD_STARTED; // command started
			 }
			 if ( PendingLen > (cmd_length - cmd_index + 2))
					 cpy_len = (cmd_length - cmd_index + 2);
			 else
					 cpy_len = PendingLen;

			 memcpy((cmd_buffer+cmd_index), (p_evt->params.rx_data.p_data + prev_copied_len), cpy_len);

			 prev_copied_len += cpy_len;
			 PendingLen = PendingLen - cpy_len;
			 cmd_index += cpy_len;
			 if(cmd_buffer[0] != CMD_START_CODE) {
					 /* Ignore full command , something wrong */
					 NoOfWrongCmd ++;
					 ignore = 1;
					 CmdStart = CMD_END;
			 }
                          
			 else if ( (cmd_length + 2) == cmd_index/*strlen((char *)cmd_buffer)*/)
			 {
					 /* Found a command , validate and Send to Optical IR */
					 CmdStart = CMD_END; /* Cmd end */
					 ignore = 1;
                                  
					 if(cmd_buffer[(cmd_length + 2)-1] == CMD_END_CODE)
					 {
                                                  nrf_gpio_pin_write(LED_RED,1);
                                                  UART_TX_FLAG = 1;

                                                 for (int i = 0; i < (cmd_length + 2); i++)
                                                 {
                                                             // NRF_LOG_RAW_INFO(" %x",cmd_buffer[i]);
                                                                 while(app_uart_put(cmd_buffer[i]) != NRF_SUCCESS);
                                                 }

                                                 NRF_LOG_INFO("sent command");
					 }
					 else
					 {
						 ignore = 1;
						 NoOfWrongCmd ++;
						/* Wrong command recieved , please ignore it */

					 }
			 }
			 else if (  cmd_index > (cmd_length + 2))
			 {
						 SomethingWoring++;
						 /* OOPs Something wrong , we should not be here, please clear all data */
						 CmdStart = CMD_END; /* Cmd end */
						 ignore =  1;
			 }


		 } /* Loop still Pending data received */
#endif
	}
}
#else

#define SOFT_UART_TX_PIN 6  // Example TX pin
#define SOFT_UART_RX_PIN 5 
#define BAUD_RATE 600

#define BIT_DELAY_US (baud300 ? (1000000 / 300) : (1000000 / 600))


void soft_uart_send_byte(uint8_t data) {
    // Start bit
    nrf_gpio_pin_clear(SOFT_UART_TX_PIN);
    nrf_delay_us(BIT_DELAY_US);

    // Data bits LSB first
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            nrf_gpio_pin_set(SOFT_UART_TX_PIN);
        } else {
            nrf_gpio_pin_clear(SOFT_UART_TX_PIN);
        }
        nrf_delay_us(BIT_DELAY_US);
    }

    // Stop bit
    nrf_gpio_pin_set(SOFT_UART_TX_PIN);
    nrf_delay_us(BIT_DELAY_US);
}

enum
{
  k_cmddevname,
  k_cmdavrtstim, 
  k_cmdfwversion,
  k_cmdbdrchange,
};

typedef struct
{
  uint8_t size;
  const char *cmd_string;
} sidecommand_t;

const char cmddevname[] = "^advn";
const char cmdavrtstim[]  = "^advtim";
const char cmdfwversion[] = "^fwvrsn";
const char cmdbdrchange[] = "^BR=";


const sidecommand_t metercommand[] = 
{
  /* #    size    cmd_string */
  /* 0 */   {5, cmddevname},               
  /* 1 */   {7, cmdavrtstim}, 
  /* 2 */   {7, cmdfwversion},
  /* 3 */   {4, cmdbdrchange},
};


static void nus_data_handler(ble_nus_evt_t * p_evt)
{
     uint32_t baudrate = 0;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        NRF_LOG_FLUSH();
	if(p_evt->params.rx_data.p_data[0] == 0x5E)
	{
           if(strncmp((const char *)p_evt->params.rx_data.p_data, metercommand[k_cmddevname].cmd_string, metercommand[k_cmddevname].size) == 0)
           {

		memset(ble_Adv_Name,'\0',ADV_NAME_BLOCK_SIZE);
		memcpy(ble_Adv_Name,(p_evt->params.rx_data.p_data + metercommand[k_cmddevname].size),(p_evt->params.rx_data.length-(metercommand[k_cmddevname].size+1)));
		flash_update = 1;
            }
            else if(strncmp((const char *)p_evt->params.rx_data.p_data, metercommand[k_cmdavrtstim].cmd_string, metercommand[k_cmdavrtstim].size) == 0)
           {

                   memset(ble_Adv_Duration,'\0',ADV_TIME_BUFF);
                   memcpy(ble_Adv_Duration,(p_evt->params.rx_data.p_data + metercommand[k_cmdavrtstim].size ),(p_evt->params.rx_data.length-(metercommand[k_cmdavrtstim].size+1)));
                   flash_update = 2;
            }
            else if(strncmp((const char *)p_evt->params.rx_data.p_data, metercommand[k_cmdfwversion].cmd_string, metercommand[k_cmdfwversion].size) == 0)
            {
                firmware_version_flag = 1;
            } 
            else if(strncmp((const char *)p_evt->params.rx_data.p_data, metercommand[k_cmdbdrchange].cmd_string, metercommand[k_cmdbdrchange].size) == 0)
            {
                memset(requested_baudrate,'\0',BAUDRATE_SIZE);
                memcpy(requested_baudrate,(p_evt->params.rx_data.p_data + metercommand[k_cmdbdrchange].size ),(p_evt->params.rx_data.length-(metercommand[k_cmdbdrchange].size+1)));
                baudrate = atoi(requested_baudrate);
                NRF_LOG_INFO("baudrate %d",baudrate);
                NRF_LOG_FLUSH();
                change_uart_baud(baudrate);
            }
            else
            {
              memset(p_evt->params.rx_data.p_data,'\0',p_evt->params.rx_data.length);
            }
	}
       else
       {

              for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
              {
                  do
                  {
                      if(baud300 != 1 && baud600 != 1)
                      {
                          UART_TX_FLAG = 1;
                          nrf_gpio_pin_write(LED_RED,1);
                          err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                          if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                          {
                              NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                              APP_ERROR_CHECK(err_code);
                          }
                      }
                      else
                      {
                        UART_TX_FLAG = 1;
                        nrf_gpio_pin_write(LED_RED,1);
                        soft_uart_send_byte(p_evt->params.rx_data.p_data[i]);
                      }
                  } while (err_code == NRF_ERROR_BUSY);
              }
              if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
              {
                  while (app_uart_put('\n') == NRF_ERROR_BUSY);
              }
      }
        
    }

}
#endif
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
#if 1
    uint32_t err_code;
    // = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    NVIC_SystemReset();
    #endif
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code; 

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
             delyidel2_5min++;
             if(delyidel2_5min >= delyidel2_5min1)
             {
               sleep_mode_enter();
               delyidel2_5min = 0;
               break;
             }
             ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            break;
        default:
            break;
    }
}

#if 0
uint8_t data_array[RX_BUF_SIZE];
uint8_t cmd_buffer[RX_BUF_SIZE];
uint16_t ir_index = 0;
uint16_t length = 0;

void uart_event_handle(app_uart_evt_t * p_event)
{
  

	switch (p_event->evt_type)
	{
		case APP_UART_DATA_READY:

			app_uart_get(&data_array[ir_index]);
			ir_index++;

			if(data_array[0] != 0x7E)
			{
				data_array[0] = 0;
				ir_index = 0;
				length = 0;
			}
			else
			{
				if((ir_index == 3) && (data_array[ir_index - 1] <= (RX_BUF_SIZE - 2)))
				{
                                        length = (((data_array[1] & 0x07) << 8) | data_array[2]);
                                        length += 2;
				} 

				if((data_array[0] == 0x7E) && data_array[length - 1] == 0x7E )
				{
					if(length == ir_index)
					{
                                              
						//nrfx_uart_rx_disable();
                                                UART_RX_FLAG = 1;
                                                nrf_gpio_pin_write(LED_YELLOW,1);
						send_Response_Data(data_array,length);
                                                
						memset(&data_array,'\0',RX_BUF_SIZE);
						ir_index = 0;
                                                length =0;
                                                
						//nrfx_uart_rx_enable();
					}
				}
      //                          else
      //                          {
      //                             if((data_array[0] == 0x7E) && (data_array[ir_index - 1] == 0x7E) &&(ir_index != 1))
      //                             {
      //                                         send_Response_Data(data_array,ir_index);
						//memset(&data_array,'\0',RX_BUF_SIZE);
						//ir_index = 0;
      //                             }
                                
                                
      //                          }
			}

			break;

		case APP_UART_COMMUNICATION_ERROR:
                        
			//APP_ERROR_HANDLER(p_event->data.error_communication);
			break;

		case APP_UART_FIFO_ERROR:
			APP_ERROR_HANDLER(p_event->data.error_code);
			break;

		default:
			break;
	}
}
#endif
static uint8_t data_array1[RX_BUF_SIZE];
uint8_t data_received = 0;
//static uint8_t index = 0;
static uint16_t rx_index = 0;
static uint16_t tx_index = 0;

#define CBUFF_RX_INC()  ((rx_index+1) >= RX_BUF_SIZE) ? rx_index=0: rx_index++
#define CBUF_LEN()  (rx_index >= tx_index) ? (rx_index-tx_index): (RX_BUF_SIZE - tx_index)
#define CBUF_TX_INC(l)   ((tx_index + l) >= RX_BUF_SIZE) ? 0 : (tx_index + l)

void uart_event_handle(app_uart_evt_t * p_event)
{

    
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            
            nrf_gpio_pin_write(LED_YELLOW,1);
            UNUSED_VARIABLE(app_uart_get(&data_array1[rx_index]));
            CBUFF_RX_INC();
            data_received = 1;
            break;

        case APP_UART_COMMUNICATION_ERROR:
           // APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            tx_index = 0;
            rx_index = 0;
            delyidel2_5min = 0;
            //itr = 0;
            CmdStart = 0;
            cmd_index = 0;
            cmd_length = 0;
            //cpy_length = 0;
            memset(data_array1,'\0',RX_BUF_SIZE);
            memset(cmd_buffer,'\0',RX_BUF_SIZE);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
 #if 0
static void ble_stack_init(void)
{
    ret_code_t err_code;

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

NRF_LOG_INFO("Suggested RAM start: 0x%X", ram_start);
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
   // uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

#endif
/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
     
    }

}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
static ble_nus_t                        m_nus1;

uint16_t max_mtu_size = 243;
static void send_Response_Data(uint8_t *buf,uint16_t length)
{
 uint32_t       err_code;
       // memset(buf,'\0',length);
        
       //uint8_t buf1[] = {0xAF,0xCC ,0x47,0xC3,0xB1,0xB1,0x30,0x4E,0xB1,0x30,0x2E,0xB1,0xB8,0xD8,0x30,0xB1,0xB1,0x30,0x30,0xB2,0xB7,0xB7,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0x8D,0x0A};
       //length = strlen(buf1);
	uint16_t sent_bytes_count = 0;
        uint16_t length1 = length;

    //   NRF_LOG_INFO("first %d",length);
	while(length > 0)
	{
               
		if(length > max_mtu_size)
		{
			err_code = ble_nus_data_send(&m_nus, buf+sent_bytes_count, &max_mtu_size, m_conn_handle);
                        for(int i =sent_bytes_count;i<= max_mtu_size;i++)
                        {
                          NRF_LOG_RAW_INFO(" %x",buf[i]);
                        }
			sent_bytes_count += max_mtu_size;
			length -= max_mtu_size;


		}
		else
		{
                       length1  = length;
			err_code = ble_nus_data_send(&m_nus, buf+sent_bytes_count, &length1, m_conn_handle);
			
                        for(int i =sent_bytes_count;i< sent_bytes_count+length;i++)
                        {
                            NRF_LOG_RAW_INFO(" %x",buf[i]);
                        }
                        length = 0;
		}
	}
}


/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
#if 1


void soft_uart_init(void) {
      nrf_gpio_cfg_output(SOFT_UART_TX_PIN);
      nrf_gpio_cfg_input(SOFT_UART_RX_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(SOFT_UART_TX_PIN); // idle high

}
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#define HALF_BIT_DELAY_US (BIT_DELAY_US / 2) 

void gpio_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
nrf_drv_gpiote_in_event_disable(SOFT_UART_RX_PIN);
   
    if (pin == SOFT_UART_RX_PIN)
    {
       uint8_t bit = 0;
        uint8_t data = 0;

        // Wait half bit to sample middle of start bit
        nrf_delay_us(HALF_BIT_DELAY_US);

        // Read 8 data bits
        for (uint8_t i = 0; i < 8; i++)
        {
            nrf_delay_us(BIT_DELAY_US);
             bit = nrf_gpio_pin_read(SOFT_UART_RX_PIN);
            data |= (bit << i); // LSB first
        }

        // Wait stop bit (optional)
        nrf_delay_us(BIT_DELAY_US);
        nrf_gpio_pin_write(LED_YELLOW,1);
        data_array1[rx_index] = data;

            CBUFF_RX_INC();

            data_received = 1;
        
            }
            nrf_drv_gpiote_in_event_enable(SOFT_UART_RX_PIN, true);
}

void soft_uart_rx_init(void)
{
    if (!nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_init();

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    nrf_drv_gpiote_in_init(SOFT_UART_RX_PIN, &in_config, gpio_evt_handler);
    nrf_drv_gpiote_in_event_enable(SOFT_UART_RX_PIN, true);
}


void soft_uart_rx_deinit(void)
{
    // Disable event for RX pin
    nrf_drv_gpiote_in_event_disable(SOFT_UART_RX_PIN);

    // Uninitialize GPIOTE input
    nrf_drv_gpiote_in_uninit(SOFT_UART_RX_PIN);

    // Optional: uninitialize GPIOTE driver 
    // (only if your project is not using GPIOTE anywhere else!)
    if (nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_uninit();
}

void soft_uart_deinit(void)
{
    // Put TX pin to default state (input floating)
    nrf_gpio_cfg_input(SOFT_UART_TX_PIN, NRF_GPIO_PIN_NOPULL);

    // Deinit RX side
    soft_uart_rx_deinit();
}

void change_uart_baud(uint32_t new_baud)
{
   uint32_t                     err_code;
   uint8_t ok[] = "ok";
   if(baud300 == 0 && baud600 == 0)
   {
      app_uart_flush();  // flush TX
      app_uart_close();
    }  // uninit UART


    if(new_baud == 300)
    {
      soft_uart_init();
      soft_uart_rx_init();
      itoa(new_baud,ok+2,10);
      send_Response_Data(ok, strlen(ok));
      baud300 = 1;
    }
    else if(new_baud == 600)
    {
     if(baud300 != 1)
     {
            soft_uart_init();
            soft_uart_rx_init();
     }
      itoa(new_baud,ok+2,10);
      send_Response_Data(ok, strlen(ok));
      baud300 = 0;
      baud600 = 1;
    }
    else
    {
    
       if(baud300 == 1 || baud600 == 1)
       {
            NRF_LOG_INFO("INSIDE DEINIT");
            NRF_LOG_FLUSH();
            soft_uart_deinit();
       }
        baud300 = 0;
        baud600 = 0;
        app_uart_comm_params_t comm_params = {
            .rx_pin_no    = RX_PIN_NUMBER,
            .tx_pin_no    = TX_PIN_NUMBER,
            .rts_pin_no   = NRF_UART_PSEL_DISCONNECTED,
            .cts_pin_no   = NRF_UART_PSEL_DISCONNECTED,
            .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
            .use_parity   = false,
            };
           switch (new_baud)
           {
             case 1200 :
                        comm_params.baud_rate    = NRF_UARTE_BAUDRATE_1200;
                        itoa(new_baud,ok+2,10);//  new_baud
                        NRF_LOG_INFO("%s",ok);
                        send_Response_Data(ok, strlen(ok));
                        break;
             case 2400 :
                        comm_params .baud_rate    = NRF_UARTE_BAUDRATE_2400;
                        itoa(new_baud,ok+2,10);
                    
                        NRF_LOG_INFO("SETTED 2400 baudrate");
                        send_Response_Data(ok, strlen(ok));
                        break;
             case 4800 :
                        comm_params.baud_rate    = NRF_UARTE_BAUDRATE_4800;
                        itoa(new_baud,ok+2,10);
                        NRF_LOG_INFO("SETTED 4800 baudrate");
                        send_Response_Data(ok, strlen(ok));
                        break;
             case 9600 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_9600;
                         itoa(new_baud,ok+2,10);
                         NRF_LOG_INFO("SETTED 9600 baudrate");
                         send_Response_Data(ok, strlen(ok));
                         break;
             case 14400:
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_14400;
                         itoa(new_baud,ok+2,10);
                         NRF_LOG_INFO("SETTED 14400 baudrate");
                         send_Response_Data(ok, strlen(ok));
                         break;
             case 19200 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_19200;
                         itoa(new_baud,ok+2,10);
                         NRF_LOG_INFO("SETTED 19200 baudrate");
                         send_Response_Data(ok, strlen(ok));
                         break;
             case 28800 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_28800;
                         NRF_LOG_INFO("SETTED 28800 baudrate");
                         break;
             case 31250 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_31250;
                         NRF_LOG_INFO("SETTED 31250 baudrate");
                         break;
             case 38400 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_38400;
                         NRF_LOG_INFO("SETTED 38400 baudrate");
                         break;
             case 56000 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_56000;
                         NRF_LOG_INFO("SETTED 56000 baudrate");
                         break;
             case 57600 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_57600;
                         NRF_LOG_INFO("SETTED 57600 baudrate");
                       break; 
             case 76800 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_76800;
                         NRF_LOG_INFO("SETTED 76800 baudrate");
                         break;
             case 115200 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_115200;
                         NRF_LOG_INFO("SETTED 115200 baudrate");
                         break;
             case 230400 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_230400;
                         NRF_LOG_INFO("SETTED 230400 baudrate");
                         break;
             case 250000 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_250000;
                         NRF_LOG_INFO("SETTED 250000 baudrate");
                         break;
             case 460800 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_460800;
                         NRF_LOG_INFO("SETTED 460800 baudrate");
                         break;
             case 921600 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_921600;
                         NRF_LOG_INFO("SETTED 921600 baudrate");
                         break;
             case 1000000 :
                         comm_params.baud_rate    = NRF_UARTE_BAUDRATE_1000000;
                         NRF_LOG_INFO("SETTED 1000000 baudrate");
                         break;
            default:
                        comm_params.baud_rate    = NRF_UARTE_BAUDRATE_9600;
                        new_baud = 9600;
                        itoa(new_baud,ok+2,10);
                        send_Response_Data(ok, strlen(ok));
                         break;
       
           }
              baud300 =0;
               APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           _PRIO_APP_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
    
    }   
}
#endif

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = NRF_UART_PSEL_DISCONNECTED,
        .cts_pin_no   = NRF_UART_PSEL_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UARTE_BAUDRATE_9600 
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_9600
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       _PRIO_APP_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


void Rgb_init()
{
	nrf_gpio_cfg_output(LED_RED);
	nrf_gpio_cfg_output(LED_GREEN);
	nrf_gpio_cfg_output(LED_YELLOW);
        nrf_gpio_pin_write(LED_RED,0);
	nrf_gpio_pin_write(LED_GREEN,0);
	nrf_gpio_pin_write(LED_YELLOW,0);
}
void pow_button_config(void)
{
	nrf_gpio_cfg_input(POW_INPUT_PIN,GPIO_PIN_CNF_PULL_Pulldown);
	nrf_gpio_cfg_output(POW_CNTRL_PIN);
	//nrf_gpio_pin_write(POW_CNTRL_PIN,1);
}

uint32_t pow_Input_button_status(void)
{
	uint32_t pinstate;
	pinstate = nrf_gpio_pin_read(POW_INPUT_PIN);
	return pinstate;
}

void start_timer(void)
{
	uint32_t err_code;
	err_code = app_timer_start(m_pow_off_timer_id,TIMER_MAX_VALUE,NULL);
	APP_ERROR_CHECK(err_code);
}

void stop_timer(void)
{
	uint32_t err_code;
	err_code = app_timer_stop(m_pow_off_timer_id);
	APP_ERROR_CHECK(err_code);
}

uint32_t get_timer_tick_cnt(void)
{
	uint32_t ticks;
	ticks = app_timer_cnt_get();
	return ticks;
}

uint32_t get_timer_tick_diff(uint32_t ticks_to,uint32_t ticks_from)
{
	uint32_t ticks_diff;
	ticks_diff = app_timer_cnt_diff_compute(ticks_to,ticks_from);
	return ticks_diff;
}

void pow_button_write(uint32_t value)
{
	nrf_gpio_pin_write(POW_CNTRL_PIN,value);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}
uint16_t avg = 0;
uint16_t get_battery_volatge_adc(void)
{
//adc_bat_buffer
           int i;

    //    NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i <2 ; i++)
        {
      //   NRF_LOG_INFO("buffer i %d %d %d",i, adc_bat_buffer[i],avg);
            avg += adc_bat_buffer[i];

        }
       // NRF_LOG_INFO("bavg %d", avg);
        avg /= 2;
     //   NRF_LOG_INFO("aavg %d", avg);
        if(avg>=70 && avg <= 85)
        {
        avg = 0;
          return 50;
        }
        else if(avg > 85)
        {
        avg = 0;
          return 100;
        }
        else
        {
        avg = 0;
          return 25;
        }

}
static uint8_t itr = 0;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

// BLE observer declaration
NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

static void ble_stack_init(void)
{
    ret_code_t err_code;
    uint32_t ram_start = 0;

    // 1. Enable the SoftDevice
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // 2. Wait until SoftDevice is fully enabled
    if (!nrf_sdh_is_enabled())
    {
        NRF_LOG_WARNING("Waiting for SoftDevice to be enabled...");
        while (!nrf_sdh_is_enabled());
    }

    // 3. Configure default BLE stack settings (GAP, GATT, etc.)
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    if (err_code == NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
    {
        NRF_LOG_ERROR("SoftDevice not enabled yet. Cannot set BLE default configuration.");
        APP_ERROR_CHECK(err_code);
    }
    APP_ERROR_CHECK(err_code);

    // 4. Enable BLE stack
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("BLE stack enabled. RAM starts at 0x%x", ram_start);
}


static void sys_evt_handler(uint32_t sys_evt, void * p_context)
{
    pstorage_sys_event_handler(sys_evt);
}

NRF_SDH_SOC_OBSERVER(sys_observer, 0, sys_evt_handler, NULL);

/**@brief Application main function.
 */
  uint32_t cnt = 0;
 uint8_t iter_tx = 0;
 uint8_t controller_onn = 0;
int main(void)
{

    uint32_t initial_tick = 0;
    uint16_t battery_volatge = 0;
    uint16_t cont0 = 0; 

    bool erase_bonds;

    // Initialize.
    timers_init();
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    uart_init();//same as our code
    log_init();

    buttons_leds_init(&erase_bonds);
    Rgb_init ();
    pow_button_config();
    power_management_init();
    ble_stack_init();

    nrf_flash_Init();
    get_device_name();
    get_device_advtime();
    gap_params_init();
    gatt_init();
    services_init();


    advertising_init();
    conn_params_init();

  
    advertising_start();
    while (1)
    {

#if 1

           NRF_LOG_FLUSH();
           if(data_received == 1)
           {
                uint32_t err_code = 0;
           
                itr1++;
                uint16_t length = (uint16_t)CBUF_LEN() ;
                if((itr1 >= 20000 && length > 0))//||( length > 243 ) )&& length > 0)
                {

                       send_Response_Data(data_array1 + tx_index , length);
                        tx_index= CBUF_TX_INC(length);           
                        length = 0;
                        itr1 = 0;
                        nrf_gpio_pin_write(LED_YELLOW,0);
           
                }
                else if (length == 0)
               {
                       tx_index = 0;
                       rx_index = 0;
                       data_received = 0;
                       memset(data_array1,'\0',RX_BUF_SIZE);
                }

            } 
            if(UART_TX_FLAG >= 1)
            {
                itr1++;

                if(itr1 >= 10000)
                {
                   nrf_gpio_pin_write(LED_RED,0);
                   UART_TX_FLAG = 0;
                   itr1 = 0; 
                }
              
            }

    	if( itr != 0 || (pow_Input_button_status() == 0))
    	{
    		if(itr == 0)
    		{
    			//start timer
    			start_timer();
    			//take the current tick time value
    			initial_tick = get_timer_tick_cnt();
    			itr = 1;
    		}

                if((get_timer_tick_diff(get_timer_tick_cnt(),initial_tick) > PUSH_BUTTON_DELAY_TICKS_ONN)&&(controller_onn == 0))
                {
                     pow_button_write(1);
                     stop_timer ();
                     nrf_gpio_pin_write(LED_GREEN,1);
                     initial_tick = 0;
                     controller_onn = 1;  
                     itr = 0;       
                }
    		if((get_timer_tick_diff(get_timer_tick_cnt(),initial_tick) > PUSH_BUTTON_DELAY_TICKS_OFF)&&(controller_onn == 1))
    		{
    			if((pow_Input_button_status() == 0))
    			{
                            pow_button_write(0);
                            nrf_gpio_pin_write(LED_RED,1);   
                        }                   
    			else
    			{
    				itr = 0;
    			}
    			stop_timer ();
                        controller_onn = 0;
                }
        }

        if (flash_update == 1)
        {
            set_device_name(ble_Adv_Name,strlen((char *)ble_Adv_Name));
            flash_update = 0;
            sd_nvic_SystemReset(); //reset the device
        }
        if(flash_update == 2)
        {
            set_bleadv_duration(ble_Adv_Duration,strlen((char *)ble_Adv_Duration));
            flash_update = 0;
            sd_nvic_SystemReset(); //reset the device

        }
        if(firmware_version_flag == 1)
        {
            send_Response_Data(firmware_version,5);
            firmware_version_flag = 0;
        }
        /*calculating battery and led glowing */
       if(adc_sampling_done == 1)
       {
            battery_volatge = get_battery_volatge_adc();
            adc_sampling_done =0;
            if(battery_volatge == 100)
            {
              nrf_gpio_pin_write(LED_GREEN,1);
            }
            else if(battery_volatge == 50)
            {
              cnt++;
              nrf_gpio_pin_write(LED_GREEN,1);
              if(cnt >= 2)
              {
                nrf_gpio_pin_write(LED_GREEN,0);
                cnt = 0;
              }
            }
            else
            {
              nrf_gpio_pin_write(LED_GREEN,0);
            }

       }
 

    }
}
#endif



/**
 * @}
 */
