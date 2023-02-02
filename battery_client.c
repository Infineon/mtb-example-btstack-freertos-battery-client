/******************************************************************************
* File Name: battery_client.c
*
* Description: This is the source code for the
*             Battery Client Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/

#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "cyhal.h"
#include "stdio.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "GeneratedSource/cycfg_gap.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "battery_client.h"
#include "wiced_timer.h"
#include "task.h"
#include "app_bt_bonding.h"
#include "app_serial_flash.h"
#include "inttypes.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_battery_client.h"
#include "app_bt_bonding.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
* Macros
********************************************************************************/

/* Battery Client App Timer Timeout in seconds  */
#define BATTERY_CLIENT_APP_TIMEOUT_IN_MS                      1000

/* Battery Client App Fine Timer Timeout in milli seconds  */
#define BATTERY_CLIENT_APP_FINE_TIMEOUT_IN_MS                1
#define GPIO_INTERRUPT_PRIORITY                              4

#define USR_BUTTON_STATE_DOWN       0
#define USR_BUTTON_STATE_UP         1
#define USR_BUTTON_STATE_UNKNOWN    2

#define USR_BUTTON_ACTION_NONE           0
#define USR_BUTTON_ACTION_SCAN           1   // Scan and connect to battery server
#define USR_BUTTON_ACTION_READ           2   // Read the battery level of the server
#define USR_BUTTON_ACTION_NOTIFICATION   3   // Enable/disable battery level notification from server
#define USR_BUTTON_ACTION_BROADCAST      4   // Enable/disable battery broadcast from server

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;  // remote peer device address
    uint32_t  timer_count;                  // timer count used to count the timeout value in seconds
    uint32_t  fine_timer_count;             // fine timer count in milliseconds
    uint16_t  conn_id;                      // connection ID referenced by the stack
    uint8_t   flag_stay_connected;          // stay connected or disconnect after all messages are sent
    uint8_t   battery_level;                // dummy battery level
    uint8_t   usr_button_state;             // button state - down/up
    uint8_t   usr_button_action;            // button action on duration of button pressed
} battery_client_state_t;

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
battery_service_client_peer_info_t  battery_client_app_data;
battery_service_client_app_t battery_client_app_state;
wiced_bool_t is_bas_enabled = WICED_FALSE;
wiced_bool_t is_broadcast_enabled = WICED_FALSE;
static uint8_t battery_service_found  = 0;

/*******************************************************************************
* Variable Definitions
*******************************************************************************/

/* Holds the global state of the battery client application */
static battery_client_state_t battery_client_state;

/* Host information saved in  NVRAM */
host_info_t battery_client_hostinfo;

/* To check if the device has entered pairing mode to connect and bond with a new device */
bool pairing_mode = FALSE;

/* App timer objects */

/* battery_client_second_timer is a periodic timer that ticks every second. Upon
   expiring, the current time in seconds is displayed in the serial terminal */
/* battery_client_ms_timer is a periodic timer that ticks every millsecond. This
   timer is used to measure the duration of button press events */
TimerHandle_t battery_client_second_timer,battery_client_ms_timer;

/* Handle of the button task */
TaskHandle_t button_handle;

typedef void  (*pfn_free_buffer_t) (uint8_t *);

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void  battery_client_application_init                (void);
static void  battery_client_interrupt_config                (void);
static void  gpio_interrupt_handler                         (void *handler_arg,
                                                             cyhal_gpio_event_t event);
static void* battery_client_alloc_buffer                    (int len);
static void  battery_client_free_buffer                     (uint8_t *p_event_data);
static void  battery_client_timeout                         (TimerHandle_t timer_handle);
static void  battery_client_fine_timeout                    (TimerHandle_t timer_handle);
static void  battery_client_smp_bond_result                 (uint8_t result);

static wiced_bt_gatt_status_t battery_client_gatts_callback (wiced_bt_gatt_evt_t event,
                                                             wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t battery_client_connection_up  (wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t battery_client_connection_down(wiced_bt_gatt_connection_status_t *p_status);
static void battery_client_callback                         (wiced_bt_battery_client_event_t event,
                                                             wiced_bt_battery_client_event_data_t *p_data);

static void battery_client_enter_pairing                    ();
static void battery_client_enable                           ( wiced_bool_t enable );
static wiced_bool_t battery_client_is_device_bonded         (wiced_bt_device_address_t bd_address);
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover     (uint16_t conn_id,
                                                             wiced_bt_gatt_discovery_type_t type,
                                                             uint16_t uuid,
                                                             uint16_t s_handle,
                                                             uint16_t e_handle);


cyhal_gpio_callback_data_t btn_cb_data =
    {
        .callback     = gpio_interrupt_handler,
        .callback_arg = NULL
    };

/**************************************************************************************************
* Function Name: app_bt_management_callback
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t battery_client_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    cy_rslt_t rslt;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_scan_type_t *p_scan_type = NULL;
    uint8_t bondindex = 0;
    printf("\n--- Event:%s ---\r\n", get_bt_event_name(event));

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                battery_client_application_init();

                /* Init bac */
                wiced_bt_battery_client_init(battery_client_callback);
            }
            else
            {
                printf( "Bluetooth enable failed \r\n" );
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            printf("Numeric_value: %"PRIu32" \r\n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            printf("\r\nPassKey Notification");
            print_bd_address(p_event_data->user_passkey_notification.bd_addr);
            printf(",PassKey %"PRIu32" \r\n", p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            printf( "Battery client, Pairing Complete Reason: %s ",
                                            get_bt_smp_status_name((wiced_bt_smp_status_t) p_info->reason));
            battery_client_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
             /* save device keys to Flash */
             rslt = app_bt_save_device_link_keys(&(p_event_data->paired_device_link_keys_update));
             if (CY_RSLT_SUCCESS == rslt)
             {
                 printf("Successfully Bonded to ");
                 print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
             }
             else
             {
                 printf("Failed to bond! \r\n");
             }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* Paired Device Link Keys Request */
            printf("Paired Device Link keys Request Event for device ");
            print_bd_address((uint8_t *)(p_event_data->paired_device_link_keys_request.bd_addr));

            /* Need to search to see if the BD_ADDR we are looking for is in Flash.
             * If not, we return WICED_BT_ERROR and the stack will generate keys
             * and will then call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that
             * they can be stored
             */

            /* Assume the device won't be found. If it is, we will set this back to WICED_BT_SUCCESS */
            wiced_result = WICED_BT_ERROR;

            bondindex = app_bt_find_device_in_flash(p_event_data->paired_device_link_keys_request.bd_addr);
            if ( bondindex < BOND_INDEX_MAX)
            {
                /* Copy the keys to where the stack wants it */
                memcpy(&(p_event_data->paired_device_link_keys_request), &(bond_info.link_keys[bondindex]), sizeof(wiced_bt_device_link_keys_t));
                wiced_result = WICED_BT_SUCCESS;
            }
            else
            {
                printf("Device Link Keys not found in the database! \n");
                bondindex=0;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Update of local privacy keys - save to Flash */
            rslt = app_bt_save_local_identity_key(p_event_data->local_identity_keys_update);
            if (CY_RSLT_SUCCESS != rslt)
            {
                wiced_result = WICED_BT_ERROR;
            }
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            app_kv_store_init();
            /*Read Local Identity Resolution Keys*/
            rslt = app_bt_read_local_identity_keys();
            if(CY_RSLT_SUCCESS == rslt)
            {
                memcpy(&(p_event_data->local_identity_keys_request), &(identity_keys), sizeof(wiced_bt_local_identity_keys_t));
                wiced_result = WICED_BT_SUCCESS;
            }
            else
            {
                wiced_result = WICED_BT_ERROR;
            }

            break;

        case BTM_ENCRYPTION_STATUS_EVT:
        {
            wiced_bt_dev_encryption_status_t *p_status = &p_event_data->encryption_status;
            printf("encryption status: res %d\n", p_status->result);
        }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;


        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Connection parameter update status:%d, Connection Interval: %d, Connection Latency: %d, Connection Timeout: %d\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            printf("Selected TX PHY - %dM\r\n Selected RX PHY - %dM\r\n",
                                        p_event_data->ble_phy_update_event.tx_phy,
                                        p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            p_scan_type = &p_event_data->ble_scan_state_changed;
            if (BTM_BLE_SCAN_TYPE_NONE == *p_scan_type)
            {
                printf("Scanning stopped\n");
#ifdef PSOC6_BLE
                /* Refer to the Note in Document History section of Readme.md */
                if(pairing_mode == TRUE)
                {
                    app_bt_add_devices_to_address_resolution_db();
                    pairing_mode = FALSE;
                }
#endif
            }
            printf( "Scan State Change: %d\r\n", p_event_data->ble_scan_state_changed );
            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
            break;
    }

    return wiced_result;
}

#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**************************************************************************************************
 * Function Name: battery_client_application_init
 **************************************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is called from the BT
 *   management callback once the LE stack enabled event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 *************************************************************************************************/
static void battery_client_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    /*
     * Interrupt configuration for User Button
     */
    battery_client_interrupt_config();
    cyhal_gpio_init(CYBSP_USER_LED1 , CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /*
     * Starting the app timers
     * Seconds timer and the ms timer
     */
    battery_client_second_timer = xTimerCreate("timeout_sectimer", pdMS_TO_TICKS(BATTERY_CLIENT_APP_TIMEOUT_IN_MS), pdTRUE,
                                            NULL, battery_client_timeout);
    xTimerStart(battery_client_second_timer, BATTERY_CLIENT_APP_TIMEOUT_IN_MS);

    battery_client_ms_timer = xTimerCreate("timeout_mstimer", pdMS_TO_TICKS(BATTERY_CLIENT_APP_FINE_TIMEOUT_IN_MS), pdTRUE,
                                            NULL, battery_client_fine_timeout);
    xTimerStart(battery_client_ms_timer, BATTERY_CLIENT_APP_FINE_TIMEOUT_IN_MS);

    if(app_bt_restore_bond_data() == CY_RSLT_SUCCESS)
    {
        //printf("Keys found in flash, add them to Addr Res DB\r\n");
        /* Load previous paired keys for address resolution */
        app_bt_add_devices_to_address_resolution_db();
    }

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(battery_client_gatts_callback);
    printf("GATT event Handler registration status: %s \n",get_bt_gatt_status_name(gatt_status));

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    battery_client_state.flag_stay_connected = 1;

    /* Prompt user */
    printf("======================================================================\n");
    printf("| Press and Hold User Button:                                         |\n");
    printf("|   0-2  seconds: Start Scan and Pair with Battery Server.            |\n");
    printf("|   2-5  seconds: Request a Read of Battery Level.                    |\n");
    printf("|   5-10 seconds: Enable/Disable Battery Service.                     |\n");
#ifdef PSOC6_BLE
    printf("|   >10  seconds:  Connect, Pair and Bond with a new peer device      |\n");
#endif
    printf("======================================================================\n");
}

/**************************************************************************************************
 * Function Name: battery_client_interrupt_config
 ***************************************************************************************************
 * Summary:
 *   This function initializes a pin as input that triggers interrupt on falling and rising edges.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 *************************************************************************************************/
void battery_client_interrupt_config(void)
{
    /* Initialize the user button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, CYBSP_BTN_OFF);
    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_cb_data);
    /* Enable for both FALL and RISE */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH, GPIO_INTERRUPT_PRIORITY, true);
    /* Set button state to unknown */
    battery_client_state.usr_button_state = USR_BUTTON_STATE_UNKNOWN;

}

/**************************************************************************************************
* Function Name: le_app_gatt_event_callback
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t battery_client_gatts_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            if (p_event_data->connection_status.connected)
            {
                battery_client_connection_up(&p_event_data->connection_status);
            }
            else
            {
                battery_client_connection_down(&p_event_data->connection_status);
            }
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            gatt_status = battery_client_gatt_discovery_result(&p_event_data->discovery_result);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            gatt_status = battery_client_gatt_discovery_complete(&p_event_data->discovery_complete);
            break;

        case GATT_OPERATION_CPLT_EVT:
            gatt_status = battery_client_gatt_operation_complete(&p_event_data->operation_complete);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            battery_client_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)battery_client_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;
        case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;

        default:
            gatt_status = WICED_BT_GATT_SUCCESS;
               break;
    }

    return gatt_status;
}

/*******************************************************************************
 * Function Name: battery_client_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 ******************************************************************************/
static void battery_client_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
 * Function Name: battery_client_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 *
 ******************************************************************************/
static void* battery_client_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/*******************************************************************************
 * Function Name : battery_client_smp_bond_result
 * *****************************************************************************
 * Summary :
 *    Process SMP bonding result. If we successfully paired with the central
 *    device, save its BDADDR in the NVRAM and initialize associated data
 *
 *
 * Parameters:
 *    uint8_t result : Result of BTM_PAIRING_COMPLETE_EVT event
 *
 * Return:
 *    None
 ******************************************************************************/
static void battery_client_smp_bond_result( uint8_t result )
{
    /* Bonding success */
    if ( result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( battery_client_hostinfo.bdaddr, battery_client_state.remote_addr, sizeof( wiced_bt_device_address_t ) );

        app_bt_update_hostinfo();
    }

}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
* Return
*  None
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(event == CYHAL_GPIO_IRQ_FALL)
    {
        battery_client_state.usr_button_state = USR_BUTTON_STATE_DOWN;
    }
    if(event == CYHAL_GPIO_IRQ_RISE)
    {
        battery_client_state.usr_button_state = USR_BUTTON_STATE_UP;
    }

    vTaskNotifyGiveFromISR(button_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: battery_client_timeout
********************************************************************************
* Summary:
*  The function is invoked on timeout of app seconds timer.
*
* Parameters:
*  Timerhandle
*
* Return
*  None
*
*******************************************************************************/
void battery_client_timeout(TimerHandle_t timer_handle)
{
    battery_client_state.timer_count++;
}

/*******************************************************************************
* Function Name: battery_client_fine_timeout
********************************************************************************
* Summary:
*  The function invoked on timeout of app milliseconds fine timer
*
* Parameters:
*  Timerhandle
*
* Return
*  None
*
*******************************************************************************/
void battery_client_fine_timeout(TimerHandle_t timer_handle)
{
    battery_client_state.fine_timer_count++;
}

/*******************************************************************************
* Function Name: button_task
********************************************************************************
* Summary:
 *  This is a FreeRTOS task that handles the button press events.
*
* Parameters:
*  None
*
* Return
*  None
*
*******************************************************************************/
void button_task(void *arg)
{
    static uint32_t button_state = USR_BUTTON_STATE_UNKNOWN;
    static unsigned long previous_timer = 0;
    static unsigned long down_duration = 0;      /* in seconds */
#ifdef PSOC6_BLE
    wiced_result_t result = WICED_BT_SUCCESS;
#endif
    battery_client_state.usr_button_action = USR_BUTTON_ACTION_NONE;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE,  portMAX_DELAY);
        button_state = cyhal_gpio_read (CYBSP_USER_BTN);

        /*
         * 1. User Button pressed and released within 2 seconds:
         *   - Battery Client App to scan and connect to the Battery Service Server
         *   - If no Battery Service Server device is found nearby for 90 seconds,
         *      then the scan stops automatically
         *   - Upon successful Connection, the Battery Client App will discover
         *      all the characteristics/descriptors of the server device
         *
         * 2. To read the battery level of the server, push the button on the board and release between 2-4 seconds
         *
         * 3. User button pressed and released after 5 seconds once connected
         *   - Enable/Disable battery notifications of level from server
         *
         * 4 . User button pressed and released after 10 seconds for PSoC6 Bluetooth LE
         *   - This enables PSoC 6 Bluetooth LE devices to send scan request packets while active scanning.
         *   - This is a work around for a known issue referenced in the Note section under document history.
         *   - This work around is required only if PSoC 6 Bluetooth LE device has bonded with a device and
         *     needs to connect with a new device.
         */

        if (button_state == USR_BUTTON_STATE_DOWN)
        {
            down_duration = 0;
            previous_timer = 0;
            battery_client_state.fine_timer_count = 0;   /* reset */
            printf("\n--- User Button Down ---\n");

            /* Set to scan mode */
            printf("Release to Scan and Pair to Battery Server \n");
            battery_client_state.usr_button_action = USR_BUTTON_ACTION_SCAN;

            while(battery_client_state.usr_button_state == USR_BUTTON_STATE_DOWN)
            {
                down_duration = battery_client_state.fine_timer_count / 1000;
                if (down_duration > previous_timer)
                {
                    printf("%lu seconds\n", down_duration);
                    previous_timer = down_duration;

                    switch (down_duration)
                    {
                    case 2:     /* 2-4 seconds - BAS read */
                        printf("Release to perform BAS Read\r\n");
                        break;

                    case 5:     /* 5 seconds - Enable/Disable BAS notifications */
                        printf("Release now to %s BAS\r\n", is_bas_enabled ? "disable" : "enable");
                        break;
                    case 9:     /* 9 seconds - To enter paring mode */
#ifdef PSOC6_BLE
                        printf("Release now to enter pairing mode\r\n");
#endif
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        else if (button_state == USR_BUTTON_STATE_UP)
        {
            printf("\n--- User Button Up ---\n");
            printf("Press Duration - %lu\n", down_duration);
            if(down_duration >= 10)
            {
#ifdef PSOC6_BLE
                /* Refer to the Note in document history of Readme.md */
                printf("Entering Pairing Mode: Connect, Pair and Bond with a new peer device...\n");
                pairing_mode = TRUE;
                result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                       0,
                                                       NULL);
                /* Refer to the Note in Document History section of Readme.md */
                result = wiced_bt_ble_address_resolution_list_clear_and_disable();
                if(WICED_BT_SUCCESS == result)
                {
                    printf("Address resolution list cleared successfully \n");
                }
                else
                {
                    printf("Failed to clear address resolution list \n");
                }
                /* Start scanning after clearing address resolution list */
                result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, true,
                        battery_client_scan_result_cback);

                /* Failed to start scanning. Stop program execution */
                if ((WICED_BT_PENDING != result) && (WICED_BT_BUSY != result))
                {
                    printf("Failed to start scanning!, result = %d \n", result);
                    CY_ASSERT(0);
                }
                else
                {
                    printf("Scanning started successfully\r\n");
                }
#endif
            }
            else if (down_duration > 4)
            {
                printf("BAC Client enabled\r\n");
                battery_client_enable(!is_bas_enabled);
            }
            else if(down_duration >= 2)
            {
                printf("BAC Read\r\n");
                wiced_bt_battery_client_read( battery_client_app_data.conn_id );
            }
            else if (down_duration < 2)
            {
                printf("BAC enter pairing\r\n");
                battery_client_enter_pairing();
            }
        }
    }
}

/*******************************************************************************
* Function Name: battery_client_enable
********************************************************************************
* Summary:
 *  Enable/Disable server notification.
*
* Parameters:
*  enable
*
* Return
*  None
*
*******************************************************************************/
static void battery_client_enable( wiced_bool_t enable )
{
    is_bas_enabled = enable;

    if(is_bas_enabled)
    {
        printf("Enabling notifications\n");
        wiced_bt_battery_client_enable( battery_client_app_data.conn_id );
    }
    else
    {
        printf("Disabling notifications\n");
        wiced_bt_battery_client_disable( battery_client_app_data.conn_id );
    }
}

/*******************************************************************************
* Function Name: battery_client_out_bytes
********************************************************************************
* Summary:
 *  Print message and char array of hex bytes
*
* Parameters:
*  msg
*  len
*  *p_data
*
* Return
*  None
*
*******************************************************************************/
static void battery_client_out_bytes(char * msg, uint16_t len, uint8_t  *p_data)
{
    printf("%s data: ", msg);
    while (len--)
    {
        printf("%02x ",*p_data++);
    }
    printf("\r\n");
}

/*******************************************************************************
* Function Name: battery_client_show_data
********************************************************************************
* Summary:
 *  Print battery level event data
*
* Parameters:
*  msg
*  event
*  *p_data
*
* Return
*  None
*
*******************************************************************************/
static void battery_client_show_data(char * msg, wiced_bt_battery_client_event_t event, wiced_bt_battery_client_event_data_t *p_data)
{
#ifdef BAS_1_1
    uint8_t flags8 = 0, flags16 = 0;
#endif

    switch (p_data->data.uuid)
    {
    case UUID_CHARACTERISTIC_BATTERY_LEVEL:
        /* Skip battery level messages output if the user button being pressed */
        if (cyhal_gpio_read (CYBSP_USER_BTN) != USR_BUTTON_STATE_DOWN)
        {
            printf("[%13s] Battery Level:%d\r\n", msg, p_data->data.p_data[0]);
        }
        break;

 #ifdef BAS_1_1
    case UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS:
        {
            UINT16 identifier = 0, power_state = 0, flags8 = 0;
            UINT8  battery_level = 0, additional_status = 0;

            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            STREAM_TO_UINT16(power_state, p_data->data.p_data);
            batt_power_state_t *batt_power_state = (batt_power_state_t *)&power_state;

            if ( flags8 & BATTERY_LEVEL_STATUS_ID_PRESENT )
            {
                STREAM_TO_UINT16(identifier, p_data->data.p_data);
            }
            if ( flags8 & BATTERY_LEVEL_STATUS_LEVEL_PRESENT )
            {
                STREAM_TO_UINT8(battery_level, p_data->data.p_data);
            }
            if ( flags8 & BATTERY_LEVEL_STATUS_ADD_ST_PRESENT )
            {
               STREAM_TO_UINT8(additional_status, p_data->data.p_data);
            }
            batt_add_status_t *batt_add_status = (batt_add_status_t *)&additional_status;

            printf("BATTERY_LEVEL_STATUS: flags8:%d  batt_present:%d wired_ext_pwr_connected:%d wireless_ext_pwr_connected:%d \
                           batt_charge_state:%d  batt_charge_level:%d batt_charge_type:%d batt_charge_fault:%d, rfu:%d\r\n", flags8, batt_power_state->batt_present,
                           batt_power_state->wired_ext_pwr_connected, batt_power_state->wireless_ext_pwr_connected, batt_power_state->batt_charge_state,
                           batt_power_state->batt_charge_level, batt_power_state->batt_charge_type, batt_power_state->batt_charge_fault, batt_power_state->rfu);

            printf("identifier:%d  battery_level:%d service_required:%d rfu:%d\r\n\r\n", identifier, battery_level,
                            batt_add_status->service_required, batt_add_status->rfu );

            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_ESTIMATED_SERVICE_DATE:
        {
            // okay to cast p_date into batt_estimated_service_date_t as no flags field or optional fields
            batt_estimated_service_date_t  *p_date = (batt_estimated_service_date_t *) p_data->data.p_data;
            uint32_t date = 0;
            uint8_t *buffer = (uint8_t *) p_date->estimated_service_date;
            STREAM_TO_UINT24(date, buffer);
            printf("ESTIMATED_SERVICE_DATE:%d  %s \r\n\r\n", date, msg );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS:
        {
            // same as above - no flags field or optional fields
            batt_critical_status_t  *p_critical = (batt_critical_status_t *) p_data->data.p_data;
            printf("CRITICAL_STATUS:%s critical_power_state:%d immediate_service_required:%d critical_status.rfu:%d\r\n\r\n",  msg,
            p_critical->batt_critical_status.critical_power_state,  p_critical->batt_critical_status.immediate_service_required, p_critical->batt_critical_status.rfu );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS:
        {
            UINT16 external_source_power = 0, present_voltage = 0, available_energy = 0;
            UINT16 available_power_capacity = 0, change_rate = 0, available_energy_at_last_change = 0;

            // batt_energy_status_t *p_energy = (batt_energy_status_t *) p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE_PRESENT )
            {
                STREAM_TO_UINT16( external_source_power, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE_PRESENT )
            {
                STREAM_TO_UINT16( present_voltage, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_PRESENT )
            {
                STREAM_TO_UINT16( available_energy, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY_PRESENT )
            {
                STREAM_TO_UINT16( available_power_capacity, p_data->data.p_data );
            }
             if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE_PRESENT )
            {
                STREAM_TO_UINT16( change_rate, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE_PRESENT )
            {
                STREAM_TO_UINT16( available_energy_at_last_change, p_data->data.p_data );
            }
            printf("BATTERY_ENERGY_STATUS:%s flags8:%d external_source_power:%d present_voltage:%d \r\n\r\n",
                            msg, flags8, external_source_power, present_voltage);
            printf("available_energy:%d  available_power_capacity:%d change_rate:%d  available_energy_at_last_change:%d \r\n",
                            available_energy, available_power_capacity,  change_rate, available_energy_at_last_change );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_TIME_STATUS:
        {
            UINT32  time_until_discharged = 0,  time_until_discharged_on_standby = 0, time_until_recharged = 0;
            //batt_time_status_t *p_time = (batt_time_status_t *)p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            STREAM_TO_UINT24( time_until_discharged, p_data->data.p_data );

            if ( flags8 & BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY_PRESENT )
            {
                STREAM_TO_UINT24( time_until_discharged_on_standby, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED_PRESENT )
            {
                STREAM_TO_UINT24( time_until_recharged, p_data->data.p_data );
            }

            printf("TIME_STATUS:%s flags8:%d time_until_discharged:%d time_until_discharged_on_standby:%d time_until_recharged:%d\r\n\r\n",
                            msg, flags8, time_until_discharged, time_until_discharged_on_standby, time_until_recharged);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS:
        {
            UINT8   health_summary = 0, current_temp = 0;
            UINT16  cycle_count = 0, deep_discharge_count = 0;

            //batt_health_status_t *p_health = (batt_health_status_t *)p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);

            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_SUMMARY_PRESENT )
            {
                STREAM_TO_UINT8( health_summary, p_data->data.p_data );
            }

            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT_PRESENT )
            {
                STREAM_TO_UINT16( cycle_count, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP_PRESENT )
            {
                STREAM_TO_UINT8( current_temp, p_data->data.p_data );
            }

            if ( flags16 & BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT_PRESENT )
            {
                STREAM_TO_UINT16( deep_discharge_count, p_data->data.p_data );
            }

            printf("HEALTHSTATUS:%s flags16:%d health_summary:%d cycle_count:%d current_temp:%d deep_discharge_count:%d \r\n\r\n", msg, flags16,
                           health_summary, cycle_count, current_temp, deep_discharge_count);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO:
        {
            //batt_health_info_t *p_health = (batt_health_info_t *)p_data->data.p_data;
            UINT16 cycle_count = 0;
            UINT8 min_op_temp = 0, max_op_temp = 0;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);

            if ( flags8 & BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME_PRESENT )
            {
                STREAM_TO_UINT16( cycle_count, p_data->data.p_data );
            }

            if ( flags8 & BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP_PRESENT )
            {
                STREAM_TO_UINT8( min_op_temp, p_data->data.p_data );
                STREAM_TO_UINT8( max_op_temp, p_data->data.p_data );
            }

            printf("HEALTH_STATUS:%s flags:%d cycle_count :%d min__op_temp:%d max_op_temp:%d \r\n\r\n", msg,
                            flags8, cycle_count, min_op_temp, max_op_temp );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_INFO:
        {
            UINT8    features = 0, batt_aggr_grp = 0, batt_chemistry = 0;
            UINT32   batt_manuf_date = 0, batt_expr_date = 0;
            UINT16 batt_designed_cap = 0, batt_low_energy = 0, batt_critical_energy = 0, batt_nominal_volage = 0;

            STREAM_TO_UINT16(flags16, p_data->data.p_data);
            STREAM_TO_UINT8(features, p_data->data.p_data);
            batt_features_t *batt_features = (batt_features_t *)&features;

            //batt_info_t *p_info = (batt_info_t *)p_data->data.p_data;
            if ( flags16 & BATTERY_INFO_FLAG_BATT_MANUF_DATE_PRESENT )
            {
                STREAM_TO_UINT24( batt_manuf_date, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_EXPRATION_DATE_PRESENT )
            {
                STREAM_TO_UINT24( batt_expr_date, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_DESIGNED_CAP_PRESENT )
            {
                STREAM_TO_UINT16( batt_designed_cap, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_LOW_ENGERGY_PRESENT )
            {
                STREAM_TO_UINT16( batt_low_energy, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY_PRESENT )
            {
                STREAM_TO_UINT16( batt_critical_energy, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_CHEMISTRY_PRESENT )
            {
                STREAM_TO_UINT8( batt_chemistry, p_data->data.p_data )
            }
            if ( flags16 & BATTERY_INFO_FLAG_NOMINAL_VOLTAGE_PRESENT )
            {
                STREAM_TO_UINT16( batt_nominal_volage, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_AGGR_GRP_PRESENT )
            {
                STREAM_TO_UINT8( batt_aggr_grp, p_data->data.p_data )
            }

            printf("BATTERY INFO:[%5s] flags16:%d battery_features:%02X battery_replace:%d battery_recharge:%d batt_features_rfu:%d \r\n",
                            msg, flags16, features, batt_features->batt_replaceable, batt_features->batt_rechargeable, batt_features->rfu);
            printf("batt_manuf_date:%d batt_expr_date:%d  batt_designed_cap:%d batt_low_energy:%d batt_critical_energy:%d \r\n",
                            batt_manuf_date, batt_expr_date, batt_designed_cap, batt_low_energy, batt_critical_energy);
            printf("batt_chemistry:%d batt_nominal_volage:%d batt_aggr_grp:%d \r\n\r\n",
                            batt_chemistry, batt_nominal_volage, batt_aggr_grp);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NAME:
        p_data->data.p_data[p_data->data.len] = 0; // terminate string
        printf("MANUFACTURE NAME [%13s]:%s \r\n\r\n",  msg, p_data->data.p_data );
        break;
    case UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NUMBER:
        p_data->data.p_data[p_data->data.len] = 0;
        printf("MODEL NUMBER [%13s]:%s \r\n\r\n",  msg, p_data->data.p_data );
        break;
    case UUID_CHARACTERISTIC_BATTERY_SERIAL_NUMBER:
        p_data->data.p_data[p_data->data.len] = 0;
        printf("SERIAL NUMBER [%13s]: %s \r\n\r\n",  msg, p_data->data.p_data );
        break;
#endif

    default:
        battery_client_out_bytes("", p_data->data.len, p_data->data.p_data);
        break;
    }
}

/*******************************************************************************
* Function Name: battery_client_callback
********************************************************************************
* Summary:
 *  Callback for battery client events
*
* Parameters:
*  event
*  *p_data
*
* Return
*  None
*
*******************************************************************************/
static void battery_client_callback(wiced_bt_battery_client_event_t event, wiced_bt_battery_client_event_data_t *p_data)
{
    switch (event)
    {
    case WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE:
        /* If Battery Service successfully discovered */
        if (p_data->discovery.status == WICED_BT_GATT_SUCCESS)
        {
            /* Enable Battery Level services */
            battery_client_enable( WICED_TRUE );

            /* Perform a BAS read all data */
            wiced_bt_battery_client_read(p_data->discovery.conn_id);
        }
        else
        {
            printf("Battery Level Service not found!\n");
        }
        break;

    case WICED_BT_BAC_EVENT_RSP:
        battery_client_show_data("Read Response", event, p_data);
        break;

    case WICED_BT_BAC_EVENT_NOTIFICATION:
        battery_client_show_data("Notification", event, p_data);
        break;

    case WICED_BT_BAC_EVENT_INDICATION:
        battery_client_show_data("Indication", event, p_data);
        break;

    default:
        printf("Unknown BAC Event:%d\r\n", event);
        break;
    }
}

/*******************************************************************************
* Function Name: battery_client_scan_result_cback
********************************************************************************
* Summary:
*  Handles the scan results and attempt to connect to Battery Service Server
*
* Parameters:
*  *p_scan_result
*  *p_adv_data
*
* Return
*  None
*
*******************************************************************************/
void battery_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t          status;
    wiced_bool_t            ret_status;
    uint8_t                 length = 0;
    uint8_t *               p_data;
    uint16_t                service_uuid16=0;

    if ( p_scan_result )
    {

#ifdef BATTERY_LEVEL_BROADCAST
        // check to see if the advertisement is battery level broadcast
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &length );
        if ( p_data != NULL )
        {
            printf( "Broadcast advertisement with length %d\r\n", length);

            uint8_t len = 0, battery_level = 0, rfu = 0;
            uint16_t flags = 0, identifier = 0;

            STREAM_TO_UINT16( service_uuid16, p_data );
            STREAM_TO_UINT8( len, p_data );

            if ( (service_uuid16 == UUID_SERVICE_BATTERY)
            {
                STREAM_TO_UINT16( flags, p_data );
                STREAM_TO_UINT16( identifier, p_data );

                STREAM_TO_UINT8( battery_level, p_data );
                STREAM_TO_UINT8( rfu, p_data );
                printf( "Broadcast advertisement with len:%d flags:%02X identifier:%02X level:%d rfu:%d \r\n",
                                 len, flags, identifier, battery_level, rfu );
            }
            return;     // safe to return from here as the broadcast adv packet not used for connection purpose

        }
#endif
        if ( battery_client_app_data.conn_id )
        {
            // return if connected to a server already
            return;
        }
        // Search for SERVICE_UUID_16 element in the Advertisement data received.Check for both
        // complete and partial list
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length );
        if ( p_data == NULL )
        {
            p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL, &length );

            if (p_data == NULL){
                return;     // No UUID_16 element
            }
        }

        while (length >= LEN_UUID_16)
        {
            STREAM_TO_UINT16(service_uuid16, p_data);
            if (service_uuid16 == UUID_SERVICE_BATTERY)
            {
                // UUID16 Battery Service found
                break;
            }
            length -= LEN_UUID_16;
        }

        if (service_uuid16 != UUID_SERVICE_BATTERY)
        {
            // UUID16 Battery Service not found. Ignore device
            return;
        }

        printf("Battery Server Device found: ");
        print_bd_address(p_scan_result->remote_bd_addr);
        printf(" addr_type:%d\r\n", p_scan_result->ble_addr_type);

        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, battery_client_scan_result_cback );
        printf( "scan off status %d\r\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_LOW_DUTY, TRUE );
        printf( "wiced_bt_gatt_connect status %d\r\n", ret_status );
    }
    else
    {
        printf( "Scan completed:\r\n" );
    }
}

/*******************************************************************************
* Function Name: battery_client_enter_pairing
********************************************************************************
* Summary:
*  Initiate scanning with callback on scan result
*
* Parameters:
*  None
*
* Return
*  None
*
*******************************************************************************/
static void battery_client_enter_pairing()
{
    wiced_result_t result;

    printf("Entering discovery\r\n");
    /*start scan if not connected and no scan in progress*/
    if (( battery_client_app_data.conn_id == 0 ) && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, battery_client_scan_result_cback );
        printf("wiced_bt_ble_scan: %d \r\n", result);
    }
    else
    {
         printf("Scanning not initiated, scan in progress ??  \r\n");
    }
}

/*******************************************************************************
* Function Name: battery_client_enter_pairing
********************************************************************************
* Summary:
*  Save peer info
*
* Parameters:
*  conn_id
*  p_bd_addr
*  role
*  transport
*  address_type
*
* Return
*  None
*
*******************************************************************************/
void battery_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type )
{
    battery_client_app_data.addr_type = address_type;
    battery_client_app_data.conn_id = conn_id;
    memcpy(battery_client_app_data.peer_addr,p_bd_addr,BD_ADDR_LEN);
    battery_client_app_data.role = role;
    battery_client_app_data.transport = transport;
}

/*******************************************************************************
* Function Name: battery_client_connection_up
********************************************************************************
* Summary:
*  Process a connection event and perforam primary service search
*
* Parameters:
*  p_conn_status
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
static wiced_bt_gatt_status_t battery_client_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    uint8_t dev_role;
    wiced_bt_gatt_status_t status ;
    battery_service_found = 0;

    printf("battery_client_connection_up()\n");
    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Adding the peer info
    battery_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, dev_role , p_conn_status->transport, p_conn_status->addr_type );

    printf( "battery client_connection_up Conn Id:%d ",p_conn_status->conn_id);
    print_bd_address(p_conn_status->bd_addr);
    printf( " role:%d\n", dev_role);

    wiced_bt_battery_client_connection_up( p_conn_status );

    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

    // perform primary service search
    status = wiced_bt_util_send_gatt_discover( p_conn_status->conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    printf("start discover status:%d\r\n", status);
    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: battery_client_connection_down
********************************************************************************
* Summary:
*  Process a dis-connection event
*
* Parameters:
*  p_conn_status
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
static wiced_bt_gatt_status_t battery_client_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    printf("battery client connection down\r\n");
    battery_client_app_data.conn_id = 0;

    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

#ifdef TEST_HCI_CONTROL
    battery_client_hci_send_disconnect_event( p_conn_status->reason, p_conn_status->conn_id );
#endif
    wiced_bt_battery_client_connection_down( p_conn_status );

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: battery_client_gatt_discovery_result
********************************************************************************
* Summary:
*  Process a GATT discovery result
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
wiced_bt_gatt_status_t battery_client_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    printf("[%s] conn %d, discovery type %d, state %d\r\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
        case BAC_DISCOVERY_STATE_CHAR:
            wiced_bt_battery_client_discovery_result(p_data);
            break;

        default:
            if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
            {
                if (p_data->discovery_data.group_value.service_type.len == LEN_UUID_16 )
                {
                   printf("uuid:%04x start_handle:%04x end_handle:%04x\r\n",
                           p_data->discovery_data.group_value.service_type.uu.uuid16,
                           p_data->discovery_data.group_value.s_handle,
                           p_data->discovery_data.group_value.e_handle);
                   if( (p_data->discovery_data.group_value.service_type.uu.uuid16 == UUID_SERVICE_BATTERY)  && (battery_service_found != 1) )
                   {
                       battery_service_found = 1; // ignore any more battery services, connect to first discovered one
                       printf("Battery Service found s:%04x e:%04x\r\n",
                               p_data->discovery_data.group_value.s_handle,
                               p_data->discovery_data.group_value.e_handle);
                        battery_client_app_state.bac_s_handle = p_data->discovery_data.group_value.s_handle;
                        battery_client_app_state.bac_e_handle = p_data->discovery_data.group_value.e_handle;
                    }
                }
            }
            else
            {
                printf("!!!! invalid op:%d\r\n", p_data->discovery_type);
            }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: battery_client_gatt_discovery_complete
********************************************************************************
* Summary:
*  Process a GATT discovery complete event
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
wiced_bt_gatt_status_t battery_client_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    printf("[%s] conn %d type %d state %d\r\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
    case BAC_DISCOVERY_STATE_CHAR:
        wiced_bt_battery_client_discovery_complete(p_data);
        break;

    default:
        if (p_data->discovery_type == GATT_DISCOVER_SERVICES_ALL)
        {
            printf("bac handles:%04x-%04x\r\n", battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle);

            /* If bac Service found - start discovery */
            if ((battery_client_app_state.bac_s_handle != 0) && (battery_client_app_state.bac_e_handle != 0))
            {
                battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_CHAR;
                if (wiced_bt_battery_client_discover(battery_client_app_data.conn_id, battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle))
                    break;
            }
        }
        else
        {
            printf("!!!! invalid op:%d\r\n", p_data->discovery_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: battery_client_gatt_operation_complete
********************************************************************************
* Summary:
*  Process a GATT operation complete event
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
wiced_bt_gatt_status_t battery_client_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_result_t              status;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    //printf("battery_client_gatt_operation_complete conn %d op %d st %d\r\n", p_data->conn_id, p_data->op, p_data->status );

    battery_client_handle_op_complete(p_data);

    /* server puts authentication requirement. Encrypt the link */
    if ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION )
    {
        printf("Insufficient Authentication\r\n");
        if ( battery_client_is_device_bonded(battery_client_app_data.peer_addr) )
        {
            printf("Authentified. Start Encryption\r\n");
            status = wiced_bt_dev_set_encryption( battery_client_app_data.peer_addr,
                    battery_client_app_data.transport, &encryption_type );
            printf( "wiced_bt_dev_set_encryption %d \r\n", status );
        }
        else
        {
            printf("Start Authentification/Pairing\r\n");
            status = wiced_bt_dev_sec_bond( battery_client_app_data.peer_addr,
                    battery_client_app_data.addr_type, battery_client_app_data.transport,0, NULL );
            printf( "wiced_bt_dev_sec_bond %d \r\n", status );
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: battery_client_handle_op_complete
********************************************************************************
* Summary:
*  Process a GATT operation complete event
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
void battery_client_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch ( p_data->op )
    {
    case GATTC_OPTYPE_READ_HANDLE:
    case GATTC_OPTYPE_READ_BY_TYPE:
    case GATTC_OPTYPE_READ_MULTIPLE:
        //printf( "read_rsp status:%d\n", p_data->status );
        battery_client_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
        //printf( "write_rsp status:%d desc_handle:%x \n", p_data->status,p_data->response_data.handle );
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
        //printf( "peer mtu:%d\n", p_data->response_data.mtu );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        //printf( "notification status:%d\n", p_data->status );
        battery_client_notification_handler( p_data );
        break;

    case GATTC_OPTYPE_INDICATION:
        //printf( "indication status:%d\n", p_data->status );
        battery_client_indication_handler( p_data );
        break;
    }
}

/*******************************************************************************
* Function Name: battery_client_process_read_rsp
********************************************************************************
* Summary:
*  Pass read response to appropriate client based on the attribute handle
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
void battery_client_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
//    printf("read response handle:%04x s:%04x e:%04x\r\n", p_data->response_data.handle, battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle);

    // Verify that read response is for our service
    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_read_rsp(p_data);
    }
}

/*******************************************************************************
* Function Name: battery_client_notification_handler
********************************************************************************
* Summary:
*  Pass notification to appropriate client based on the attribute handle
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
void battery_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    //printf("notification handle:%04x\r\n", p_data->response_data.att_value.handle);

    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_process_notification(p_data);
    }
}

/*******************************************************************************
* Function Name: battery_client_indication_handler
********************************************************************************
* Summary:
*  Pass indication to appropriate client based on the attribute handle
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
void battery_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    printf("indication handle:%04x\r\n", p_data->response_data.att_value.handle);

    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_process_indication(p_data);
    }
}

/*******************************************************************************
* Function Name: battery_client_is_device_bonded
********************************************************************************
* Summary:
*  Check for device entry exists in NVRAM list
*
* Parameters:
*  *p_data
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
wiced_bool_t battery_client_is_device_bonded(wiced_bt_device_address_t bd_address)
{
    // search through all available NVRAM IDs.
    if (app_bt_find_device_in_flash(bd_address))
    {
        printf(" [%s] read success\n", __FUNCTION__);
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*
 * Format and send GATT Discover request
 */
 /*******************************************************************************
* Function Name: wiced_bt_util_send_gatt_discover
********************************************************************************
* Summary:
*  Format and send GATT Discover request
*
* Parameters:
*  conn_id
*  type
*  uuid
*  s_handle
*  e_handle
*
* Return
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param;
    wiced_bt_gatt_status_t          status;

    memset(&param, 0, sizeof(param));
    if (uuid != 0)
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }
    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_client_send_discover(conn_id, type, &param);
    return status;
}
/* END OF FILE [] */
