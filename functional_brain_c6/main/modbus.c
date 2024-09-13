#include "modbus.h"
#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 0

#define MB_PORT_NUM (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED (CONFIG_MB_UART_BAUD_RATE) // The communication speed of the UART

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS (500)
#define UPDATE_CIDS_TIMEOUT_TICS (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) {.opt1 = min_val, .opt2 = max_val, .opt3 = step_val}

const char *TAG = "MASTER_TEST";

// Enumeration of modbus device addresses accessed by master device
enum
{
    MEANWELL_ID = 192, // Only one slave device used for the test (add other slave addresses here)
    JBD_ID = 1
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum
{
    CID_INP_DATA_0 = 0,
    CID_HOLD_DATA_0,
    CID_INP_DATA_1,
    CID_HOLD_DATA_1,
    CID_INP_DATA_2,
    CID_HOLD_DATA_2,
    CID_HOLD_TEST_REG,
    CID_RELAY_P1,
    CID_RELAY_P2,
    CID_DISCR_P1,
    CID_COUNT
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode, Scalar}
    {0, STR("Cell Over Voltage Trigger Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x00, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {1, STR("Cell Over Voltage Release Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x01, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {2, STR("Cell Under Voltage Trigger Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x02, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {3, STR("Cell Under Voltage Release Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x03, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {4, STR("Battery Over Voltage Trigger Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x04, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {5, STR("Battery Over Voltage Release Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x05, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {6, STR("Battery Under Voltage Trigger Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x06, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {7, STR("Battery Under Voltage Release Val"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x07, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},

    {8, STR("Charge over Current"), STR("mA"), JBD_ID, MB_PARAM_HOLDING, 0x10, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {9, STR("Discharge over Current"), STR("mA"), JBD_ID, MB_PARAM_HOLDING, 0x11, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {10, STR("Pack Voltage"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x12, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {11, STR("Pack Amperes"), STR("mA"), JBD_ID, MB_PARAM_HOLDING, 0x13, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 10},
    {12, STR("Current Pack Capacity"), STR("Ah"), JBD_ID, MB_PARAM_HOLDING, 0x14, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},

    {13, STR("Cell Voltage 1"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x15, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {14, STR("Cell Voltage 2"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x16, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {15, STR("Cell Voltage 3"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x17, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {16, STR("Cell Voltage 4"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x18, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {17, STR("Cell Voltage 5"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x19, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {18, STR("Cell Voltage 6"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1A, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {19, STR("Cell Voltage 7"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1B, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {20, STR("Cell Voltage 8"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1C, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {21, STR("Cell Voltage 9"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1D, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {22, STR("Cell Voltage 10"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1E, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {23, STR("Cell Voltage 11"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x1F, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {24, STR("Cell Voltage 12"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x20, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {25, STR("Cell Voltage 13"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x21, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {26, STR("Cell Voltage 14"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x22, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {27, STR("Cell Voltage 15"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x23, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {28, STR("Cell Voltage 16"), STR("mV"), JBD_ID, MB_PARAM_HOLDING, 0x24, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},

    {29, STR("Full Pack Capacity"), STR("Ah"), JBD_ID, MB_PARAM_HOLDING, 0x27, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {30, STR("State of Charge"), STR("%%"), JBD_ID, MB_PARAM_HOLDING, 0x28, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},

    // Meanwell Stuff
    // {0, STR("READ_VIN"), STR("mV"), MEANWELL_ID, MB_PARAM_INPUT, 0x50, 1,
    //  INPUT_OFFSET(input_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    // {1, STR("READ_VIN"), STR("mA"), MEANWELL_ID, MB_PARAM_INPUT, 0x53, 1,
    //  INPUT_OFFSET(input_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},

    {31, STR("CURVE_CC"), STR("__"), MEANWELL_ID, MB_PARAM_HOLDING, 0xB0, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {32, STR("CHG_STATUS_2"), STR("__"), MEANWELL_ID, MB_PARAM_HOLDING, 0xB1, 1,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},

    {33, STR("READ_FREQ"), STR("Hz"), MEANWELL_ID, MB_PARAM_INPUT, 0x56, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {34, STR("READ_ TEMPERATURE_1"), STR("__"), MEANWELL_ID, MB_PARAM_INPUT, 0x62, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {35, STR("READ_FAN_SPEED_1"), STR("__"), MEANWELL_ID, MB_PARAM_INPUT, 0x70, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {36, STR("READ_FAN_SPEED_2"), STR("__"), MEANWELL_ID, MB_PARAM_INPUT, 0x71, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {37, STR("READ_AC_FOUT"), STR("Hz"), MEANWELL_ID, MB_PARAM_INPUT, 0x105, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {38, STR("READ_AC_VOUT"), STR("mV"), MEANWELL_ID, MB_PARAM_INPUT, 0x108, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {39, STR("READ_OP_LD_PCNT"), STR("%%"), MEANWELL_ID, MB_PARAM_INPUT, 0x10B, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {40, STR("READ_OP_WATT_HI"), STR("W"), MEANWELL_ID, MB_PARAM_INPUT, 0x10E, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {41, STR("READ_OP_WATT_LO"), STR("W"), MEANWELL_ID, MB_PARAM_INPUT, 0x10F, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {42, STR("READ_OP_VA_HI"), STR("VA"), MEANWELL_ID, MB_PARAM_INPUT, 0x114, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {43, STR("READ_OP_VA_LO"), STR("VA"), MEANWELL_ID, MB_PARAM_INPUT, 0x115, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {44, STR("READ_VBAT"), STR("mV"), MEANWELL_ID, MB_PARAM_INPUT, 0x11A, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {45, STR("READ_CHG_CURR"), STR("mA"), MEANWELL_ID, MB_PARAM_INPUT, 0x11B, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .01},
    {46, STR("BAT_CAPACITY"), STR("%%"), MEANWELL_ID, MB_PARAM_INPUT, 0x11C, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {47, STR("INV_STATUS"), STR("__"), MEANWELL_ID, MB_PARAM_INPUT, 0x11D, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {48, STR("INV_FAULT"), STR("__"), MEANWELL_ID, MB_PARAM_INPUT, 0x11E, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, 1},
    {49, STR("READ_BP_WATT_HI"), STR("W"), MEANWELL_ID, MB_PARAM_INPUT, 0x11F, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {50, STR("READ_BP_WATT_LO"), STR("W"), MEANWELL_ID, MB_PARAM_INPUT, 0x120, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {51, STR("READ_BP_VA_HI"), STR("VA"), MEANWELL_ID, MB_PARAM_INPUT, 0x125, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {52, STR("READ_BP_VA_LO"), STR("VA"), MEANWELL_ID, MB_PARAM_INPUT, 0x126, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},
    {53, STR("READ_AC_IOUT"), STR("Hz"), MEANWELL_ID, MB_PARAM_INPUT, 0x12B, 1,
     INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 2, OPTS(0x0000, 0xFFFF, 1), PAR_PERMS_READ_WRITE_TRIGGER, .1},

};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

// Modbus master initialization
esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
        .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
        .mode = MB_MODE_RTU,
#endif
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY_NONE};
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail, returns(0x%x).", (int)err);
    err = mbc_master_setup((void *)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller setup fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller set descriptor fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

// The function to get pointer to parameter storage (instance) according to parameter description table
void *master_get_param_data(const mb_parameter_descriptor_t *param_descriptor)
{
    assert(param_descriptor != NULL);
    void *instance_ptr = NULL;
    if (param_descriptor->param_offset != 0)
    {
        switch (param_descriptor->mb_param_type)
        {
        case MB_PARAM_HOLDING:
            instance_ptr = ((void *)&holding_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_INPUT:
            instance_ptr = ((void *)&input_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_COIL:
            instance_ptr = ((void *)&coil_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_DISCRETE:
            instance_ptr = ((void *)&discrete_reg_params + param_descriptor->param_offset - 1);
            break;
        default:
            instance_ptr = NULL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%u", (unsigned)param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

void set_value(int edd_val, int bates_val, uint16_t *val)
{
    // NEITHER PLUGGED IN
    if (edd_val && bates_val)
    {
        *val = 0;
        printf("Eddison and Bates register HIGH (NEITHER PLUGGED IN) \n");
    }
    //
    else if (edd_val && !bates_val)
    {
        printf("Eddison registers HIGH, Bates registers LOW (BATES PLUGGED IN)\n");
        if (INPUT_40AMP_SWITCH_VAL == 1)
        {
            *val = 4000; // writes 40amps
            printf("Writing 40 amps\n");
        }
        else if (INPUT_40AMP_SWITCH_VAL == 0 && INPUT_60AMP_SWITCH_VAL == 0)
        {
            *val = 5000; // writes 50amps
            printf("Writing 50 amps\n");
        }
        else if (INPUT_60AMP_SWITCH_VAL == 1)
        {
            *val = 6000; // writes 60amps
            printf("Writing 60 amps\n");
        }
        else
        {
            ESP_LOGE(TAG, "Error where no switch selection for bates charging is detected.");
        }
    }
    else if (!edd_val && bates_val)
    {
        *val = 1300;
        printf("Eddison registers LOW, Bates registers HIGH (EDDISON PLUGGED IN)\n");
    }
    // BOTH PLUGGED IN
    else if (!edd_val && !bates_val)
    {
        printf("Eddison and Bates registers LOW (BOTH PLUGGED IN) \n");
        if (INPUT_40AMP_SWITCH_VAL == 1)
        {
            *val = 4000; // writes 40amps
            printf("Writing 40 amps\n");
        }
        else if (INPUT_40AMP_SWITCH_VAL == 0 && INPUT_60AMP_SWITCH_VAL == 0)
        {
            *val = 5000; // writes 50amps
            printf("Writing 50 amps\n");
        }
        else if (INPUT_60AMP_SWITCH_VAL == 1)
        {
            *val = 6000; // writes 60amps
            printf("Writing 60 amps\n");
        }
        else
        {
            ESP_LOGE(TAG, "Error where no switch selection for bates charging is detected.");
        }
    }
}

void set_ssrs(int edd_val, int bates_val)
{
    // NEITHER PLUGGED IN
    if (edd_val && bates_val)
    {
        gpio_set_level(EDDISON_SSR_SELECT_IO, 1);
        gpio_set_level(BATES_SSR_SELECT_IO, 0);
        printf("Eddison and Bates register HIGH (NEITHER PLUGGED IN) \n");
    }
    //
    else if (edd_val && !bates_val)
    {
        gpio_set_level(EDDISON_SSR_SELECT_IO, 0);
        gpio_set_level(BATES_SSR_SELECT_IO, 1);
        printf("Eddison registers HIGH, Bates registers LOW (BATES PLUGGED IN)\n");
    }
    else if (!edd_val && bates_val)
    {
        gpio_set_level(EDDISON_SSR_SELECT_IO, 1);
        gpio_set_level(BATES_SSR_SELECT_IO, 0);
        printf("Eddison registers LOW, Bates registers HIGH (EDDISON PLUGGED IN)\n");
    }
    // BOTH PLUGGED IN
    else if (!edd_val && !bates_val)
    {
        gpio_set_level(EDDISON_SSR_SELECT_IO, 0);
        gpio_set_level(BATES_SSR_SELECT_IO, 1);
        printf("Eddison and Bates registers LOW (BOTH PLUGGED IN) \n");
    }
}

// User operation function to read slave values and check alarm
void master_operation_func(void *arg, int EDDISON_VAL, int BATES_VAL, OperationType op_type, Queue *queue)
{
    
    esp_err_t err = ESP_OK;
    float value = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t *param_descriptor = NULL;

    //    ESP_LOGI(TAG, "Start modbus test...");

    bool reading_from_meanwell = true;

    for (uint16_t retry = 0; retry <= MASTER_MAX_RETRY && (!alarm_state); retry++)
    {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            bool valid_push = false;
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL))
            {
                void *temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;

                DataPoint tempData;
                tempData.address = param_descriptor->mb_slave_addr;
                tempData.name = param_descriptor->param_key;
                tempData.units = param_descriptor->param_units;
                tempData.time = time(NULL);

                // // Get and print time
                // char time_string[64];
                // get_time_string(time_string, sizeof(time_string));

                if (op_type == WRITE_DATA && cid == 31)
                {
                    tempData.command = "write";
                    // Writing the parameter to slave for non-ASCII parameters
                    uint16_t value;

                    set_value(EDDISON_VAL, BATES_VAL, &value);

                    memcpy((void *)temp_data_ptr, &value, sizeof(value));
                    err = mbc_master_set_parameter(cid, (char *)param_descriptor->param_key,
                                                   (uint8_t *)temp_data_ptr, &type);

                    if (err == ESP_OK)
                    {

                        ESP_LOGI(TAG, "Characteristic #%u %s (%s) value = (0x%" PRIx16 "), write successful.",

                                 param_descriptor->cid,

                                 param_descriptor->param_key,

                                 param_descriptor->param_units,

                                 *(uint16_t *)temp_data_ptr);
                        char value_str[20];
                        snprintf(value_str, sizeof(value_str), "%u", value);
                        tempData.data = strdup(value_str);
                        set_ssrs(EDDISON_VAL, BATES_VAL);
                    }
                    else
                    {

                        ESP_LOGE(TAG, "Characteristic #%u (%s) write fail, err = 0x%x (%s).",

                                 param_descriptor->cid,

                                 param_descriptor->param_key,

                                 (int)err,

                                 (char *)esp_err_to_name(err));
                        char value_str[20];
                        snprintf(value_str, sizeof(value_str), "ERROR WRITING %u", value);
                        tempData.data = strdup(value_str);
                    }
                    valid_push = true;
                    // Read value after it's written (For Debug)
                    // err = mbc_master_get_parameter(cid, (char *)param_descriptor->param_key,
                    //                                (uint8_t *)temp_data_ptr, &type);
                    // if (err == ESP_OK)
                    // {
                    //     value = *(float *)temp_data_ptr;
                    //     printf("Reading value written: %" PRIx32 "\n", *(uint32_t *)temp_data_ptr);
                    // }
                    // else
                    // {
                    //     printf("Failed to read value written :(\n");
                    // }
                }
                else if (op_type == READ_MEANWELL && param_descriptor->mb_slave_addr == MEANWELL_ID && reading_from_meanwell)
                {
                    tempData.command = "read";

                    err = mbc_master_get_parameter(cid, (char *)param_descriptor->param_key,
                                                   (uint8_t *)temp_data_ptr, &type);
                    if (err == ESP_OK)
                    {
                        uint16_t raw_value = *(uint16_t *)temp_data_ptr;
                        float scaled_value = raw_value /** param_descriptor->scalar*/;
                        ESP_LOGI(TAG, "Characteristic #%u %s (%s) value = %.2f (0x%04X) read successful.",
                                 param_descriptor->cid,
                                 param_descriptor->param_key,
                                 param_descriptor->param_units,
                                 scaled_value,
                                 raw_value);
                        char value_str[20];
                        snprintf(value_str, sizeof(value_str), "%.2f", value);
                        tempData.data = strdup(value_str);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Characteristic #%u (%s) read fail, err = 0x%x (%s).",
                                 param_descriptor->cid,
                                 param_descriptor->param_key,
                                 (int)err,
                                 (char *)esp_err_to_name(err));
                        tempData.data = "ERROR READ";
                        if (err == ESP_ERR_TIMEOUT)
                        {
                            ESP_LOGI(TAG, "Timeout error occurred. Skipping all other CID's.");
                            reading_from_meanwell = false;
                            tempData.data = "DISCONNECTED";
                            tempData.address = -1;
                        }
                    }
                    valid_push = true;
                }
                // If we have found a CID that works
                if (valid_push)
                {
                    push(queue, createDataPoint(&tempData));
                }

                //                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        //        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS);
    }

    //    if (alarm_state) {
    //        ESP_LOGI(TAG, "Alarm triggered by cid #%u.", param_descriptor->cid);
    //    } else {
    //        ESP_LOGE(TAG, "Alarm is not triggered after %u retries.", MASTER_MAX_RETRY);
    //    }
    //    ESP_LOGI(TAG, "Destroy master...");
    //    ESP_ERROR_CHECK(mbc_master_destroy());
}

void get_time_string(char *time_str, size_t max_len)
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    strftime(time_str, max_len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

void set_time_manually(int year, int month, int day, int hour, int minute, int second)
{
    struct tm timeinfo = {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second};
    time_t t = mktime(&timeinfo);
    struct timeval now = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&now, NULL);
}
