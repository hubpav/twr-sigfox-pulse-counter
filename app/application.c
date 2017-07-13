#include <application.h>

#define SIGFOX_FIRST_REPORT_SECONDS 10
#define SIGFOX_REPORT_MINUTES (60 * 60)
#define BATTERY_MODULE_UPDATE_INTERVAL 100 // (10 * 1000)
#define SENSOR_DATA_STREAM_SAMPLES 6

// Define custom frame to access sigfox buffer
#pragma pack(1)
typedef struct
{
    uint8_t channel_count_a_overflow;
	uint32_t channel_count_a;
    uint8_t channel_count_b_overflow;
	uint32_t channel_count_b;
	uint16_t battery_voltage_mv;
} buffer_frame_t;
#pragma pack()

buffer_frame_t buffer_framed;

// Data stream instance
BC_DATA_STREAM_INT_BUFFER(stream_buffer_battery_voltage_mv, SENSOR_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_battery_voltage_mv;

// LED instance
bc_led_t led;

// SigFox Module instance
bc_module_sigfox_t sigfox_module;

// Overflow count
uint8_t channel_a_overflow_count;
uint8_t channel_b_overflow_count;

void pulse_counter_event_handler(bc_module_sensor_channel_t channel, bc_pulse_counter_event_t event, void *event_param);
void battery_module_event_handler(bc_module_battery_event_t event, void *event_param);
void sigfox_module_event_handler(bc_module_sigfox_t *self, bc_module_sigfox_event_t event, void *event_param);

void application_init(void)
{
    // Initialize stream_battery_voltage
    bc_data_stream_init(&stream_battery_voltage_mv, SENSOR_DATA_STREAM_SAMPLES, &stream_buffer_battery_voltage_mv);

	// Initialize LED
	bc_led_init(&led, BC_GPIO_LED, false, false);
	bc_led_set_mode(&led, BC_LED_MODE_FLASH);

	// Initialize counter on Module Sensor channels
	bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
	bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, pulse_counter_event_handler, NULL);
	bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_B, BC_PULSE_COUNTER_EDGE_FALL);
    bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_B, pulse_counter_event_handler, NULL);

	// Initialize counter on Module Battery
	bc_module_battery_init(BC_MODULE_BATTERY_FORMAT_MINI);
	bc_module_battery_set_update_interval(BATTERY_MODULE_UPDATE_INTERVAL);
    bc_module_battery_set_event_handler(battery_module_event_handler, NULL);

	// Initialize SigFox Module
	bc_module_sigfox_init(&sigfox_module, BC_MODULE_SIGFOX_REVISION_R2);
	bc_module_sigfox_set_event_handler(&sigfox_module, sigfox_module_event_handler, NULL);

	// Plan application_task
	bc_scheduler_plan_absolute(0, SIGFOX_FIRST_REPORT_SECONDS * 1000);
}

void application_task(void *param)
{
	(void) param;

	if (!bc_module_sigfox_is_ready(&sigfox_module))
	{
		bc_scheduler_plan_current_now();
		return;
	}

	int battery_voltage_mv;
	bc_data_stream_get_average(&stream_battery_voltage_mv, &battery_voltage_mv);

	// Fill sigfox buffer
	buffer_framed.channel_count_a = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
	buffer_framed.channel_count_b = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_B);
	buffer_framed.battery_voltage_mv = battery_voltage_mv;

	if (bc_module_sigfox_send_rf_frame(&sigfox_module, &buffer_framed, sizeof(buffer_framed)))
	{
		bc_scheduler_plan_current_relative(SIGFOX_REPORT_MINUTES * 1000);
	}
	else
	{
		bc_scheduler_plan_current_relative(1000);
	}
}

void pulse_counter_event_handler(bc_module_sensor_channel_t channel, bc_pulse_counter_event_t event, void *event_param)
{
    (void) event_param;

    if (event == BC_PULSE_COUNTER_EVENT_OVERFLOW)
    {
        if (channel == BC_MODULE_SENSOR_CHANNEL_A)
        {
            buffer_framed.channel_count_a_overflow++;
        }
        else
        {
            buffer_framed.channel_count_b_overflow++;
        }
    }
}

void battery_module_event_handler(bc_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    if(event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        float battery_voltage;  bc_module_battery_update_voltage_on_battery(&battery_voltage);
        int battery_voltage_mv = battery_voltage * 1000;

        bc_data_stream_feed(&stream_battery_voltage_mv, &battery_voltage_mv);
    }
    else
    {
        bc_data_stream_reset(&stream_battery_voltage_mv);
    }
}

void sigfox_module_event_handler(bc_module_sigfox_t *self, bc_module_sigfox_event_t event, void *event_param)
{
	(void) self;
	(void) event_param;

	if (event == BC_MODULE_SIGFOX_EVENT_SEND_RF_FRAME_START)
	{
		bc_led_set_mode(&led, BC_LED_MODE_ON);
	}
	else if (event == BC_MODULE_SIGFOX_EVENT_SEND_RF_FRAME_DONE)
	{
		bc_led_set_mode(&led, BC_LED_MODE_OFF);
	}
	else if (event == BC_MODULE_SIGFOX_EVENT_ERROR)
	{
		bc_led_set_mode(&led, BC_LED_MODE_BLINK);
	}
}
