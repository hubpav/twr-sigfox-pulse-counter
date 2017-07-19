#include <application.h>

#define BATTERY_VOLTAGE_DATA_STREAM_SAMPLES  6
#define SIGFOX_FIRST_REPORT_SECONDS (10)
#define SIGFOX_REPORT_INTERVAL_SECONDS (60 * 60)
#define BATTERY_MODULE_UPDATE_INTERVAL_SECONDS (SIGFOX_REPORT_INTERVAL_SECONDS / BATTERY_VOLTAGE_DATA_STREAM_SAMPLES )

#define HEADER_EVENT_ERROR 0xff
#define HEADER_EVENT_UPDATE 0x00

BC_DATA_STREAM_INT_BUFFER(stream_buffer_battery_voltage_mv, BATTERY_VOLTAGE_DATA_STREAM_SAMPLES )
bc_data_stream_t stream_battery_voltage_mv;

bc_led_t led;

bc_module_sigfox_t sigfox_module;

uint8_t header = HEADER_EVENT_UPDATE;

unsigned int channel_a_overflow_count = 0;
unsigned int channel_b_overflow_count = 0;

void pulse_counter_event_handler(bc_module_sensor_channel_t channel, bc_pulse_counter_event_t event, void *event_param)
{
    (void) event_param;

    if (event == BC_PULSE_COUNTER_EVENT_OVERFLOW)
    {
        if (channel == BC_MODULE_SENSOR_CHANNEL_A)
        {
            channel_a_overflow_count++;
        }
        else
        {
            channel_b_overflow_count++;
        }
    }
}

void battery_module_event_handler(bc_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        float battery_voltage;
        int battery_voltage_mv;

        bc_module_battery_update_voltage_on_battery(&battery_voltage);
        battery_voltage_mv = battery_voltage * 1000;

        bc_data_stream_feed(&stream_battery_voltage_mv, &battery_voltage_mv);
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

void application_init(void)
{
    bc_data_stream_init(&stream_battery_voltage_mv, 1, &stream_buffer_battery_voltage_mv);

	bc_led_init(&led, BC_GPIO_LED, false, false);
	bc_led_set_mode(&led, BC_LED_MODE_FLASH);

	bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
	bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, pulse_counter_event_handler, NULL);
	bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_B, BC_PULSE_COUNTER_EDGE_FALL);
    bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_B, pulse_counter_event_handler, NULL);

	bc_module_battery_init(BC_MODULE_BATTERY_FORMAT_MINI);
	bc_module_battery_set_update_interval(BATTERY_MODULE_UPDATE_INTERVAL_SECONDS * 1000);
    bc_module_battery_set_event_handler(battery_module_event_handler, NULL);

	bc_module_sigfox_init(&sigfox_module, BC_MODULE_SIGFOX_REVISION_R1);
	bc_module_sigfox_set_event_handler(&sigfox_module, sigfox_module_event_handler, NULL);

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

    uint8_t buffer[12];
    uint32_t channel_count_a;
    uint32_t channel_count_b;
    int battery_voltage;
	uint16_t battery_voltage_mv;

    channel_count_a = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
    channel_count_b = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_B);
	bc_data_stream_get_average(&stream_battery_voltage_mv, &battery_voltage);

	if ((channel_a_overflow_count > 0xff) || (channel_b_overflow_count > 0xff))
	{
	    header = HEADER_EVENT_ERROR;
	}

	battery_voltage_mv = (uint16_t)battery_voltage;

    buffer[0] = header;
    buffer[1] = (((channel_a_overflow_count << 4) & 0xf0) | ((channel_b_overflow_count) & 0x0f));
    buffer[2] = channel_count_a;
    buffer[3] = channel_count_a >> 8;
    buffer[4] = channel_count_a >> 16;
    buffer[5] = channel_count_a >> 24;
    buffer[6] = channel_count_b;
    buffer[7] = channel_count_b >> 8;
    buffer[8] = channel_count_b >> 16;
    buffer[9] = channel_count_b >> 24;
    buffer[10] = battery_voltage_mv;
    buffer[11] = battery_voltage_mv >> 8;

	if (bc_module_sigfox_send_rf_frame(&sigfox_module, &buffer, sizeof(buffer)))
	{
		bc_scheduler_plan_current_relative(SIGFOX_REPORT_INTERVAL_SECONDS * 1000);
	}
	else
	{
		bc_scheduler_plan_current_relative(1000);
	}
}
