/****************************************************************************
 *
 *   Copyright (C) 2012-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <uORB/Subscription.hpp>

#include "SystemPower.hpp"

SystemPower::SystemPower() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SystemPower::~SystemPower()
{
	DataReadyInterruptDisable();
	ScheduleClear();

	perf_free(_cycle_perf);
}

int SystemPower::init()
{
	DataReadyInterruptConfigure();
	ScheduleDelayed(1_s);
	return PX4_OK;
}

int SystemPower::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SystemPower *>(arg)->DataReady();
	return 0;
}

void SystemPower::DataReady()
{
	ScheduleNow();
}

bool SystemPower::DataReadyInterruptConfigure()
{
	// Setup data ready on rising and falling edge
#if defined(GPIO_OTGFS_VBUS)
	px4_arch_gpiosetevent(GPIO_OTGFS_VBUS, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_OTGFS_VBUS

#if defined(GPIO_nVDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_USB_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_USB_VALID

#if defined(GPIO_VDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_USB_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_USB_VALID

#if defined(GPIO_VDD_SERVO_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_SERVO_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_SERVO_VALID

#if defined(GPIO_nVDD_5V_PERIPH_OC)
	px4_arch_gpiosetevent(GPIO_nVDD_5V_PERIPH_OC, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_5V_PERIPH_OC

#if defined(GPIO_VDD_5V_HIPOWER_OC)
	px4_arch_gpiosetevent(GPIO_VDD_5V_HIPOWER_OC, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_5V_HIPOWER_OC

#if defined(GPIO_nVDD_BRICK1_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK1_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK1_VALID

#if defined(GPIO_nVDD_BRICK2_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK2_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK2_VALID

#if defined(GPIO_nVDD_BRICK3_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK3_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK3_VALID

#if defined(GPIO_nVDD_BRICK4_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK4_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK4_VALID

	return true;
}

bool SystemPower::DataReadyInterruptDisable()
{
#if defined(GPIO_OTGFS_VBUS)
	px4_arch_gpiosetevent(GPIO_OTGFS_VBUS, false, false, false, nullptr, nullptr);
#endif // GPIO_OTGFS_VBUS

#if defined(GPIO_nVDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_USB_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_USB_VALID

#if defined(GPIO_VDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_USB_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_USB_VALID

#if defined(GPIO_VDD_SERVO_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_SERVO_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_SERVO_VALID

#if defined(GPIO_nVDD_5V_PERIPH_OC)
	px4_arch_gpiosetevent(GPIO_nVDD_5V_PERIPH_OC, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_5V_PERIPH_OC

#if defined(GPIO_VDD_5V_HIPOWER_OC)
	px4_arch_gpiosetevent(GPIO_VDD_5V_HIPOWER_OC, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_5V_HIPOWER_OC

#if defined(GPIO_nVDD_BRICK1_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK1_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK1_VALID

#if defined(GPIO_nVDD_BRICK2_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK2_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK2_VALID

#if defined(GPIO_nVDD_BRICK3_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK3_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK3_VALID

#if defined(GPIO_nVDD_BRICK4_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK4_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK4_VALID

	return true;
}

void SystemPower::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	ScheduleDelayed(500_ms);

	perf_begin(_cycle_perf);

	adc_report_s adc_report{};

	_adc_report_sub.copy(&adc_report);

	{
#if defined (BOARD_ADC_USB_CONNECTED)
		system_power_s system_power {};

		/* Assume HW provides only ADC_SCALED_V5_SENSE */
		int cnt = 1;
		/* HW provides both ADC_SCALED_V5_SENSE and ADC_SCALED_V3V3_SENSORS_SENSE */
#  if defined(ADC_SCALED_V5_SENSE) && defined(ADC_SCALED_V3V3_SENSORS_SENSE)
		cnt++;
#  endif

		uint32_t channel_count = 0;

		for (unsigned i = 0; i < channel_count; i++) {
#  if defined(ADC_SCALED_V5_SENSE)

			if (adc_report.channel_id[i] == ADC_SCALED_V5_SENSE) {
				// it is 2:1 scaled
				system_power.voltage5v_v = adc_report.raw_data[i] * (ADC_V5_V_FULL_SCALE / adc_report.resolution);
				cnt--;

			} else
#  endif

			{
#  if defined(ADC_SCALED_V3V3_SENSORS_SENSE)

				if (adc_report.channel_id[i] == ADC_SCALED_V3V3_SENSORS_SENSE) {
					// it is 2:1 scaled
					system_power.voltage3v3_v = adc_report.raw_data[i]  * (ADC_3V3_SCALE * (3.3f / adc_report.resolution));
					system_power.v3v3_valid = true;
					cnt--;
				}

#  endif
			}

			if (cnt == 0) {
				break;
			}
		}

		/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
		 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
		 */
		// these are not ADC related, but it is convenient to
		// publish these to the same topic

		system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
		/* If provided used the Valid signal from HW*/
#if defined(BOARD_ADC_USB_VALID)
		system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
		/* If not provided then use connected */
		system_power.usb_valid = system_power.usb_connected;
#endif

#if defined(BOARD_BRICK_VALID_LIST)
		/* The valid signals (HW dependent) are associated with each brick */
		bool valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
		system_power.brick_valid = 0;

		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
			system_power.brick_valid |= valid_chan[b] ? 1 << b : 0;
		}

#endif

#if defined(BOARD_ADC_SERVO_VALID)
		system_power.servo_valid = BOARD_ADC_SERVO_VALID;
#endif

#if defined(BOARD_ADC_PERIPH_5V_OC)
		// OC pins are active low
		system_power.periph_5v_oc = BOARD_ADC_PERIPH_5V_OC;
#endif

#if defined(BOARD_ADC_HIPOWER_5V_OC)
		system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;
#endif

		system_power.timestamp = hrt_absolute_time();
		_to_system_power.publish(system_power);

#endif // BOARD_ADC_USB_CONNECTED
	}

	perf_end(_cycle_perf);
}

int SystemPower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SystemPower::task_spawn(int argc, char *argv[])
{
	SystemPower *instance = new SystemPower();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SystemPower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("system_power", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int system_power_main(int argc, char *argv[])
{
	return SystemPower::main(argc, argv);
}
