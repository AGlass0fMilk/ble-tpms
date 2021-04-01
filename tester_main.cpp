/*
 * Copyright (c) 2020 George Beckstein
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
 */

#if MBED_CONF_APP_TESTER_BUILD

#include "events/EventQueue.h"
#include "drivers/DigitalOut.h"
#include "drivers/Timeout.h"
#include "platform/Callback.h"
#include "platform/mbed_wait_api.h"
#include "platform/PlatformMutex.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "mbed_trace.h"

#include "config.h"

#include <chrono>

#include "TPMSScanner.h"

#if defined(TARGET_CALADESI)
#include "drivers/CAN.h"
#endif

#define TRACE_GROUP "main"

using namespace std::chrono;

events::EventQueue event_queue;

#if defined(TARGET_CALADESI)
/** CAN bus through TCAN4551 interface */
mbed::CAN can_bus(MBED_CONF_TCAN4551_DEFAULT_TCAN_MISO,
                  MBED_CONF_TCAN4551_DEFAULT_TCAN_MOSI);
#endif

mbed::DigitalOut led1(LED1, 1);

static PlatformMutex mutex;
void mutex_wait(void) {
    mutex.lock();
}

void mutex_release(void) {
    mutex.unlock();
}

const static char DEVICE_NAME[] = "TPMS0_ABCDEF";

using namespace std::literals::chrono_literals;

class TPMSBeacon : ble::Gap::EventHandler {
public:
    TPMSBeacon(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _pressure(150000),
        _temp(25000),
        _bat(66),
        _adv_data_builder(_adv_buffer)
    {
    }

    void start()
    {
        /* mbed will call on_init_complete when when ble is ready */
        _ble.init(this, &TPMSBeacon::on_init_complete);

        /* this will never return */
        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params)
    {
        if (params->error != BLE_ERROR_NONE) {
            tr_err("Ble initialization failed (%d)", params->error);
            return;
        }

        start_advertising();
    }

    void start_advertising()
    {
        /* create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            /* you cannot connect to this device, you can only read its advertising data,
             * scannable means that the device has extra advertising data that the peer can receive if it
             * "scans" it which means it is using active scanning (it sends a scan request) */
            ble::advertising_type_t::SCANNABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setName(DEVICE_NAME);

        /* The BLE TPMS beacons advertise a service with the UUID 0xFBB0 */
        const UUID tpms_svc_uuid(0xFBB0);
        _adv_data_builder.setLocalServiceList(mbed::make_const_Span<const UUID>(&tpms_svc_uuid, 1), true);

        /* setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            tr_err("Gap::setAdvertisingParameters() failed (%d)", error);
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            tr_err("Gap::setAdvertisingPayload() failed (%d)", error);
            return;
        }

        /* when advertising you can optionally add extra data that is only sent
         * if the central requests it by doing active scanning */
        _adv_data_builder.clear();
        ble::address_t address;
        ble::own_address_type_t addr_type;
        _ble.gap().getAddress(addr_type, address);
        TPMSScanner::TPMSPacket tpms_packet(address, _pressure, _temp, _bat);

        _adv_data_builder.setManufacturerSpecificData(tpms_packet.get_payload());

        _ble.gap().setAdvertisingScanResponse(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        /* start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            tr_err("Gap::startAdvertising() failed (%d)", error);
            return;
        }

        /* we simulate battery discharging by updating it every second */
        _event_queue.call_every(
            1000ms,
            [this]() {
                update_tpms_data();
            }
        );
    }

    void update_tpms_data()
    {

        _pressure -= 1000;
        _temp -= 100;
        _bat--;
        if(_pressure < 10000) {
            _pressure = 200000;
        }

        if(_temp < 1000) {
            _temp = 32000;
        }

        if(_bat < 3) {
            _bat = 80;
        }

        /* update the payload with the new value */
        ble::address_t address;
        ble::own_address_type_t addr_type;
        _ble.gap().getAddress(addr_type, address);
        TPMSScanner::TPMSPacket tpms_packet(address, _pressure, _temp, _bat);
        ble_error_t error = _adv_data_builder.setManufacturerSpecificData(tpms_packet.get_payload());

        if (error) {
            tr_err("_adv_data_builder::setServiceData() failed (%d)", error);
            return;
        }

        /* set the new payload, we don't need to stop advertising */
        error = _ble.gap().setAdvertisingScanResponse(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            tr_err("Gap::setAdvertisingPayload() failed (%d)", error);
            return;
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    uint32_t _pressure;
    int32_t _temp;
    uint8_t _bat;


    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/* Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(mbed::Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{

    /* Initialize trace */
    mbed_trace_mutex_wait_function_set(mutex_wait);
    mbed_trace_mutex_release_function_set(mutex_release);
    mbed_trace_init();

    tr_info("TPMS tester begin");

    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    TPMSBeacon beacon(ble, event_queue);
    beacon.start();

    return 0;
}

#endif

