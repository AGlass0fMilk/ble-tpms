/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
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
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include "drivers/CAN.h"
#include "drivers/DigitalOut.h"
#include "drivers/Timeout.h"
#include "platform/Callback.h"
#include "platform/mbed_wait_api.h"
#include "platform/PlatformMutex.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "mbed_trace.h"

#include <chrono>

#include "TPMSScanner.h"

#define TRACE_GROUP "main"

#define BLE_TPMS_BASE_ADDR 0x1FFA0000
#define MAC_TO_CAN_ID(addr) (BLE_TPMS_BASE_ADDR | (addr[1] << 8) | addr[0])

#define PASCALS_TO_PSI(x) (x * 0.000145038f)

/**
 * Configuration preprocessors
 */

/** Scan duration */
#ifndef MBED_CONF_APP_SCAN_INTERVAL
#define MBED_CONF_APP_SCAN_INTERVAL 800
#endif

/** Scan window, must be less than scan duration! */
#ifndef MBED_CONF_APP_SCAN_WINDOW
#define MBED_CONF_APP_SCAN_WINDOW 640
#endif

/**
 * Number of TPMS sensor IDs to store in the "recently-seen" cache
 * If a TPMS packet is transmitted by this OR another TPMS receiver on the CAN bus,
 * the ID will be cached and retranmissions will be prevented for a certain
 * period of time.
 *
 * Ideally, this is large enough to store the maximum number of sensors
 * you plan to use.
 */
#define CACHE_SIZE 8

/**
 * Delay before a given TPMS sensor ID is removed from the "recently-seen" cache.
 */
#define RETRANSMIT_DELAY 1s

using namespace std::chrono;

events::EventQueue event_queue;

/** CAN bus through TCAN4551 interface */
mbed::CAN can_bus(MBED_CONF_TCAN4551_DEFAULT_TCAN_MISO,
                  MBED_CONF_TCAN4551_DEFAULT_TCAN_MOSI);

mbed::DigitalOut led1(LED1, 1);

mbed::Timeout retransmit_delay;
volatile bool retransmit_flag = true;

/** Demonstrate advertising, scanning and connecting
 */
class GapScanner : private mbed::NonCopyable<GapScanner>, public TPMSScanner
{
public:
    GapScanner(BLE& ble, events::EventQueue& event_queue) :
        TPMSScanner(ble),
        _gap(ble.gap()),
        _event_queue(event_queue) {
    }

    ~GapScanner()
    {
        if (_ble.hasInitialized()) {
            _ble.shutdown();
        }
    }

    /** Start BLE interface initialization */
    void run(mbed::Callback<void(const TPMSPacket&)> cb)
    {
        start(cb);
        if (_ble.hasInitialized()) {
            tr_err("ble instance already initialized");
            return;
        }

        /* handle gap events */
        _gap.setEventHandler(this);

        ble_error_t error = _ble.init(this, &GapScanner::on_init_complete);
        if (error) {
            tr_err("error returned by BLE::init");
            return;
        }

        /* this will not return until shutdown */
        _event_queue.dispatch_forever();
    }

private:
    /** This is called when BLE interface is initialised and starts the first mode */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event)
    {
        if (event->error) {
            tr_err("ble: error during the initialization");
            return;
        }

        /* all calls are serialised on the user thread through the event queue */
        _event_queue.call(this, &GapScanner::scan);
    }

    /** Set up and start scanning */
    void scan()
    {
        ble::scan_interval_t interval(MBED_CONF_APP_SCAN_INTERVAL);
        ble::scan_window_t window(MBED_CONF_APP_SCAN_WINDOW);

        /*
         * Scanning happens repeatedly and is defined by:
         *  - The scan interval which is the time (in 0.625us) between each scan cycle.
         *  - The scan window which is the scanning time (in 0.625us) during a cycle.
         * If the scanning process is active, the local device sends scan requests
         * to discovered peer to get additional data.
         */
        ble_error_t error = _gap.setScanParameters(
            ble::ScanParameters(
                ble::phy_t::LE_1M,
                interval,
                window,
                true
            )
        );

        if (error) {
            tr_err("error caused by Gap::setScanParameters");
            return;
        }

        /* start scanning and attach a callback that will handle advertisements
         * and scan requests responses */
        error = _gap.startScan(ble::scan_duration_t::forever());
        if (error) {
            tr_err("error caused by Gap::startScan");
            return;
        }

        tr_info("Scanning started (interval: %lums, window: %lums, timeout: never)",
               interval.valueInMs(), window.valueInMs());
    }

private:
    ble::Gap           &_gap;
    events::EventQueue &_event_queue;

};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(mbed::Callback<void()>(&context->ble, &BLE::processEvents));
}

/**
 * CAN format:
 * ID = 0x1FFA<last two octets of BLE TPMS MAC address>
 * Data = {
 *      8-bits:  Third-to-last octect of BLE TPMS MAC address
 *      unsigned 24-bits: Tire pressure in pascals
 *      signed 16-bits: Tire temperature in 0.01 celsius
 *      unsigned 16-bits:  TPMS Sensor Battery Voltage in 0.01 volts
 * }
 */
void handle_tpms(const TPMSScanner::TPMSPacket &packet) {

    if(retransmit_flag) {
        retransmit_flag = false;
        led1 = !led1;

        /* Create CAN packet */
        uint8_t data[8];

        uint32_t pressure = packet.get_tire_pressure();
        int16_t temperature = (int16_t) packet.get_tire_temperature();
        uint16_t voltage = (uint16_t)(packet.get_battery_voltage() * 100);

        data[0] = packet.get_mac_addr()[2];
        data[1] = (uint8_t)((pressure & 0xFF0000) >> 16);
        data[2] = (uint8_t)((pressure & 0x00FF00) >> 8);
        data[3] = (uint8_t)(pressure & 0x0000FF);
        data[4] = (uint8_t)((temperature & 0xFF00) >> 8);
        data[5] = (uint8_t)(temperature & 0x00FF);
        data[6] = (uint8_t)((voltage & 0xFF00) >> 8);
        data[7] = (uint8_t)(voltage & 0x00FF);

        mbed::CANMessage msg(
                MAC_TO_CAN_ID(packet.get_mac_addr()),
                data, 8, CANData, CANExtended);
        can_bus.write(msg);

        /* Delay retransmission
         * TODO - make this dependent on received TPMS CAN messages
         */
        retransmit_delay.attach(mbed::Callback<void(void)>([&](){
            retransmit_flag = true;
        }), 1000ms);
    }
}

static PlatformMutex mutex;
void mutex_wait(void) {
    mutex.lock();
}

void mutex_release(void) {
    mutex.unlock();
}

int main()
{

    /* Initialize trace */
    mbed_trace_mutex_wait_function_set(mutex_wait);
    mbed_trace_mutex_release_function_set(mutex_release);
    mbed_trace_init();

    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    GapScanner scanner(ble, event_queue);
    scanner.run(mbed::callback(handle_tpms));

    return 0;
}
