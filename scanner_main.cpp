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

// TODO look into CMAKE build system
// This may enable building the main app, tester app, and bootloader from the same repository...

#if !MBED_CONF_APP_TESTER_BUILD

#include <events/mbed_events.h>
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
#include "rtos/Kernel.h"
#include "TPMSScanner.h"

#include "drivers/SPI.h"

#include "TCAN4551.h"

#define TRACE_GROUP "main"

using namespace std::chrono;

events::EventQueue event_queue;

TCAN4551 can_bus(TCAN4551_MOSI, TCAN4551_MISO, TCAN4551_SCLK, TCAN4551_CSN, TCAN4551_NINT);

mbed::DigitalOut led1(LED1, 1);

mbed::Timeout retransmit_delay;
volatile bool retransmit_flag = true;

static int tpms_can_filter_handle = 0;

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

    void stop() {
        _event_queue.break_dispatch();
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
#if defined(TARGET_CALADESI)
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
#endif
}

static PlatformMutex mutex;
void mutex_wait(void) {
    mutex.lock();
}

void mutex_release(void) {
    mutex.unlock();
}

/**
 * Trace Prefix Format
 */
char* trace_prefix(size_t sz) {
    static char prefix[16];
    snprintf(prefix, 16, "[%llu]", rtos::Kernel::Clock::now());
    return prefix;
}

void handle_can_rx() {
    tr_info("handling received CAN message...");
    mbed::CANMessage msg;
    if(can_bus.read(msg, tpms_can_filter_handle)) {
        /* We want to avoid retransmitting packets sent out by other receivers on the same bus */

    }
}

int main()
{

    /* Initialize trace */
    mbed_trace_mutex_wait_function_set(mutex_wait);
    mbed_trace_mutex_release_function_set(mutex_release);
    mbed_trace_prefix_function_set(trace_prefix);
    mbed_trace_init();

    /* Add a filter for other BLE TPMS  receiver CAN packets */
    tpms_can_filter_handle = can_bus.filter(BLE_TPMS_BASE_ADDR, BLE_TPMS_BASE_ADDR_MASK, CANExtended, 0);

    /* Attach interrupt handler for CAN RX events */
    can_bus.attach(mbed::callback([&]() {
        event_queue.call(handle_can_rx);
    }));

    BLE &ble = BLE::Instance();

    ble.onEventsToProcess(schedule_ble_events);

    GapScanner scanner(ble, event_queue);

    // TODO remove this -- proper way of shutting down
    //event_queue.call_in(4min, mbed::callback(&scanner, &GapScanner::stop));

    scanner.run(mbed::callback(handle_tpms));

    /*
     * Go to sleep
     * Since the TCAN455x powers the MCU from its internal LDO, this will
     * power down the MCU as well.
     */
    //can_bus.sleep();

    /* Spin until we're asleep :) */
//    while(true) {
//    }

    return 0;
}

#endif
