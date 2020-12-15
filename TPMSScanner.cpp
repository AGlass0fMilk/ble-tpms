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

#include "TPMSScanner.h"

#include "gap/AdvertisingDataParser.h"

#include "mbed_trace.h"

#define TRACE_GROUP "tpms"

#define LIMIT_SIZE(x, y) ((x > y)? y : x)

#define TPMS_MAX_NAME_LENGTH 16

TPMSScanner::TPMSPacket::TPMSPacket(ble::address_t addr,
        mbed::Span<const uint8_t> manufacturer_data) : _mac_addr(addr) {
    uint32_t *tire_pressure;
    int32_t *tire_temperature;
    tire_pressure = (uint32_t*)&manufacturer_data[8];
    tire_temperature = (int32_t*)&manufacturer_data[12];
    _tire_pressure = *tire_pressure;
    _tire_temperature = *tire_temperature;
    _battery_voltage = convert_battery_voltage(manufacturer_data[16]);
}

TPMSScanner::TPMSScanner(BLE &ble) : _ble(ble) {
}

TPMSScanner::~TPMSScanner() {
}

void TPMSScanner::start(mbed::Callback<void(const TPMSPacket&)> cb) {

    /* Already started */
    if(_cb != nullptr) {
        return;
    }

    _cb = cb;


}

void TPMSScanner::stop() {
    _tpms_macs.clear();
    _cb = nullptr;
}

void TPMSScanner::onAdvertisingReport(const ble::AdvertisingReportEvent &event)
{

    ble::AdvertisingDataParser adv_parser(event.getPayload());

    /* Parse the advertising payload, looking for a TPMS beacon */
    while (adv_parser.hasNext()) {
        ble::AdvertisingDataParser::element_t field = adv_parser.next();
        const ble::address_t &addr = event.getPeerAddress();

        if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME) {

            /* Copy over the name and null-terminate it */
            char name[TPMS_MAX_NAME_LENGTH];
            uint8_t size = LIMIT_SIZE(field.value.size(),
                    (TPMS_MAX_NAME_LENGTH-1));
            memcpy(name, field.value.data(), size);
            name[size] = '\0';

            /* Check to see if the name contains "TPMS" */
            if (strstr(name, "TPMS")) {

                /* See if we've already added this TPMS beacon to our list */
                bool unique = true;
                for(auto it = _tpms_macs.begin(); it != _tpms_macs.end(); ++it) {
                    if (addr == *it) {
                        unique = false;
                        break;
                    }
                }

                if(unique) {
                    tr_info("ble-tpms - found tpms beacon: %s", name);
                    tr_info("\tbeacon addr: %02x:%02x:%02x:%02x:%02x:%02x", addr[5],
                            addr[4], addr[3], addr[2], addr[1], addr[0]);
                    tr_info("\tbeacon addr type: %i",
                            event.getPeerAddressType().value());

                    /* Add the MAC to our unique list of nearby TPMS beacon MAC addresses */
                    _tpms_macs.push_front(addr);
                }
            }
        }

        /* Copy the manufacturer data */
        if (field.type == ble::adv_data_type_t::MANUFACTURER_SPECIFIC_DATA) {
            /* Check to see if the peer is a known BLE TPMS beacon */
            for (auto it = _tpms_macs.begin(); it != _tpms_macs.end(); ++it) {
                if (addr == *it) {
                    TPMSPacket packet(addr, field.value);
                    tr_info(
                            "(%02x:%02x:%02x:%02x:%02x:%02x) pressure: %lu Pa, temp: %02f C, battery voltage: %02f V",
                            addr[5], addr[4], addr[3], addr[2], addr[1],
                            addr[0], packet.get_tire_pressure(), (float) packet.get_tire_temperature()/100.0f,
                            packet.get_battery_voltage());

                    /* Notify the application */
                    _cb(packet);
                    break;
                }
            }
        }
    }
}
