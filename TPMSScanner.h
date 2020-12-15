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

#ifndef TPMSSCANNER_H_
#define TPMSSCANNER_H_

#include "ble/BLE.h"
#include "ble/Gap.h"

#include "platform/Span.h"
#include "platform/Callback.h"

#include <list>
#include <math.h>

class TPMSScanner : public ble::Gap::EventHandler
{

public:

    /**
     * Class representing a TPMS packet parsed from manufacturer's payload
     *
     * A few observations:
     *
     * - The last 3 octets MAC address of the TPMS sensors is printed w/ a barcode onto stickers for scanning with the mobile app
     * - The 3rd-to-last octet is always "10", "20", "30", or "40" and indicate the "wheel number" from the set it came in
     * - It may be possible to have other values in this location if the set came with more than 4 sensors
     *
     * Manufacturer packet format as reverse-engineered from Android app decompilation:
     * Bytes 0-1: Likely protocol version or manufacturer ID. App checks to make sure it is "0x0001" before parsing the rest
     * Byte 2: Tire prefix, valid values appear to be 0x80 (tire 1) to 0x83 (tire 4), also first octet of MAC address
     * Bytes 3-7: The rest of the MAC address
     * Bytes 8-11: Unsigned 32-bit tire pressure value in pascals
     * Bytes 12-15: Signed(?) 32-bit tire temperature value in Celsius (0.01C precision)
     * Byte 16: Battery voltage encoded in a really weird format
     * Byte 17: Appears to change from 0x00 to 0x01 depending on if it's "on tire" (0) or "off tire" (1), maybe an alert flag?
     *
     * Battery Voltage Decoding (based on reverse engineering app):
     * The Battery Voltage Byte (BVB) appears to be decoded using a piece-wise function as follows:
     *
     * 0 < BVB <= 4:     Voltage = (((((BVB << 16)/4*224 >> 16 + 1136) / 2) / 1023) * 3.6)
     * 5 <= BVB <= 28):  Voltage = (((((((BVB - 4) << 16) / 24) * 224) >> 16) + 1360) / 2) / 1023 * 3.6
     * 29 <= BVB <= 100: Voltage = (((((((BVB - 28) << 16) / 72) * 121) >> 16) + 1584) / 2) / 1023 * 3.6
     *
     *
     */
    class TPMSPacket {

        friend TPMSScanner;

    public:

        float get_battery_voltage() const {
            return _battery_voltage;
        }

        const ble::address_t& get_mac_addr() const {
            return _mac_addr;
        }

        uint32_t get_tire_pressure() const {
            return _tire_pressure;
        }

        int32_t get_tire_temperature() const {
            return _tire_temperature;
        }



    protected:

        /**
         * Instantiate a TPMSPacket with the manufacturer data payload
         * from scan results
         *
         * @param[in] addr MAC Address of TPMS beacon
         * @param[in] manufacturer_data Manufacturer data payload from scan results
         *
         * TODO I think the manufacturer data includes the MAC address
         * for some reason but we'll just pass it in separately for now
         */
        TPMSPacket(ble::address_t addr, mbed::Span<const uint8_t> manufacturer_data);

        /**
         * Converts the payload's battery voltage byte (BVB) to an actual voltage.
         *
         * The calculations performed are based off of reverse engineering of the TPMS
         * Android application.
         *
         * My guess is that the sensor does some sort of mapping of its own to get a "percentage"
         * based off of battery voltage. The math below undoes this mapping to get raw battery voltage.
         */
        static float convert_battery_voltage(uint8_t bvb) {
            int temp = bvb;
            if(bvb <= 4) {
                temp <<= 16;
                temp /= 4;
                temp *= 224;
                temp >>= 16;
                temp += 1136;
                temp /= 2;
            } else if (5 <= bvb && bvb <= 28) {
                temp -= 4;
                temp <<= 16;
                temp /= 24;
                temp *= 224;
                temp >>= 16;
                temp += 1360;
                temp /= 2;
            } else if (29 <= bvb && bvb <= 100) {
                temp -= 28;
                temp <<= 16;
                temp /= 72;
                temp *= 121;
                temp >>= 16;
                temp += 1584;
                temp /= 2;
            } else {
                /* Error decoding data */
                return std::nanf("");
            }
            return (float)((temp / 1023.0f) * 3.6f);
        }

    protected:

        ble::address_t _mac_addr;
        uint32_t _tire_pressure;
        int32_t _tire_temperature;
        float _battery_voltage;
    };

public:

    TPMSScanner(BLE &ble);

    virtual ~TPMSScanner();

    /**
     * Start scanning for nearby BLE TPMS beacons
     * @param[in] cb Callback executed when a TPMS packet is received
     *
     * @note The parameters passed into the given callback have temporary scope and should be
     * copied if needed later.
     *
     * @note The TPMSScanner will assume control of scanning over BLE. Attempting to scan
     * and perform connections while the TPMSScanner is running may cause unknown behavior.
     */
    void start(mbed::Callback<void(const TPMSPacket&)> cb);

    /**
     * Stop scanning for nearby BLE TPMS beacons
     */
    void stop();

protected:

    /* Gap::EventHandler */

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) override;

protected:

    BLE &_ble;

    mbed::Callback<void(const TPMSPacket&)> _cb = nullptr;

    /* List containing MAC addresses of nearby BLE TPMS beacons */
    std::list<ble::address_t> _tpms_macs;


};

#endif /* TPMSSCANNER_H_ */
