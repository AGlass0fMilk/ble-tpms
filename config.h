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

#ifndef CONFIG_H_
#define CONFIG_H_


#define BLE_TPMS_BASE_ADDR 0x1FFA0000
#define BLE_TPMS_BASE_ADDR_MASK 0xFFFF0000
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



#endif /* CONFIG_H_ */
