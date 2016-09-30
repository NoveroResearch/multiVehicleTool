/*
 * Copyright (c) 2014 Anki, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MULTIVEHICLETOOL_UTIL_ANKIDRIVEPROTOCOL_H
#define MULTIVEHICLETOOL_UTIL_ANKIDRIVEPROTOCOL_H

#include <ankidrive/version.h>
#if (ANKIDRIVESDK_VER_MAJOR == 0 && ANKIDRIVESDK_VER_MINOR < 3)
#error "Anki Drive-SDK must be at least version 0.3.0"
#endif
#include <ankidrive/protocol.h>


typedef struct anki_drive_vehicle_msg_localization_position_update {
    uint8_t     size;
    uint8_t     msg_id;
    uint8_t     _reserved[2];
    float       offset_from_road_center_mm;
    uint16_t    speed_mm_per_sec;
    uint8_t     is_clockwise;
} ATTRIBUTE_PACKED anki_drive_vehicle_msg_localization_position_update_t;
#define ANKI_DRIVE_VEHICLE_MSG_V2C_LOCALIZATION_POSITION_UPDATE_SIZE  10

typedef struct anki_drive_vehicle_msg_localization_transition_update {
    uint8_t     size;
    uint8_t     msg_id;
    uint8_t     _reserved;
    float       offset_from_road_center_mm;
    uint8_t     is_clockwise;
} ATTRIBUTE_PACKED anki_drive_vehicle_msg_localization_transition_update_t;
#define ANKI_DRIVE_VEHICLE_MSG_V2C_LOCALIZATION_TRANSITION_UPDATE_SIZE  7

/**
 * Create a message to request a 180 degree turn for the Drive firmware.
 *
 * @param msg A pointer to the vehicle message struct to be written.
 *
 * @return size of bytes written to msg
 */
uint8_t anki_drive_vehicle_msg_turn_180(anki_vehicle_msg_t *msg);

#endif
