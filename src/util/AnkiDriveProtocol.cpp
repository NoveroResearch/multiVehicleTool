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

#include <stdio.h>
#include <assert.h>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/util/AnkiDriveProtocol.h>
#include <string.h>

#define ANKI_VEHICLE_MSG_TYPE_SIZE  2

uint8_t anki_drive_vehicle_msg_turn_180(anki_vehicle_msg_t *msg)
{
    assert(msg != NULL);
    msg->size = ANKI_VEHICLE_MSG_BASE_SIZE;
    msg->msg_id = ANKI_VEHICLE_MSG_C2V_TURN;
    return ANKI_VEHICLE_MSG_TYPE_SIZE;
}

