/*
 Copyright (c) 2016 "Novero GmbH" <http://novero.com>

 This file is part of multiVehicleTool <https://github.com/NoveroResearch/multiVehicleTool>.

 multiVehicleTool is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MULTIVEHICLETOOL_VEHICLESCANRESULT_H
#define MULTIVEHICLETOOL_VEHICLESCANRESULT_H

#include <bluetooth/bluetooth.h>
#include <multiVehicleTool/ankidrive.h>

struct VehicleScanResult
{
	bdaddr_t address;
	anki_vehicle_adv_t adv;
	uint8_t scan_complete;

	bool isAnkiVehicle()
	{
		return (scan_complete == 1);
	}
};

#endif
