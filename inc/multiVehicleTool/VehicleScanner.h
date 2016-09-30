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

#ifndef MULTIVEHICLETOOL_VEHICLESCANNER_H
#define MULTIVEHICLETOOL_VEHICLESCANNER_H

#include <vector>
#include <assert.h>
#include <multiVehicleTool/VehicleManager.h>
#include <multiVehicleTool/VehicleScanResult.h>

class VehicleScanner
{
private:
	std::vector<VehicleScanResult> scanResults_;

	int parseAdvertisingDevices(int dd);
	VehicleScanResult *getOrAddDevice(const bdaddr_t &address);
	VehicleScanResult *getDeviceWithAddress(const bdaddr_t &address);

public:
	void scan(int devId);
	void updateVehicleManager(VehicleManager &manager);

	std::size_t size() const;

	const VehicleScanResult &operator[](std::size_t index) const;
	VehicleScanResult &operator[](std::size_t index);

	VehicleScanResult *getDeviceByIndex(std::size_t index);

	void clear();
};

inline const VehicleScanResult &VehicleScanner::operator[](std::size_t index) const
{
	return scanResults_[index];
}

inline VehicleScanResult &VehicleScanner::operator[](std::size_t index)
{
	return scanResults_[index];
}

inline VehicleScanResult *VehicleScanner::getDeviceByIndex(std::size_t index)
{
	if (scanResults_.size() > index)
		return &scanResults_[index];

	return nullptr;
}

inline std::size_t VehicleScanner::size() const
{
	return scanResults_.size();
}

inline void VehicleScanner::clear()
{
	scanResults_.clear();
}

#endif
