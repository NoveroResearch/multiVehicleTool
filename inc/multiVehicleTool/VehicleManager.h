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

#ifndef MULTIVEHICLETOOL_VEHICLEMANAGER_H
#define MULTIVEHICLETOOL_VEHICLEMANAGER_H

#include <multiVehicleTool/util/Timestamp.h>
#include <multiVehicleTool/Vehicle.h>
#include <multiVehicleTool/VehicleScanResult.h>
#include <vector>

class VehicleManager
{
private:
	std::vector<std::shared_ptr<Vehicle>> vehicles_;

	bool waitForPendingConnections_{false};
	std::set<bdaddr_t> waitList_;

public:
	bool addVehicle(const std::shared_ptr<Vehicle>& newVehicle);
	bool addVehicle(const VehicleScanResult &newFoundVehicle);

	void loadVehicleList();

	void waitForPendingConnections();
	bool isWaitingForPendingConnections() const;
	void addToWaitList(bdaddr_t address);
	void removeFromWaitList(bdaddr_t address);

	std::size_t getNumVehicles() const;
	std::size_t size() const;
	std::size_t getNumConnectedVehicles() const;
	std::size_t getVehicleThatShouldConnect() const;
	std::size_t getFreeConnectionID();

	void removeDisconnectedVehicles();

	const Vehicle &operator[](std::size_t index) const;
	Vehicle &operator[](std::size_t index);

	Vehicle *getVehicleByAddress(bdaddr_t address);
	Vehicle *getVehicleByName(std::string name);
	Vehicle *getVehicleByIncompleteName(std::string name, int index);
	Vehicle *getVehicleByIndex(std::size_t index);
	std::size_t getVehicleIndexByAddress(bdaddr_t addr) const;

	static gboolean dispatchOnConnectionContinuation(gpointer userData);
	void onConnectionContinuation();
};

inline void VehicleManager::waitForPendingConnections()
{
	waitForPendingConnections_ = true;
}

inline bool VehicleManager::isWaitingForPendingConnections() const
{
	return waitForPendingConnections_;
}

inline void VehicleManager::addToWaitList(bdaddr_t address)
{
	waitList_.insert(address);
	waitForPendingConnections_ = true;
}

inline void VehicleManager::removeFromWaitList(bdaddr_t address)
{
	waitList_.erase(address);
}

inline std::size_t VehicleManager::size() const
{
	return vehicles_.size();
}

inline const Vehicle &VehicleManager::operator[](std::size_t index) const
{
	return *vehicles_[index];
}

inline Vehicle &VehicleManager::operator[](std::size_t index)
{
	return *vehicles_[index];
}

#endif
