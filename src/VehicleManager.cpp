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

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>

#include <bluetooth/bluetooth.h>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/CommandPrompt.h>
#include <multiVehicleTool/VehicleManager.h>

#include <limits>
#include <string>
#include <iostream>

void VehicleManager::loadVehicleList()
{
	std::string path;
	boost::property_tree::ptree pt;

	if (boost::filesystem::exists("vehiclePoolDefaults.json"))
		path = "vehiclePoolDefaults.json";
	else
	{
		std::cout << "vehicle configuration not found" << std::endl;
		return;
	}

	try
	{
		boost::property_tree::json_parser::read_json(path, pt);
	}
	catch (boost::property_tree::ptree_error &err)
	{
		std::cout << "error opening " << path << std::endl;
		return;
	}

	for (auto it : pt)
	{
		auto newVehicle = std::make_shared<Vehicle>();

		if (str2ba(it.first.c_str(), &newVehicle->address_) != 0)
			continue;

		if (it.second.get_optional<std::string>("name"))
			newVehicle->name_ = it.second.get<std::string>("name");
		else
			continue;

		if (it.second.get_optional<int>("id"))
			newVehicle->model_ = VehicleModel(it.second.get<int>("ankiVehicleType"));

		addVehicle(newVehicle);
	}
}

bool VehicleManager::addVehicle(const std::shared_ptr<Vehicle>& newVehicle)
{
	if (getVehicleByAddress(newVehicle->address_) != nullptr)
		return false;
	if (newVehicle->manager_ != nullptr)
		return false;

	vehicles_.push_back(newVehicle);
	vehicles_.back()->manager_ = this;

	return true;
}

bool VehicleManager::addVehicle(const VehicleScanResult &newFoundVehicle)
{
	if (getVehicleByAddress(newFoundVehicle.address) != nullptr)
		return false;

	auto newVehicle = std::make_shared<Vehicle>();
	newVehicle->address_ = newFoundVehicle.address;
	newVehicle->name_ = std::string(reinterpret_cast<const char *>(newFoundVehicle.adv.local_name.name));
	newVehicle->model_ = VehicleModel(newFoundVehicle.adv.mfg_data.model_id);
	newVehicle->version_ = newFoundVehicle.adv.local_name.version;

	return addVehicle(newVehicle);
}

std::size_t VehicleManager::getNumVehicles() const
{
	return vehicles_.size();
}

std::size_t VehicleManager::getFreeConnectionID()
{
	std::size_t id;
	for (id = 0; id < 256; ++id)
	{
		bool idFound = false;
		for (auto &vehicle : vehicles_)
		{
			if (vehicle->connectionState_ == STATE_CONNECTED)
			{
				if (vehicle->connectionID_ == id)
				{
					idFound = true;
					break;
				}
			}
		}

		if (!idFound)
		{
			return id;
		}
	}
	return id;
}

std::size_t VehicleManager::getNumConnectedVehicles() const
{
	std::size_t connectedVehicles = 0;
	for (auto &vehicle : vehicles_)
	{
		if (vehicle->connectionState_ == STATE_CONNECTED)
		{
			connectedVehicles++;
		}
	}
	return connectedVehicles;
}

std::size_t VehicleManager::getVehicleThatShouldConnect() const
{
	static std::size_t i = 0;
	std::size_t steps = 0;

	while (steps < vehicles_.size())
	{
		if (i >= vehicles_.size())
			i = 0;

		if (vehicles_[i]->shouldConnect())
		{
			i++;
			return i - 1;
		}
		steps++;
		i++;
	}

	return std::numeric_limits<std::size_t>::max();
}

void VehicleManager::removeDisconnectedVehicles()
{
	for (auto it = vehicles_.begin(); it != vehicles_.end();)
	{
		if ((*it)->connectionState_ == STATE_DISCONNECTED)
			it = vehicles_.erase(it);
		else
			++it;
	}
}

Vehicle *VehicleManager::getVehicleByAddress(bdaddr_t address)
{
	for (auto &vehicle : vehicles_)
	{
		if (vehicle->address_ == address)
			return vehicle.get();
	}

	return nullptr;
}

Vehicle *VehicleManager::getVehicleByName(std::string name)
{
	for (auto &vehicle : vehicles_)
	{
		if (vehicle->name_ == name)
			return vehicle.get();
	}

	return nullptr;
}

Vehicle *VehicleManager::getVehicleByIncompleteName(std::string name, int index)
{
	int count = 0;
	for (auto &vehicle : vehicles_)
	{
		if (vehicle->name_.find(name) == 0)
		{
			if (count == index)
				return vehicle.get();
			else
				count++;
		}
	}

	return nullptr;
}

Vehicle *VehicleManager::getVehicleByIndex(std::size_t index)
{
	if (vehicles_.size() > index)
		return vehicles_[index].get();

	return nullptr;
}

std::size_t VehicleManager::getVehicleIndexByAddress(bdaddr_t address) const
{
	for (std::size_t i = 0; i < vehicles_.size(); ++i)
	{
		if (vehicles_[i]->address_ == address)
			return i;
	}

	return std::numeric_limits<std::size_t>::max();
}

gboolean VehicleManager::dispatchOnConnectionContinuation(gpointer userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering VehicleManager::dispatchOnConnectionContinuation...\n");

	assert(userData);
	VehicleManager *vehicleManager = static_cast<VehicleManager *>(userData);
	vehicleManager->onConnectionContinuation();

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting VehicleManager::dispatchOnConnectionContinuation...\n");

	return G_SOURCE_REMOVE;
}

void VehicleManager::onConnectionContinuation()
{
	bool verbose{false};

	// Determine next vehicle to connect.
	std::size_t index = getVehicleThatShouldConnect();
	if (index < size())
	{
		// Determine free HCI device.
		HciDevice *hciDevice = HciManager::getFreeAndUnblockedDevice();

		if (!hciDevice)
		{
			if (!HciManager::getFreeDevice())
			{
				if (HciManager::getNumberOfHciDevices() == 0)
					rl_printf(COLOR_RED "Error: " COLOR_OFF "No HCI devices available. Giving up on pending connections.\n");
				else
					rl_printf(COLOR_RED "Error: " COLOR_OFF "All HCI devices already have the maximum number of established connections. Giving up on pending connections.\n");

				// Disconnect all remaining vehicles.
				for (std::size_t i = getVehicleThatShouldConnect(); i < size(); i = getVehicleThatShouldConnect())
					getVehicleByIndex(i)->disconnect();

				// Wait until all connects are done before setting up standard input again.
				if (waitForPendingConnections_ && waitList_.empty())
				{
					CommandPrompt::getInstance().updatePrompt();
					CommandPrompt::getInstance().setupStandardInput();

					waitForPendingConnections_ = false;
				}
			}
			else
			{
				if (verbose)
					rl_printf(COLOR_YELLOW "Warning: " COLOR_OFF "All HCI devices are occupied. Waiting for device to become unblocked.\n");
			}
		}
		else
		{
			getVehicleByIndex(index)->connectNow(hciDevice);
		}
	}
	else if (waitForPendingConnections_ && waitList_.empty())
	{
		rl_printf(COLOR_GREEN "All vehicles connected." COLOR_OFF "\n");

		CommandPrompt::getInstance().updatePrompt();
		CommandPrompt::getInstance().setupStandardInput();

		waitForPendingConnections_ = false;
	}
}
