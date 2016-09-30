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

#ifndef MULTIVEHICLETOOL_HCIMANAGER_H_
#define MULTIVEHICLETOOL_HCIMANAGER_H_

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/uuid.h>

#include <vector>
#include <cassert>
#include <cstddef>
#include <climits>
#include <algorithm>

void rl_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

struct knownHciDevice
{
	union
	{
		uint8_t addressArray_[6];
		bdaddr_t address_;
	};
	size_t maxConnections_;
	size_t maxUsableConnections_;
};

static const int numKnownHciDevices_ = 5;

// clang-format off
static knownHciDevice knownHciDevices_[numKnownHciDevices_] =
{
    { 	// LogLink (CSR8510 A10)
		{{0x00, 0x00, 0x00, 0x7D, 0x1A, 0x00}},
		5,
		5
    },
    { 	// Broadcom BCM20701 A0
		{{0x00, 0x00, 0x00, 0x70, 0xF3, 0x5C}},
		14,
		8
    },
    { 	// Apple MacBook internal (Broadcom)
		{{0x00, 0x00, 0x00, 0x08, 0x40, 0x6C}},
		15,
		12
    },
    { 	// Apple MacBook internal (Broadcom)
		{{0x00, 0x00, 0x00, 0x3B, 0x36, 0x34}},
		15,
		12
    },
    { 	// Lenovo Thinkpad internal (Intel chipset)
		{{0x00, 0x00, 0x00, 0xD4, 0xC5, 0x5C}},
		7,
		5
    }
};
// clang-format on

struct HciDevice
{
	int devId_;
	bdaddr_t address_;
	std::size_t use_;
	std::size_t maxUse_;
	bool lastUsed_;
	bool blocked_{false};

	HciDevice(int devId, bdaddr_t address) : devId_(devId), address_(std::move(address)), use_(0), maxUse_(5)
	{
		for (int i = 0; i < numKnownHciDevices_; ++i)
		{
			uint8_t *addressArray = reinterpret_cast<uint8_t *>(&address_);
			if (memcmp(&knownHciDevices_[i].addressArray_[3], &addressArray[3], sizeof(uint8_t) * 3) == 0)
			{
				maxUse_ = knownHciDevices_[i].maxUsableConnections_;
				break;
			}
		}
	}

	std::size_t addUser()
	{
		return ++use_;
	}

	std::size_t removeUser()
	{
		assert(use_ > 0);
		return --use_;
	}

	std::size_t getUsers()
	{
		return use_;
	}

	std::size_t getMaxUsers()
	{
		return maxUse_;
	}

	bool isFull()
	{
		if (use_ >= maxUse_)
			return true;

		return false;
	}
	
	void block()
	{
		blocked_ = true;
	}

	void unblock()
	{
		blocked_ = false;
	}

	bool isBlocked() const
	{
		return blocked_;
	}
};

struct HciManager
{
	static std::vector<HciDevice> hciDevices_;

	static int addHciToList(int dd, int dev_id, long arg)
	{
		bdaddr_t src{{0, 0, 0, 0, 0, 0}};
		hci_devba(dev_id, &src);
		hciDevices_.emplace_back(dev_id, src);

		char address[24];
		ba2str(&src, address);
		rl_printf("found hci%i with address %s, with max Connections %i\n", dev_id, address, static_cast<int>(hciDevices_.back().getMaxUsers()));

		return 0;
	}

	static std::size_t getNumberOfHciDevices()
	{
		return hciDevices_.size();
	}

	static bool searchForHciDevices()
	{
		if (hciDevices_.size() > 0)
			return true;

		rl_printf("searching for Bluetooth devices...\n");
		hci_for_each_dev(HCI_UP, addHciToList, 0);
		return hciDevices_.size() > 0;
	}

	static HciDevice *getFreeDevice()
	{
		HciDevice *freeDevice = nullptr;
		size_t minUsers = INT_MAX;
		bool selectedDeviceUsedBefore = false;

		for (auto &hciDevice : hciDevices_)
		{
			if (!hciDevice.isFull())
			{
				if (hciDevice.getUsers() < minUsers || (hciDevice.getUsers() <= minUsers + 1 && selectedDeviceUsedBefore))
				{
					selectedDeviceUsedBefore = false;
					if (hciDevice.lastUsed_ == true)
					{
						selectedDeviceUsedBefore = true;
					}
					minUsers = hciDevice.getUsers();
					freeDevice = &hciDevice;
				}
			}
			hciDevice.lastUsed_ = false;
		}

		if (freeDevice != nullptr)
		{
			freeDevice->lastUsed_ = true;
		}

		return freeDevice;
	}

	static HciDevice *getFreeAndUnblockedDevice()
	{
		HciDevice *freeDevice = nullptr;
		size_t minUsers = INT_MAX;
		bool selectedDeviceUsedBefore = false;

		for (auto &hciDevice : hciDevices_)
		{
			if (!hciDevice.isFull() && !hciDevice.isBlocked())
			{
				if (hciDevice.getUsers() < minUsers || (hciDevice.getUsers() <= minUsers + 1 && selectedDeviceUsedBefore))
				{
					selectedDeviceUsedBefore = false;
					if (hciDevice.lastUsed_ == true)
					{
						selectedDeviceUsedBefore = true;
					}
					minUsers = hciDevice.getUsers();
					freeDevice = &hciDevice;
				}
			}
			hciDevice.lastUsed_ = false;
		}

		if (freeDevice != nullptr)
		{
			freeDevice->lastUsed_ = true;
		}

		return freeDevice;
	}


	static bool deviceWithIdAvailable(int id)
	{
		for (auto &hciDevice : hciDevices_)
		{
			if (hciDevice.devId_ == id && !hciDevice.isFull())
			{
				return true;
			}
		}
		return false;
	}

	static void printHciState()
	{
		for (auto &hciDevice : hciDevices_)
		{
			char addressstr[24];
			ba2str(&hciDevice.address_, addressstr);
			rl_printf("hci%i %12s %6zu/%zu\n", hciDevice.devId_, addressstr, hciDevice.use_, hciDevice.maxUse_);
		}
	}
};

#endif // MULTIVEHICLETOOL_HCIMANAGER_H_
