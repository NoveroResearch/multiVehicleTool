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

#include <multiVehicleTool/config.h>
#include <multiVehicleTool/CommandPrompt.h>
#include <multiVehicleTool/HciManager.h>
#include <multiVehicleTool/VehicleScanner.h>

#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

VehicleScanResult *VehicleScanner::getOrAddDevice(const bdaddr_t &address)
{
	VehicleScanResult *device = getDeviceWithAddress(address);

	if (device == nullptr)
	{
		VehicleScanResult newDevice;
		newDevice.scan_complete = 0;
		newDevice.address = address;
		memset(&newDevice.adv, 0, sizeof(anki_vehicle_adv_t));
		scanResults_.push_back(newDevice);
		device = &scanResults_.back();
	}

	return device;
}

VehicleScanResult *VehicleScanner::getDeviceWithAddress(const bdaddr_t &address)
{
	for (VehicleScanResult &device : scanResults_)
	{
		if (device.address == address)
			return &device;
	}

	return nullptr;
}

void VehicleScanner::scan(int devId)
{
	int err, dd;
	uint8_t own_type = 0x00;
	uint8_t scan_type = 0x01;
	uint8_t filter_policy = 0x00;
	uint16_t interval = htobs(0x0010);
	uint16_t window = htobs(0x0010);
	uint8_t filter_dup = 1;

	if (!HciManager::searchForHciDevices())
	{
		rl_printf("Error: No bluetooth devices available.\n");
		return;
	}

	if (devId == -1)
	{
		HciDevice *p = HciManager::getFreeDevice();
		if (p == nullptr)
		{
			rl_printf("Error: No bluetooth device available.\n");
			return;
		}

		devId = p->devId_;
	}
	else if (!HciManager::deviceWithIdAvailable(devId))
	{
		rl_printf("Error: Selected bluetooth device not available.\n");
		return;
	}

	dd = hci_open_dev(devId);
	if (dd < 0)
	{
		rl_printf("Error: Could not open bluetooth device (%d).\n", dd);
		return;
	}

	err = hci_le_set_scan_parameters(dd, scan_type, interval, window, own_type, filter_policy, 2000);
	if (err < 0)
	{
		rl_printf("Error: Failed to set scan parameters (%d). Possibly due to missing permissions.\n", err);
		hci_close_dev(dd);
		return;
	}

	err = hci_le_set_scan_enable(dd, 0x01, filter_dup, 2000);
	if (err < 0)
	{
		rl_printf("Error: Failed to enable scan (%d).\n", err);
		hci_close_dev(dd);
		return;
	}

	rl_printf("Scanning for bluetooth low-energy devices using hci%i\n", devId);

	clear();
	parseAdvertisingDevices(dd);

	err = hci_le_set_scan_enable(dd, 0x00, filter_dup, 2000);
	if (err < 0)
	{
		rl_printf("Failed to disable scan (%d).\n", err);
		hci_close_dev(dd);
		return;
	}

	hci_close_dev(dd);
}

void VehicleScanner::updateVehicleManager(VehicleManager &manager)
{
	manager.removeDisconnectedVehicles();

	for (std::size_t i = 0; i < size(); ++i)
	{
		if (scanResults_[i].isAnkiVehicle())
			manager.addVehicle(scanResults_[i]);
	}
}

int VehicleScanner::parseAdvertisingDevices(int dd)
{
	unsigned char buf[HCI_MAX_EVENT_SIZE], *ptr;
	struct hci_filter nf, of;
	socklen_t olen;
	int len;
	struct timeval ts_now, ts_until;
	gettimeofday(&ts_now, NULL);
	ts_until = ts_now;
	ts_until.tv_sec += 3;

	/* get initial filter settings for restauration after scan */
	olen = sizeof(of);
	if (getsockopt(dd, SOL_HCI, HCI_FILTER, &of, &olen) < 0)
	{
		rl_printf("Failed to get socket options.\n");
		return -1;
	}

	/* set filter for scanning */
	hci_filter_clear(&nf);
	hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
	hci_filter_set_event(EVT_LE_META_EVENT, &nf);

	if (setsockopt(dd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0)
	{
		rl_printf("Failed to set socket options.\n");
		return -1;
	}

	int flags = fcntl(dd, F_GETFL, 0);
	fcntl(dd, F_SETFL, flags | O_NONBLOCK);
	while (ts_now.tv_sec < ts_until.tv_sec)
	{
		evt_le_meta_event *meta;
		le_advertising_info *info;
		char addr[18];

		while ((len = read(dd, buf, sizeof(buf))) < 0)
		{
			if (errno == EAGAIN || errno == EINTR)
			{
				gettimeofday(&ts_now, NULL);
				if (ts_now.tv_sec > ts_until.tv_sec)
				{
					goto done;
				}
				continue;
			}
			goto done;
		}

		ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
		len -= (1 + HCI_EVENT_HDR_SIZE);

		meta = (evt_le_meta_event *)ptr;

		if (meta->subevent != 0x02)
			goto done;

		/* Ignoring multiple reports */
		info = (le_advertising_info *)(meta->data + 1);

		ba2str(&info->bdaddr, addr);

		VehicleScanResult *v = getOrAddDevice(info->bdaddr);

		if (v == nullptr)
			continue;

		int err = anki_vehicle_parse_adv_record(info->data, info->length, &v->adv);
		if (err == 0 && v->adv.mfg_data.identifier > 0 && v->adv.local_name.version > 0 && !v->scan_complete)
		{
			v->scan_complete = 1;
			rl_printf("Discovered %s %s [v%04x] (%s %04x)\n", addr, v->adv.local_name.name,
			          v->adv.local_name.version & 0xffff, VehicleModel(v->adv.mfg_data.model_id).getName().c_str(),
			          v->adv.mfg_data.identifier & 0xffff);
		}
		gettimeofday(&ts_now, NULL);
	}

done:
	setsockopt(dd, SOL_HCI, HCI_FILTER, &of, sizeof(of));

	return 0;
}
