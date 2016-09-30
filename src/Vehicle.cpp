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

#include <multiVehicleTool/ankidrive.h>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/CommandPrompt.h>
#include <multiVehicleTool/HciManager.h>
#include <multiVehicleTool/Vehicle.h>
#include <multiVehicleTool/VehicleManager.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/uuid.h>
#include <bluetooth/hci_lib.h>

extern "C" {
#include <src/shared/att.h>
#include <src/shared/gatt-db.h>
#include <src/shared/gatt-client.h>
#include <attrib/att.h>
}

#include <poll.h>

#include <iostream>
#include <memory>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cinttypes>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <limits>
#include <boost/numeric/conversion/cast.hpp>

#ifdef LED_SUPPORT
#include <wiringPi.h>
#endif

#define ATT_CID 4

bool verboseDispatcherEntryAndExit_{false};

bool operator==(const bdaddr_t &addr1, const bdaddr_t &addr2)
{
	return (addr1.b[0] == addr2.b[0] && addr1.b[1] == addr2.b[1] && addr1.b[2] == addr2.b[2] && addr1.b[3] == addr2.b[3] && addr1.b[4] == addr2.b[4] && addr1.b[5] == addr2.b[5]);
}

bool operator<(const bdaddr_t &addr1, const bdaddr_t &addr2)
{
	for (std::size_t i = 0; i < 6; ++i)
	{
		if (addr1.b[i] < addr2.b[i])
			return true;
		if (addr1.b[i] > addr2.b[i])
			return false;
	}

	return false;
}

Vehicle::~Vehicle()
{
	disconnect();
}

bool Vehicle::sendMessageBuffer()
{
	bool ret;
	if(bt_gatt_client_is_ready(gattClient_))
		ret = bt_gatt_client_write_without_response(gattClient_, writeCharacteristicHandle_, false, (uint8_t *)&messageBuffer_, messageBufferLength_);
	else
		ret = false;

	messageBufferLength_ = 0;
	memset(&messageBuffer_, 0, sizeof(messageBuffer_));
	return ret;
}

bool Vehicle::sendToVehicle(const uint8_t *message, int length, bool enqueue)
{
#if MEASURE_SEND_RATE
	Timestamp now = Timestamp::clock::now();
	if (timestampMeasureSendRateBase_ == Timestamp())
		timestampMeasureSendRateBase_ = now;
	double dt = toSeconds(now - timestampMeasureSendRateBase_);

	if (dt > 3.0)
	{
		rl_printf("%12s: BT send rate = %.2f, effective send rate = %.2f, drop rate = %.2f\n", name_.c_str(), numberOfMessagesSent_ / dt, numberOfMessagesSentEffectively_ / dt, numberOfMessagesDropped_ / dt);
		timestampMeasureSendRateBase_ = now;
		numberOfMessagesSentEffectively_ = 0;
		numberOfMessagesSent_ = 0;
		numberOfMessagesDropped_ = 0;
	}

	++numberOfMessagesSent_;
#endif

	if(!bt_gatt_client_is_ready(gattClient_))
	{
#if MEASURE_SEND_RATE
		++numberOfMessagesDropped_;
#endif
		return false;
	}

	if (!isDriveFirmware())
	{
		bool ret = true;

		if (enqueue == false && messageBufferLength_ == 0)
		{
#if MEASURE_SEND_RATE
			++numberOfMessagesSentEffectively_;
#endif
			return bt_gatt_client_write_without_response(gattClient_, writeCharacteristicHandle_, false, message, length);
		}

		if (messageBufferLength_ + length > static_cast<int>(sizeof(messageBuffer_)))
		{
#if MEASURE_SEND_RATE
			++numberOfMessagesSentEffectively_;
#endif
			ret = sendMessageBuffer();
		}

		memcpy(&messageBuffer_[messageBufferLength_], message, length);
		messageBufferLength_ = messageBufferLength_ + length;

		// Also flush buffer if not even the smallest possible message of size 2 would fit into it anymore.
		if (!enqueue || messageBufferLength_ + 2 > static_cast<int>(sizeof(messageBuffer_)))
		{
#if MEASURE_SEND_RATE
			++numberOfMessagesSentEffectively_;
#endif
			ret = sendMessageBuffer();
		}

		return ret;
	}
	else
	{
#if MEASURE_SEND_RATE
		++numberOfMessagesSentEffectively_;
#endif
		return bt_gatt_client_write_without_response(gattClient_, writeCharacteristicHandle_, false, message, length);
	}
}

bool Vehicle::connect(std::size_t maxConnectionTries, bool addToWaitList)
{
	if (connectionState_ == STATE_CONNECTED || connectionState_ == STATE_CONNECTING)
		return true;
	if (connectionState_ == STATE_DISCONNECTING)
		return false;
	if (connectionState_ == STATE_SHOULD_CONNECT)
		return true;

	setState(STATE_SHOULD_CONNECT);

	addToWaitList_ = addToWaitList;
	connectionTries_ = 0;
	maxConnectionTries_ = maxConnectionTries;

	if (addToWaitList_)
		manager_->addToWaitList(address_);

	continueConnection(true);

	return true;
}

bool Vehicle::connectNow(HciDevice *hciDevice)
{
	bool verbose{false};

	// WARNING: This function is to be called by the vehicle manager only. The vehicle manager guarantees that now is a good time to attempt to establish a connection.

	assert(connectionState_ == STATE_SHOULD_CONNECT);
	assert(validateConnectionStateVariables());

	if (hciDevice == nullptr)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Vehicle manager supplied invalid HCI device in connection request.\n");
		return false;
	}

	setState(STATE_CONNECTING);
	++connectionTries_;
	
	hciDevice_ = hciDevice;
	hciDevice_->addUser();
	assert(!hciDevice_->isBlocked());
	hciDevice_->block();
	blockingHciDevice_ = true;

	rl_printf("Connecting to %s (using hci%i)...\n", name_.c_str(), hciDevice_->devId_);

	if (!openSocket(hciDevice))
	{
		int err = errno;

		disconnect();

		// Try to disrupt alien connections if we get an immediate device or resource busy error.
		if (err == EBUSY)
		{
			rl_printf(COLOR_YELLOW "Warning: " COLOR_OFF "Trying to disrupt existing connection to %s...\n", name_.c_str());
			if (disruptAlienConnection())
				err = EAGAIN;
		}

		continueConnection(err == EAGAIN ? true : false);
		return false;
	}
	
	GIOChannel *channel = g_io_channel_unix_new(sock_);

	g_io_channel_set_encoding(channel, NULL, NULL);
	g_io_channel_set_buffered(channel, FALSE);

	eventSourceIdSocketConnectIOWatch_ = g_io_add_watch(channel, (GIOCondition)(G_IO_IN | G_IO_OUT | G_IO_PRI | G_IO_HUP | G_IO_ERR | G_IO_NVAL), dispatchOnSocketConnect, static_cast<gpointer>(this));

	g_io_channel_unref(channel);

	if (verbose)
		rl_printf("Adding 5 seconds timeout watching for socket connection to %s.\n", name_.c_str());

	eventSourceIdSocketConnectTimeout_ = g_timeout_add_seconds(5, dispatchOnSocketConnectTimeout, static_cast<gpointer>(this));

	return true;
}

void Vehicle::disconnect()
{
	auto connectionStateBefore = connectionState_;

	if (connectionState_ == STATE_DISCONNECTED)
		return;

	setState(STATE_DISCONNECTING);

	cleanupFromGattDownToAtt();
	cleanupBelowAtt();
	
	setState(STATE_DISCONNECTED);

	if (connectionStateBefore == STATE_CONNECTED)
	{
		rl_printf(COLOR_GREEN "Disconnected from %s." COLOR_OFF "\n", name_.c_str());
		CommandPrompt::getInstance().updatePrompt();
	}
}

void Vehicle::disconnectWithDelay()
{
	g_idle_add(dispatchDisconnect, static_cast<gpointer>(this));
}

gboolean Vehicle::dispatchDisconnect(gpointer userData)
{
	assert(userData);

	auto vehicle = static_cast<Vehicle*>(userData);
	vehicle->disconnect();

	return G_SOURCE_REMOVE;
}

bool Vehicle::disruptAlienConnection()
{
	if (sock_ != -1)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Alien connections of connecting or connected vehicles cannot be disrupted.\n");
		return false;
	}

	// Determine index of this vehicle in vehicle manager.
	long index = boost::numeric_cast<long>(manager_->getVehicleIndexByAddress(address_));

	// Check all connections of all HCI devices for connections to the MAC address of this vehicle.
	int deviceId = hci_for_each_dev(HCI_UP, findHandleOfAlienConnection, index);

	if (deviceId < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to find alien connection of %s.\n", name_.c_str());
		return false;
	}

	// Disrupt alien connection.
#if 0
	rl_printf(COLOR_BLUE "Info: " COLOR_OFF "Handle of alien connection of %s is %" PRIu16 " on device id %d.\n", name_.c_str(), handleOfAlienConnection_, deviceId);
#endif

	// Adapted from bluez-5.40 tools/hcitool.c:cmd_ledc.

	int dd = hci_open_dev(deviceId);
	if (dd < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to open HCI device of %s: %s.\n", name_.c_str(), strerror(errno));
		return false;
	}

	if (hci_disconnect(dd, handleOfAlienConnection_, HCI_OE_USER_ENDED_CONNECTION, 10000) < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to disrupt alien connection of %s: %s.\n", name_.c_str(), strerror(errno));

		hci_close_dev(dd);
		handleOfAlienConnection_ = 0;

		return false;
	}

	handleOfAlienConnection_ = 0;

	rl_printf(COLOR_GREEN "Disrupted alien connection of %s." COLOR_OFF "\n", name_.c_str());

	if (hci_close_dev(dd) < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to close HCI device %s: %s.\n", name_.c_str(), strerror(errno));
		return true;
	}

	return true;
}

int Vehicle::findHandleOfAlienConnection(int socket, int deviceId, long vehicleIndex)
{
	auto vehicleManager = CommandPrompt::getInstance().getVehicleManager();

	auto vehicle = vehicleManager->getVehicleByIndex(vehicleIndex);
	assert(vehicle != nullptr);

	// Adapted from bluez-5.40 tools/hcitool.c:conn_list.

	const int maxConnections = 32;

	std::size_t size = maxConnections * sizeof(hci_conn_info) + sizeof(hci_conn_list_req);
	hci_conn_list_req *cl = static_cast<hci_conn_list_req *>(malloc(size));

	if (!cl)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to allocate memory for finding handle of alien connection of %s: %s.\n", vehicle->name_.c_str(), strerror(errno));
		return 0;
	}

	memset(cl, 0, size);
	cl->dev_id = deviceId;
	cl->conn_num = maxConnections;

	if (ioctl(socket, HCIGETCONNLIST, static_cast<void *>(cl)))
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to get connection list to determine handle of alien connection of %s: %s.\n", vehicle->name_.c_str(), strerror(errno));
		return 0;
	}

	int count = 0;
	vehicle->handleOfAlienConnection_ = 0;

	//rl_printf("Found %d connections via device id %d.\n", cl->conn_num, deviceId);

	hci_conn_info *ci = cl->conn_info;
	for (int i = 0; i < cl->conn_num; i++, ci++)
	{
		// Note: LE links have unofficial type value 0x80.
		if (ci->bdaddr == vehicle->address_ && ci->type == 0x80)
		{
#if 0
			char addr[18];
			char *str;
			ba2str(&ci->bdaddr, addr);
			str = hci_lmtostr(ci->link_mode);
			rl_printf(COLOR_BLUE "Found alien connection on device %d: " COLOR_OFF "%s %s handle %d state %d lm %s\n", deviceId, ci->out ? "<" : ">", addr, ci->handle, ci->state, str);
			bt_free(str);
#endif

			vehicle->handleOfAlienConnection_ = ci->handle;

			++count;
		}
	}

	free(cl);
	return count;
}

bool Vehicle::openSocket(HciDevice *hciDevice)
{
	assert(sock_ == -1);

	struct sockaddr_l2 srcaddr, dstaddr;

	pingsSent_.clear();

	sock_ = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
	if (sock_ < 0)
	{
		int err = errno;
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to create L2CAP socket of %s: %s.\n", name_.c_str(), strerror(err));
		errno = err;
		return false;
	}

	// Set up source address.
	memset(&srcaddr, 0, sizeof(srcaddr));
	srcaddr.l2_family = AF_BLUETOOTH;
	srcaddr.l2_cid = htobs(ATT_CID);
	bacpy(&srcaddr.l2_bdaddr, &hciDevice_->address_);

	if (bind(sock_, (struct sockaddr *)&srcaddr, sizeof(srcaddr)) < 0)
	{
		int err = errno;
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to bind L2CAP socket of %s: %s.\n", name_.c_str(), strerror(err));
		errno = err;
		return false;
	}

	// Set up destination address.
	memset(&dstaddr, 0, sizeof(dstaddr));
	dstaddr.l2_family = AF_BLUETOOTH;
	dstaddr.l2_cid = htobs(ATT_CID);
	dstaddr.l2_bdaddr_type = BDADDR_LE_RANDOM;
	bacpy(&dstaddr.l2_bdaddr, &address_);

	// Set socket to non-blocking mode.
	int flags = fcntl(sock_, F_GETFL, NULL);
	if (flags < 0)
	{
		int err = errno;
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to get flags for setting socket of %s to non-blocking mode: %s.\n", name_.c_str(), strerror(err));
		errno = err;
		return false;
	}

	if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0)
	{
		int err = errno;
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set socket of %s to non-blocking mode: %s.\n", name_.c_str(), strerror(err));
		errno = err;
		return false;
	}

	if (::connect(sock_, (struct sockaddr *)&dstaddr, sizeof(dstaddr)) < 0)
	{
		int err = errno;
		if (err != EINPROGRESS)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Immediately failed to connect socket of %s: %s.\n", name_.c_str(), strerror(err));
			errno = err;
			return false;
		}
	}

	// Set socket back to non-blocking mode.
	if (fcntl(sock_, F_SETFL, flags) < 0)
	{
		int err = errno;
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set socket of %s back to blocking mode: %s.\n", name_.c_str(), strerror(err));
		errno = err;
		return false;
	}

	errno = 0;
	return true;
}

gboolean Vehicle::dispatchOnSocketConnectTimeout(gpointer userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnSocketConnectTimeout...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onSocketConnectTimeout();

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnSocketConnectTimeout...\n");

	return FALSE;
}

void Vehicle::onSocketConnectTimeout()
{
	// Ensure that we do not attempt to destroy the timer while being handled.
	eventSourceIdSocketConnectTimeout_ = 0;

	rl_printf(COLOR_YELLOW "Warning: " COLOR_OFF "Aborting attempt to connect socket of %s since it is taking too long.\n", name_.c_str());

	auto hciDevice = hciDevice_;

	disconnect();

	if (hciDevice)
	{
		// Allow the dongle to recover for 50ms in order to prevent EBUSY errors in the next connection attempt.
		hciDevice->block();

		g_timeout_add(50, Vehicle::unblockHciDeviceAndContinueConnection, static_cast<gpointer>(new std::pair<VehicleManager*, HciDevice*>(manager_, hciDevice)));
	}
	else
		continueConnection(false);
}

gboolean Vehicle::dispatchOnSocketConnect(GIOChannel *source, GIOCondition condition, gpointer userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnSocketConnect...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onSocketConnect(condition);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnSocketConnect...\n");

	return FALSE;
}

void Vehicle::onSocketConnect(GIOCondition condition)
{
	// Explicitly remove socket connect timeout.
	removeSocketConnectTimeout();

	// Socket connect IO watch is automatically removed when exiting Vehicle::dispatchOnSocketConnect.
	eventSourceIdSocketConnectIOWatch_ = 0;

	if ((condition & G_IO_ERR) || (condition & G_IO_HUP) || (condition & G_IO_NVAL))
	{
		/*
		G_IO_IN  	There is data to read.
		G_IO_OUT 	Data can be written (without blocking).
		G_IO_PRI 	There is urgent data to read.
		G_IO_ERR 	Error condition.
		G_IO_HUP 	Hung up (the connection has been broken, usually for pipes and sockets).
		G_IO_NVAL	Invalid request. The file descriptor is not open.
		*/

		int optionValue = 0;
		socklen_t optionLength = sizeof(optionValue);

		if (getsockopt(sock_, SOL_SOCKET, SO_ERROR, &optionValue, &optionLength) < 0)
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to connect socket of %s and failed to determine reason: %s.\n", name_.c_str(), strerror(errno));
		else
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to connect socket of %s: %s.\n", name_.c_str(), strerror(optionValue));

		disconnect();
		continueConnection(true);
		return;
	}

	att_ = bt_att_new(sock_, false);
	if (!att_)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to initialize ATT transport layer of %s.\n", name_.c_str());
		disconnect();
		continueConnection(true);
		return;
	}

	attDisconnectRegistrationId_ = bt_att_register_disconnect(att_, dispatchOnAttDisconnect, this, nullptr);
	if (!attDisconnectRegistrationId_)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set ATT disconnect handler of %s.\n", name_.c_str());
		disconnect();
		continueConnection(true);
		return;
	}

	struct gatt_db *gattDb = gatt_db_new();
	if (!gattDb)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to create GATT database of %s.\n", name_.c_str());
		disconnect();
		continueConnection(true);
		return;
	}

	gattClient_ = bt_gatt_client_new(gattDb, att_, 0);
	if (!gattClient_)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to create GATT client of %s.\n", name_.c_str());
		disconnect();
		continueConnection(true);
		return;
	}

	if (!bt_gatt_client_set_ready_handler(gattClient_, dispatchOnGattConnect, (void *)this, nullptr))
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set ready handler for GATT client of %s.\n", name_.c_str());
		disconnect();
		continueConnection(true);
		return;
	}

	gatt_db_unref(gattDb);

	connectionID_ = manager_->getFreeConnectionID();

#ifdef LED_SUPPORT
	if (connectionID_ < 9)
		digitalWrite(connectionID_, HIGH);
#endif
}

void Vehicle::dispatchOnGattConnect(bool success, uint8_t attEcode, void *userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnGattConnect...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onGattConnect(success, attEcode);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnGattConnect...\n");
}

void Vehicle::onGattConnect(bool success, uint8_t attEcode)
{
	assert(connectionState_ == STATE_CONNECTING);

	if (!success)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to connect to %s: %s.\n", name_.c_str(), att_ecode2str(attEcode));
		disconnectWithDelay();
		continueConnection(true);
		return;
	}

	// Unblock HCI device and poke vehicle manager such that it can issue the next connection.
	assert(blockingHciDevice_);
	hciDevice_->unblock();
	blockingHciDevice_ = false;
	continueConnection(true);

	// Determine read and write characteristic handles.
	gatt_db_foreach_service(bt_gatt_client_get_db(gattClient_), nullptr, dispatchParseService, (void *)this);
	if (readCharacteristicHandle_ == 0 || writeCharacteristicHandle_ == 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to determine read and write characteristics of %s.\n", name_.c_str());
		disconnectWithDelay();
		continueConnection(true);
		return;
	}

	// Register for notifications when the vehicle sends data.
	// We do this by setting the notification bit on the
	// client configuration characteristic:
	// see:
	// https://developer.bluetooth.org/gatt/descriptors/Pages/DescriptorViewer.aspx?u=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
	uint8_t notify_cmd[] = {0x01, 0x00};
	if (bt_gatt_client_write_without_response(gattClient_, writeCharacteristicProperties_, false, notify_cmd, 2) == 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set notification bit in write characteristic of %s.\n", name_.c_str());
		disconnectWithDelay();
		continueConnection(true);
		return;
	}

	gattClientNotifyRegistrationId_ = bt_gatt_client_register_notify(gattClient_, readCharacteristicHandle_, dispatchOnNotifyEventRegistration, dispatchOnNotifyEvent, this, nullptr);
	if (gattClientNotifyRegistrationId_ == 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to register for notify event of %s.\n", name_.c_str());
		disconnectWithDelay();
		continueConnection(true);
		return;
	}

	setState(STATE_CONNECTED);

	if (addToWaitList_)
	{
		manager_->removeFromWaitList(address_);
		addToWaitList_ = false;
	}

	rl_printf(COLOR_GREEN "Connected to %s." COLOR_OFF "\n", name_.c_str());
	CommandPrompt::getInstance().updatePrompt();

	setLatencyAsync(16);
	g_timeout_add(250, Vehicle::dispatchOnLatencyChangeCheck, this);

	requestVersion(true);

	// as disconnected vehicles are never driving switch on braking lights
	brakingLights_ = true;
	setLights(UINT8_C(0x22));

	anki_vehicle_msg_t message;
	message.msg_id = 0x0b;
	message.size = 1;
	sendToVehicle((uint8_t *)&message, message.size + 1);

	setSDKMode(1);
	setConfigParameters();
}

void Vehicle::dispatchOnAttDisconnect(int err, void *userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnAttDisconnect...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onAttDisconnect(err);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnAttDisconnect...\n");
}

void Vehicle::onAttDisconnect(int err)
{
	rl_printf(COLOR_RED "Error: " COLOR_OFF "ATT layer of %s disconnected: %s.\n", name_.c_str(), strerror(err));

	disconnect();
}

void Vehicle::dispatchParseService(struct gatt_db_attribute *gattDbAttribute, void *userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchParseService...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->parseService(gattDbAttribute);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchParseService...\n");
}


void Vehicle::parseService(struct gatt_db_attribute *attr)
{
	// Parse characteristics if service UUID matches the Anki service UUID.
	bt_uuid_t uuid_anki;
	bt_string_to_uuid(&uuid_anki, ANKI_STR_SERVICE_UUID);

	bt_uuid_t uuid_found;
	if (!gatt_db_attribute_get_service_data(attr, nullptr, nullptr, nullptr, &uuid_found))
		return;

	if (bt_uuid_cmp(&uuid_anki, &uuid_found) == 0)
	{
		gatt_db_service_foreach_char(attr, dispatchParseCharacteristic, this);
	}
}

void Vehicle::dispatchParseCharacteristic(struct gatt_db_attribute *attr, void *userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchParseCharacteristic...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->parseCharacteristic(attr);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchParseCharacteristic...\n");
}

void Vehicle::parseCharacteristic(struct gatt_db_attribute *attr)
{
	// Store read or write characteristic handle if they match the Anki characteristics UUIDs.
	uint16_t valueHandle;
	uint8_t properties;

	bt_uuid_t uuid_anki_read_char, uuid_anki_write_char, uuid_found;
	bt_string_to_uuid(&uuid_anki_read_char, ANKI_STR_CHR_READ_UUID);
	bt_string_to_uuid(&uuid_anki_write_char, ANKI_STR_CHR_WRITE_UUID);

	if (!gatt_db_attribute_get_char_data(attr, nullptr, &valueHandle, &properties, nullptr, &uuid_found))
		return;

	if (bt_uuid_cmp(&uuid_anki_read_char, &uuid_found) == 0)
	{
		readCharacteristicHandle_ = valueHandle;
	}
	else if (bt_uuid_cmp(&uuid_anki_write_char, &uuid_found) == 0)
	{
		writeCharacteristicHandle_ = valueHandle;
		writeCharacteristicProperties_ = properties;
	}
}

void Vehicle::dispatchOnNotifyEventRegistration(uint16_t attEcode, void *userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnNotifyEventRegistration...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onNotifyEventRegistration(attEcode);
	
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnNotifyEventRegistration...\n");
}

void Vehicle::onNotifyEventRegistration(uint16_t attEcode)
{
	bool verbose{false};

	if (attEcode)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to register notify handler of %s: %s.\n", name_.c_str(), att_ecode2str(attEcode));
		disconnect();
		return;
	}
	else
	{
		if (verbose)
			rl_printf("Succeeded to register message handler of %s.\n", name_.c_str());
	}
}

void Vehicle::dispatchOnNotifyEvent(uint16_t handle, const uint8_t *pdu, uint16_t len, gpointer userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnNotifyEvent...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onNotifyEvent(pdu, len);
	
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnNotifyEvent...\n");
}

void Vehicle::onNotifyEvent(const guint8 *pdu, guint16 len)
{
	onMessage(pdu, len);
}

void Vehicle::dispatchOnReadEvent(bool success, uint8_t attEcode, const guint8 *pdu, guint16 len, gpointer userData)
{
	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnReadEvent...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	vehicle->onReadEvent(success, attEcode, pdu, len);

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnReadEvent...\n");
}

void Vehicle::onReadEvent(bool success, uint8_t attEcode, const guint8 *pdu, guint16 len)
{
	if (!success)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to read from vehicle '%s': %s.\n", name_.c_str(), att_ecode2str(attEcode));
		disconnect();
		return;
	}

	onMessage(pdu, len);
}

bool Vehicle::setLatency(uint16_t value)
{
	int err = setLatencyHelper(bt_att_get_fd(att_), hciDevice_->devId_, value);
	if (err < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set connection latency interval of %s: %s.\n", name_.c_str(), strerror(err));
		return false;
	}

	return true;
}

const std::future<int> &Vehicle::setLatencyAsync(uint16_t value)
{
	resultSetLatencyAsync_ = std::async(std::launch::async, &Vehicle::setLatencyHelper, bt_att_get_fd(att_), hciDevice_->devId_, value);
	return resultSetLatencyAsync_;
}

int Vehicle::setLatencyHelper(int fd, int deviceId, uint16_t value)
{
	// latency = value * 1.25ms (min(value)=6)
	l2cap_conninfo info;
	socklen_t len = sizeof(info);
	if (getsockopt(fd, SOL_L2CAP, L2CAP_CONNINFO, &info, &len) < 0)
		return errno;

	int dd = hci_open_dev(deviceId);

	uint16_t handle = info.hci_handle;
	if (hci_le_conn_update(dd, htobs(handle), htobs(value), htobs(value), htobs(0), htobs(0x02BC), 5000) < 0)
	{
		int err = errno;
		hci_close_dev(dd);
		return err;
	}

	if (hci_close_dev(dd) < 0)
		return errno;

	return 0;
}

gboolean Vehicle::dispatchOnLatencyChangeCheck(gpointer userData)
{
	gboolean ret = G_SOURCE_CONTINUE;

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Entering Vehicle::dispatchOnLatencyChange...\n");

	assert(userData);
	Vehicle *vehicle = static_cast<Vehicle *>(userData);
	if (vehicle->onLatencyChangeCheck())
		ret = G_SOURCE_REMOVE;

	if (verboseDispatcherEntryAndExit_)
		rl_printf("Exiting Vehicle::dispatchOnLatencyChange...\n");

	return ret;
}

bool Vehicle::onLatencyChangeCheck()
{
	if (resultSetLatencyAsync_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return false;

	int err = resultSetLatencyAsync_.get();

	if (err != 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to set latency of %s: %s.\n", name_.c_str(), strerror(err));
	}
	else
	{
		//rl_printf(COLOR_GREEN "Changed latency of %s." COLOR_OFF "\n", name_.c_str());
	}

	eventSourceIdLatencyChangeCheckTimer_ = 0;

	return true;
}

int8_t Vehicle::getRSSI()
{
	l2cap_conninfo info;
	socklen_t len = sizeof(info);
	if (getsockopt(bt_att_get_fd(att_), SOL_L2CAP, L2CAP_CONNINFO, &info, &len) == -1)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to get socket options of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<int8_t>::max();
	}

	int dd = hci_open_dev(hciDevice_->devId_);
	if (dd < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to open HCI device of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<int8_t>::max();
	}

	int8_t rssi;
	if (hci_read_rssi(dd, htobs(info.hci_handle), &rssi, 1000) < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to read RSSI of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<int8_t>::max();
	}
	
	hci_close_dev(dd);

	return rssi;
}

uint8_t Vehicle::getLinkQuality()
{
	// WARNING: The following code always fails to read the link quality with an input/output error.

	l2cap_conninfo info;
	socklen_t len = sizeof(info);
	if (getsockopt(bt_att_get_fd(att_), SOL_L2CAP, L2CAP_CONNINFO, &info, &len) == -1)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to get socket options of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<uint8_t>::max();
	}

	int dd = hci_open_dev(hciDevice_->devId_);
	if (dd < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to open HCI device of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<uint8_t>::max();
	}

	uint8_t lq;
	if (hci_read_link_quality(dd, htobs(info.hci_handle), &lq, 1000) < 0)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to read link quality of %s: %s.\n", name_.c_str(), strerror(errno));
		return std::numeric_limits<uint8_t>::max();
	}
	
	hci_close_dev(dd);

	return lq;
}

void Vehicle::cleanupFromGattDownToAtt()
{
	bool verbose{false};

	assert(connectionState_ == STATE_DISCONNECTING);

	if (verbose)
		rl_printf("Cleaning up GATT context of %s...\n", name_.c_str());

	if (gattClient_ != nullptr)
	{
		if (gattClientNotifyRegistrationId_)
		{
			if (!bt_gatt_client_unregister_notify(gattClient_, gattClientNotifyRegistrationId_))
			{
				rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to unregister GATT client notify handler.\n");
			}
		}

		bt_gatt_client_unref(gattClient_);
	}
	
	gattClientNotifyRegistrationId_ = 0;
	gattClient_ = nullptr;

	readCharacteristicHandle_ = 0;
	writeCharacteristicHandle_ = 0;
	writeCharacteristicProperties_ = 0;

	if (verbose)
		rl_printf("Cleaning up ATT context of %s...\n", name_.c_str());

	if (att_ != nullptr)
	{
		if (attDisconnectRegistrationId_)
		{
			if (!bt_att_unregister_disconnect(att_, attDisconnectRegistrationId_))
			{
				rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to unregister ATT disconnect handler.\n");
			}
		}
	
		bt_att_unref(att_);
	}

	attDisconnectRegistrationId_ = 0;
	att_ = nullptr;
}


void Vehicle::cleanupBelowAtt()
{
	bool verbose{false};

	assert(connectionState_ == STATE_DISCONNECTING);

#ifdef LED_SUPPORT
	digitalWrite(connectionID_, LOW);
#endif

	connectionID_ = std::numeric_limits<std::size_t>::max();
	
	if (verbose)
		rl_printf("Cleaning up socket of %s...\n", name_.c_str());
	
	// TODO Who closes this socket before me?!??!?!
	if (sock_ != -1)
	{
		if (close(sock_) < 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Failed to close socket %d: %s.\n", sock_, strerror(errno));
		}
	}
	sock_ = -1;

	if (verbose)
		rl_printf("Cleaning up HCI device of %s...\n", name_.c_str());

	if (hciDevice_ != nullptr)
	{
		if (blockingHciDevice_)
			hciDevice_->unblock();
		hciDevice_->removeUser();
	}

	blockingHciDevice_ = false;
	hciDevice_ = nullptr;

	if (verbose)
		rl_printf("Cleaning up socket connect timeout of %s...\n", name_.c_str());

	removeSocketConnectTimeout();
	
	if (verbose)
		rl_printf("Cleaning up socket connect IO watch of %s...\n", name_.c_str());

	removeSocketConnectIOWatch();

	if (verbose)
		rl_printf("Cleaning up latency change check timer of %s...\n", name_.c_str());

	removeLatencyChangeCheckTimer();

	// Remove from wait list.
	if (addToWaitList_)
		manager_->removeFromWaitList(address_);

	// Send message to ankicontrol.
	if (verbose)
		rl_printf("Notify ankicontrol of disconnection of %s...\n", name_.c_str());

	sendDisconnectMessage();

	// Reset state variables
	timestampMarker_ = Timestamp();
	block_ = 255;
	segment_ = 255;
	offset_ = 0.0f;
	speed_ = 0;
	clockwise_ = false;

	maneuverBeforeLocalization_ = Maneuver();
	maneuversSinceLocalization_.clear();

	targetLane_ = std::numeric_limits<std::size_t>::max();
	targetDirection_ = DIRECTION_NONE;

	if (verbose)
		rl_printf("Finished disconnection of %s.\n", name_.c_str());
}

bool Vehicle::ping(bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_ping(&message);

	PingData pingData;
	pingData.timestamp_ = Timestamp::clock::now();
	pingData.tagged_ = false;

	pingsSent_.push_back(pingData);

	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	return true;
}

bool Vehicle::pingWithIdentifier(uint32_t id, bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_ping(&message);

	PingData pingData;
	pingData.timestamp_ = Timestamp::clock::now();
	pingData.id_ = id;
	pingData.tagged_ = true;

	pingsSent_.push_back(pingData);

	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	return true;
}

void Vehicle::onPingResponse(Timestamp timestamp)
{
	if (pingsSent_.empty())
	{
		rl_printf(COLOR_YELLOW "Warning" COLOR_OFF ": Received ping response without request.\n");
		return;
	}

	auto pingData = pingsSent_.front();
	pingsSent_.pop_front();

	double roundTripTime = toMilliSeconds(Timestamp::clock::now() - pingData.timestamp_);
	int verbose = CommandPrompt::getInstance().getVerbose();

	if (!pingData.tagged_)
		rl_printf("%12s: Ping response: %.0lf ms RTT\n", getMACAddress().c_str(), roundTripTime);
	else if (verbose >= 1)
		rl_printf("%12s: Ping response: %.0lf ms RTT to request with identifier %" PRIu32 "\n", getMACAddress().c_str(), roundTripTime, pingData.id_);
}

bool Vehicle::read()
{
	if (connectionState_ != STATE_CONNECTED)
		return false;
	bt_gatt_client_read_value(gattClient_, readCharacteristicHandle_, dispatchOnReadEvent, this, nullptr);

	return true;
}

bool Vehicle::disconnectPolitely()
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_disconnect(&message);

	if (!sendToVehicle((uint8_t *)&message, length))
		return false;

	return true;
}

bool Vehicle::setSDKMode(int state)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_set_sdk_mode(&message, state, ANKI_VEHICLE_SDK_OPTION_OVERRIDE_LOCALIZATION);

	if (!sendToVehicle((uint8_t *)&message, length, true))
		return false;

	return true;
}

bool Vehicle::requestVersion(bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_get_version(&message);

	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	return true;
}

void Vehicle::onVersionResponse(Timestamp timestamp, uint16_t version)
{
	version_ = bt_get_le16(&version);
	rl_printf("%12s: Version: 0x%04x\n", name_.c_str(), version_);
}

bool Vehicle::setSpeed(uint16_t speed, uint16_t acceleration, bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_set_speed(&message, speed, acceleration);

	Timestamp timestamp = Timestamp::clock::now() + toTimestampDuration(0.040);
	if (speed == 0 && brakingLights_ == false)
	{
		sendToVehicle((uint8_t *)&message, length, true);
		setLights(UINT8_C(0x22), enqueue);
		brakingLights_ = true;
	}
	else if (speed != 0 && brakingLights_ == true)
	{
		sendToVehicle((uint8_t *)&message, length, true);
		setLights(UINT8_C(0x02), enqueue);
		brakingLights_ = false;
	}
	else
	{
		if (!sendToVehicle((uint8_t *)&message, length, enqueue))
			return false;
	}

	auto maneuver = getLastManeuver();
	maneuver.timestamp_ = timestamp;
	maneuver.vLon_ = speed;
	maneuver.aLon_ = acceleration;
	reportManeuver(maneuver);

	return true;
}

/**
 * @param speed The maximum lateral speed in mm/s.
 * @param acceleration The lateral acceleration in mm/s².
 * @param offset The offset of the lateral target position in mm from the lateral position of the last localization.
 *
 * The magnitude of the offset is clamped by default to roughly 63mm
 * (7 lanes) by the firmware running on the Anki vehicles. This can be
 * changed by sending a track configuration to the vehicle using
 * configureTrack(). A positive offset changes the lane towards larger
 * lane numbers, whereas a negative offset changes the lane towards
 * smaller lane numbers.
 */
bool Vehicle::changeLane(uint16_t speed, uint16_t acceleration, float offset)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	{
		anki_vehicle_msg_t message;
		std::size_t length = anki_vehicle_msg_set_offset_from_road_center(&message, 0.0f);
		if (!sendToVehicle((uint8_t *)&message, length, true))
			return false;

		laneChangeId_ = 0;
	}

	{
		anki_vehicle_msg_change_lane_t message;
		float convertedOffset = clockwise_ && isOverdriveFirmware() ? -offset : offset;
		std::size_t length = anki_vehicle_msg_change_lane((anki_vehicle_msg_t*)&message, speed, acceleration, convertedOffset);
		message.tag = laneChangeId_++;
		if (!sendToVehicle((uint8_t *)&message, length))
			return false;
	}

	return true;
}

/**
 * @param speed The maximum lateral speed in mm/s.
 * @param acceleration The lateral acceleration in mm/s².
 * @param offset The offset of the lateral target position in mm from the lateral position of the anchor defined using setOffset().
 *
 * The magnitude of the offset is clamped by default to roughly 63mm
 * (7 lanes) by the firmware running on the Anki vehicles. This can be
 * changed by sending a track configuration to the vehicle using
 * configureTrack(). A positive offset changes the lane towards larger
 * lane numbers, whereas a negative offset changes the lane towards
 * smaller lane numbers.
 */
bool Vehicle::changeLaneAbs(uint16_t speed, uint16_t acceleration, float offset)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_change_lane_t message;
	float convertedOffset = clockwise_ && isOverdriveFirmware() ? -offset : offset;
	std::size_t length = anki_vehicle_msg_change_lane((anki_vehicle_msg_t*)&message, speed, acceleration, convertedOffset);
	message.tag = laneChangeId_++;

	Timestamp timestamp = Timestamp::clock::now() + toTimestampDuration(0.040);
	if (!sendToVehicle((uint8_t *)&message, length))
		return false;

	auto maneuver = getLastManeuver();
	maneuver.timestamp_ = timestamp;
	maneuver.vLat_ = speed;
	maneuver.aLat_ = acceleration;
	maneuver.pLat_ = offset;
	reportManeuver(maneuver);

	return true;
}

bool Vehicle::cancelLaneChange(bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_cancel_lane_change(&message);

	Timestamp timestamp = Timestamp::clock::now() + toTimestampDuration(0.040);
	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	auto maneuver = getLastManeuver();
	maneuver.timestamp_ = timestamp;
	maneuver.vLat_ = 0;
	reportManeuver(maneuver);

	return true;
}

/**
 * @param offset The offset of the lateral position of the last localization in mm from the lateral position of the anchor.
 *
 * Defines an anchor by providing the lateral offset of the last localization
 * from the lateral position of the anchor. A positive offset means the
 * last localization was towards larger lane numbers from the lateral
 * position of the anchor.
 */
bool Vehicle::setOffset(float offset)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	float convertedOffset = clockwise_ && isOverdriveFirmware() ? -offset : offset;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_set_offset_from_road_center(&message, convertedOffset);
	if (!sendToVehicle((uint8_t *)&message, length))
		return false;

	laneChangeId_ = 0;

	return true;
}

/**
 * @param delta The correction in mm that is to be added to the lateral anchor position.
 *
 * Each time a position update is received and the position does not match with the
 * offset suggested by the car the lateral anchor position should be corrected for this
 * delta. If the offset reported by the vehicle in reality is offset' then
 * delta = offset' - offset.
 */
bool Vehicle::correctOffset(float delta)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	if (delta == 0.0f)
		return true;

	float convertedDelta = clockwise_ && isOverdriveFirmware() ? -delta : delta;

	anki_vehicle_msg_t message;
	message.msg_id = 0x34;
	message.size = 5;
	std::memcpy(&message.payload[0], &convertedDelta, sizeof(convertedDelta));

	if (!sendToVehicle((uint8_t *)&message, message.size + 1))
		return false;

	return true;
}

void Vehicle::setLane(std::size_t lane)
{
	if (connectionState_ != STATE_CONNECTED)
		return;

	targetLane_ = lane;
}

void Vehicle::setDirection(DirectionType targetDirection)
{
	if (connectionState_ != STATE_CONNECTED)
		return;

	targetDirection_ = targetDirection;
}

bool Vehicle::uturn()
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length;
	if (isDriveFirmware())
		length = anki_drive_vehicle_msg_turn_180(&message);
	else
		length = anki_vehicle_msg_turn_180(&message);

	Timestamp timestamp = Timestamp::clock::now() + toTimestampDuration(0.040);
	if (!sendToVehicle((uint8_t *)&message, length))
		return false;

	auto maneuver = getLastManeuver();
	if (maneuver.direction_ != DIRECTION_NONE)
	{
		maneuver.timestamp_ = timestamp;
		maneuver.vLat_ = 0;
		maneuver.direction_ = (maneuver.direction_ == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW);
		reportManeuver(maneuver);
	}

	return true;
}

bool Vehicle::requestVoltage(bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_get_battery_level(&message);

	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	return true;
}

void Vehicle::onVoltageResponse(Timestamp timestamp, uint16_t voltage)
{
	timestampVoltage_ = timestamp;
	voltage_ = voltage;
}

bool Vehicle::setLights(uint8_t state, bool enqueue)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_set_lights(&message, state);

	if (!sendToVehicle((uint8_t *)&message, length, enqueue))
		return false;

	return true;
}

bool Vehicle::setLightsPattern(anki_vehicle_light_channel_t channel, anki_vehicle_light_effect_t effect, uint8_t start, uint8_t end, uint16_t cyclesPerMinute)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_lights_pattern(&message, channel, effect, start, end, cyclesPerMinute);

	if (!sendToVehicle((uint8_t *)&message, length))
		return false;

	return true;
}

bool Vehicle::setConfigParameters(uint8_t superCodeParseMask, anki_track_material_t trackMaterial)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;
	
	if (isDriveFirmware())
		return false;

	anki_vehicle_msg_t message;
	std::size_t length = anki_vehicle_msg_set_config_params(&message, superCodeParseMask, trackMaterial);

	if (!sendToVehicle((uint8_t*)&message, length))
		return false;

	return true;
}

bool Vehicle::configureTrack(uint8_t numberOfLanes)
{
	if (connectionState_ != STATE_CONNECTED)
		return false;

	if (isDriveFirmware())
		return false;

	// We have no idea what the payload means except for the second byte. The inserted payload values were observed in a communication dump between the Overdrive App and an Anki vehicle.
	anki_vehicle_msg_t message;
	message.msg_id = 0x49;
	message.size = 8;
	message.payload[0] = 0x00;
	message.payload[1] = numberOfLanes;
	message.payload[2] = 0x00;
	message.payload[3] = 0x01;
	message.payload[4] = 0x02;
	message.payload[5] = 0x00;
	message.payload[6] = 0x0d;

	if (!sendToVehicle((uint8_t*)&message, message.size + 1))
		return false;

	return true;
}


void Vehicle::onMessage(const uint8_t *data, uint16_t len)
{
	if (len > sizeof(anki_vehicle_msg_t))
	{
		rl_printf("Invalid vehicle response from vehicle '%s'\n", getMACAddress().c_str());
		return;
	}

	Timestamp timestamp = Timestamp::clock::now() - toTimestampDuration(0.040);

	int verbose = CommandPrompt::getInstance().getVerbose();

	const anki_vehicle_msg_t *msg = (const anki_vehicle_msg_t *)data;
	switch (msg->msg_id)
	{
	case ANKI_VEHICLE_MSG_V2C_PING_RESPONSE:
	{
		onPingResponse(timestamp);
		break;
	}
	case ANKI_VEHICLE_MSG_V2C_VERSION_RESPONSE:
	{
		const anki_vehicle_msg_version_response_t *m = (const anki_vehicle_msg_version_response_t *)msg;
		onVersionResponse(timestamp, m->version);
		break;
	}
	case ANKI_VEHICLE_MSG_V2C_LOCALIZATION_POSITION_UPDATE:
	{
		if (isDriveFirmware())
		{
			const anki_drive_vehicle_msg_localization_position_update_t *m = (const anki_drive_vehicle_msg_localization_position_update_t *)msg;

			onLocalization(timestamp, m->_reserved[0], m->_reserved[1], m->offset_from_road_center_mm, m->speed_mm_per_sec, (m->is_clockwise == 0X01), 8);

			if (verbose > 0)
			{
				rl_printf("%12s: Position: road_piece_id: 0x%02x, location_id: 0x%02x, offset: %f, speed: %i, clockwise: 0x%02x \n", name_.c_str(), m->_reserved[1], m->_reserved[0], m->offset_from_road_center_mm, m->speed_mm_per_sec, m->is_clockwise);
			}
		}
		else
		{
			const anki_vehicle_msg_localization_position_update_t *m = (const anki_vehicle_msg_localization_position_update_t *)msg;

			bool clockwise = ((m->parsing_flags & PARSEFLAGS_MASK_REVERSE_PARSING) != 0x00);
			float offset = clockwise ? -m->offset_from_road_center_mm : m->offset_from_road_center_mm;

			onLocalization(timestamp, m->location_id, m->road_piece_id, offset, m->speed_mm_per_sec, clockwise, (m->parsing_flags & PARSEFLAGS_MASK_NUM_BITS));

			if (verbose > 0)
			{
				rl_printf("%12s: Position: road_piece_id: 0x%02x, location_id: 0x%02x, offset: %f, speed: %i, flags: 0x%02x \n", name_.c_str(), m->road_piece_id, m->location_id, m->offset_from_road_center_mm, m->speed_mm_per_sec, m->parsing_flags);
			}
		}

		break;
	}
	case ANKI_VEHICLE_MSG_V2C_LOCALIZATION_TRANSITION_UPDATE:
	{
		if (isDriveFirmware())
		{
			const anki_drive_vehicle_msg_localization_transition_update_t *m = (const anki_drive_vehicle_msg_localization_transition_update_t *)msg;

			onTransition(timestamp, m->offset_from_road_center_mm, m->is_clockwise != 0x00);

			if (verbose > 0)
			{
				rl_printf("%12s: Transition: reserved: 0x%02x, offset: %f, is_clockwise: 0x%02x \n", name_.c_str(), m->_reserved, m->offset_from_road_center_mm, m->is_clockwise);
			}
		}
		else
		{
			const anki_vehicle_msg_localization_transition_update_t *m = (const anki_vehicle_msg_localization_transition_update_t *)msg;
			
			float offset = clockwise_ ? -m->offset_from_road_center_mm : m->offset_from_road_center_mm;

			// WARNING: The driving direction can be forward for clockwise and counter-clockwise direction!
			onTransition(timestamp, offset, m->driving_direction == FORWARD);

			if (verbose > 0)
			{
				rl_printf("%12s: Transition: road_piece_idx: 0x%02x, road_piece_idx_prev: 0x%02x, offset: %f, driving_direction: 0x%02x \n", name_.c_str(), m->road_piece_idx, m->road_piece_idx_prev, m->offset_from_road_center_mm, m->driving_direction);
			}
		}

		break;
	}
	case ANKI_VEHICLE_MSG_V2C_OFFSET_FROM_ROAD_CENTER_UPDATE:
	{
		const anki_vehicle_msg_offset_from_road_center_update_t *m = (const anki_vehicle_msg_offset_from_road_center_update_t *)msg;

		//onOffsetUpdate(timestamp, m->offset_from_road_center_mm, m->lane_change_id);

		if (verbose > 0)
		{
			rl_printf("%12s: Offset Update: offset: %f, lane_change_id: 0x%02x \n", name_.c_str(), m->offset_from_road_center_mm, m->lane_change_id);
		}

		break;
	}
	case ANKI_VEHICLE_MSG_V2C_BATTERY_LEVEL_RESPONSE:
	{
		const anki_vehicle_msg_battery_level_response_t *m = (const anki_vehicle_msg_battery_level_response_t *)msg;

		onVoltageResponse(timestamp, m->battery_level);

		if (verbose > 0)
		{
			rl_printf("%12s: Battery: %.3fV (0x%04x) \n", name_.c_str(), (float)m->battery_level / 1000, m->battery_level);
		}

		break;
	}
	case ANKI_VEHICLE_MSG_V2C_VEHICLE_DELOCALIZED:
	{
		onDelocalization(timestamp);

		if (verbose > 0)
		{
			rl_printf("%12s: Delocalized!\n", name_.c_str());
		}

		break;
	}
	case 0x4d:
	{
		onWheelMovement(timestamp, msg->payload[0] != 0x00, msg->payload[1] != 0x00);

		if (verbose > 0)
		{
			rl_printf("%12s: Wheel movement event with payload 0x%02x%02x\n", name_.c_str(), msg->payload[0], msg->payload[1]);
		}

		break;
	}
	case 0x3f:
	{
		onStateChange(timestamp, msg->payload[0] != 0x00, msg->payload[1] != 0x00, msg->payload[2] != 0x00, msg->payload[3] != 0x00);

		if (verbose > 0)
		{
			rl_printf("%12s: State change event with payload 0x%02x%02x%02x%02x\n", name_.c_str(), msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3]);
		}

		break;
	}
	case 0x0c:
	{
		// Response to 0x0b message.
		if (verbose > 0)
		{
			rl_printf("%12s: EHLO with payload 0x%02x%02x%02x%02x%02x\n", name_.c_str(), msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3], msg->payload[4]);
		}
	}
	default:
	{
		if (verbose < 2)
			break;

		char buf[128];
		int i;
		snprintf(&buf[0], 7, "0x%02x: ", msg->msg_id);
		for (i = 0; i < msg->size - 1; i++)
		{
			snprintf(&(buf[i * 5 + 6]), 6, "0x%02x ", msg->payload[i]);
		}
		rl_printf("%12s: %s\n", name_.c_str(), buf);
		break;
	}
	}
}

void Vehicle::onLocalization(Timestamp timestamp, uint8_t block, uint8_t segment, float offset, uint16_t speed, bool clockwise, uint8_t readinglen)
{

}

void Vehicle::onTransition(Timestamp timestamp, float offset, bool clockwise)
{

}

void Vehicle::onDelocalization(Timestamp timestamp)
{

}

void Vehicle::onWheelMovement(Timestamp timestamp, bool flag1, bool flag2)
{

}

void Vehicle::onStateChange(Timestamp timestamp, bool flag1, bool flag2, bool flag3, bool flag4)
{

}

void Vehicle::setState(ConnectionState newState)
{
	if (connectionState_ == newState)
		return;

	connectionState_ = newState;

	sendConnectionStateMessage();
}

void Vehicle::sendConnectionStateMessage()
{

}

void Vehicle::sendDisconnectMessage()
{

}

void Vehicle::reportManeuver(const Maneuver &maneuver)
{

}

void Vehicle::removeSocketConnectTimeout()
{
	if (eventSourceIdSocketConnectTimeout_ == 0)
		return;

	g_source_remove(eventSourceIdSocketConnectTimeout_);
	eventSourceIdSocketConnectTimeout_ = 0;
}

void Vehicle::removeSocketConnectIOWatch()
{
	if (eventSourceIdSocketConnectIOWatch_ == 0)
		return;

	g_source_remove(eventSourceIdSocketConnectIOWatch_);
	eventSourceIdSocketConnectIOWatch_ = 0;
}


void Vehicle::removeLatencyChangeCheckTimer()
{
	if (eventSourceIdLatencyChangeCheckTimer_ == 0)
		return;

	g_source_remove(eventSourceIdLatencyChangeCheckTimer_);
	eventSourceIdLatencyChangeCheckTimer_ = 0;
}

void Vehicle::continueConnection(bool retry)
{
	if (connectionState_ == STATE_DISCONNECTED)
	{
		if (retry)
		{
			if (connectionTries_ < maxConnectionTries_ || maxConnectionTries_ == 0)
			{
				if (maxConnectionTries_ == 0)
					rl_printf(COLOR_YELLOW "Warning: " COLOR_OFF "Scheduling retry #%zu (out of infinite retries) for connection to %s.\n", connectionTries_, name_.c_str());
				else
					rl_printf(COLOR_YELLOW "Warning: " COLOR_OFF "Scheduling retry #%zu (out of %zu retries) for connection to %s.\n", connectionTries_, maxConnectionTries_, name_.c_str());

				if (addToWaitList_)
					manager_->addToWaitList(address_);

				setState(STATE_SHOULD_CONNECT);
			}
			else
			{
				rl_printf(COLOR_RED "Error: " COLOR_OFF "Giving up to connect to %s after the maximum number of %zu retries was reached.\n", name_.c_str(), maxConnectionTries_);
			}
		}
		else
		{
			connectionTries_ = maxConnectionTries_;
			
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Giving up to connect to %s due to fatal connection error.\n", name_.c_str());
		}
	}

	g_idle_add(VehicleManager::dispatchOnConnectionContinuation, static_cast<gpointer>(manager_));
}

gboolean Vehicle::unblockHciDeviceAndContinueConnection(gpointer userData)
{
	assert(userData);
	auto p = static_cast<std::pair<VehicleManager*, HciDevice*>*>(userData);

	auto manager = p->first;
	auto hciDevice = p->second;

	delete p;

	assert(hciDevice);
	hciDevice->unblock();

	assert(manager);
	g_idle_add(VehicleManager::dispatchOnConnectionContinuation, static_cast<gpointer>(manager));

	return G_SOURCE_REMOVE;
}

bool Vehicle::validateConnectionStateVariables() const
{
	bool valid = true;

	if (connectionState_ == STATE_DISCONNECTED || connectionState_ == STATE_SHOULD_CONNECT)
	{
		if (connectionID_ != std::numeric_limits<std::size_t>::max())
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Connection ID is not reset in disconnected state: %zu\n", connectionID_);
			valid = false;
		}

		if (att_ != nullptr)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "ATT context is not reset in disconnected state: %p\n", att_);
			valid = false;
		}

		if (gattClient_ != nullptr)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "GATT context is not reset in disconnected state: %p\n", gattClient_);
			valid = false;
		}

		if (sock_ != -1)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Socket is not reset in disconnected state: %d\n", sock_);
			valid = false;
		}

		if (hciDevice_ != nullptr)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "HCI device is not reset in disconnected state: %p\n", hciDevice_);
			valid = false;
		}

		if (blockingHciDevice_)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Blocking HCI device in disconnected state.\n");
			valid = false;
		}

		if (eventSourceIdSocketConnectTimeout_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Socket connection timeout not reset in disconnected state: %u\n", eventSourceIdSocketConnectTimeout_);
			valid = false;
		}

		if (eventSourceIdSocketConnectIOWatch_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Socket connection IO watch not reset in disconnected state: %u\n", eventSourceIdSocketConnectIOWatch_);
			valid = false;
		}

		if (eventSourceIdLatencyChangeCheckTimer_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Latency change check timer not reset in disconnected state: %u\n", eventSourceIdLatencyChangeCheckTimer_);
			valid = false;
		}
		
		if (attDisconnectRegistrationId_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "ATT disconnect registration not reset in disconnected state: %u\n", attDisconnectRegistrationId_);
			valid = false;
		}

		if (gattClientNotifyRegistrationId_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "GATT client notify registration not reset in disconnected state: %u\n", gattClientNotifyRegistrationId_);
			valid = false;
		}

		if (readCharacteristicHandle_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Read characteristic handle not reset in disconnected state: %" PRIu16 "\n", readCharacteristicHandle_);
			valid = false;
		}

		if (writeCharacteristicHandle_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Write characteristic handle not reset in disconnected state: %" PRIu16 "\n", writeCharacteristicHandle_);
			valid = false;
		}

		if (writeCharacteristicProperties_ != 0)
		{
			rl_printf(COLOR_RED "Error: " COLOR_OFF "Write characteristic properties not reset in disconnected state: %" PRIu8 "\n", writeCharacteristicProperties_);
			valid = false;
		}
	}

	if (!valid)
	{
		rl_printf(COLOR_RED "Error: " COLOR_OFF "Validation of connection state variables of %s failed.\n", name_.c_str());
	}

	return valid;
}
