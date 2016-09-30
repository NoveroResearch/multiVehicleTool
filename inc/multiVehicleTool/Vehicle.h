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

#ifndef MULTIVEHICLETOOL_VEHICLE_H
#define MULTIVEHICLETOOL_VEHICLE_H

#include <bluetooth/bluetooth.h>

#include <future>
#include <glib.h>
#include <multiVehicleTool/HciManager.h>
#include <multiVehicleTool/util/AnkiDriveProtocol.h>
#include <multiVehicleTool/util/Timestamp.h>
#include <multiVehicleTool/VehicleModel.h>
#include <memory>
#include <limits>
#include <list>
#include <set>
#include <vector>
#include <string>
#define MEASURE_SEND_RATE 0

enum ConnectionState
{
	STATE_DISCONNECTED = 0,
	STATE_SHOULD_CONNECT,
	STATE_DISCONNECTING,
	STATE_CONNECTING = 5,
	STATE_CONNECTED
};

class VehicleManager;

extern bool verboseDispatcherEntryAndExit_;

struct Vehicle
{
	enum DirectionType
	{
		DIRECTION_NONE,
		DIRECTION_CW,
		DIRECTION_CCW
	};

	struct PingData
	{
		Timestamp timestamp_;
		uint32_t id_{0};
		bool tagged_{false};
	};

	ConnectionState connectionState_{STATE_DISCONNECTED};
	std::string name_;
	VehicleModel model_;
	uint16_t version_{0};
	std::size_t connectionID_{std::numeric_limits<std::size_t>::max()};
	VehicleManager *manager_{nullptr};

	HciDevice *hciDevice_{nullptr};
	bool blockingHciDevice_{false};
	struct bt_att *att_{nullptr};
	struct bt_gatt_client *gattClient_{nullptr};

	bdaddr_t address_{{0, 0, 0, 0, 0, 0}};
	uint16_t readCharacteristicHandle_{0};
	uint16_t writeCharacteristicHandle_{0};
	uint8_t writeCharacteristicProperties_{0};

	std::list<PingData> pingsSent_;
	Timestamp timestampVoltage_;
	uint16_t voltage_;

	Timestamp timestampMarker_;
	uint8_t block_{255};
	uint8_t segment_{255};
	uint8_t readinglen_{8};
	float offset_{0.0f};
	uint16_t speed_{0};
	bool clockwise_{false};
	bool brakingLights_{false};
	uint8_t laneChangeId_{0};

	bool states_[4]{false, false, false, false};

	bool addToWaitList_{false};
	std::size_t connectionTries_{0};
	std::size_t maxConnectionTries_{0};
	int sock_{-1};
	guint eventSourceIdSocketConnectTimeout_{0};
	guint eventSourceIdSocketConnectIOWatch_{0};
	guint eventSourceIdLatencyChangeCheckTimer_{0};
	unsigned int attDisconnectRegistrationId_{0};
	unsigned int gattClientNotifyRegistrationId_{0};

	struct Maneuver
	{
		Timestamp timestamp_;
		DirectionType direction_{DIRECTION_NONE};
		uint16_t vLon_{0}, aLon_{25000};
		uint16_t vLat_{0}, aLat_{1000};
		float pLat_{0.0f};
	};

	Maneuver maneuverBeforeLocalization_;
	std::vector<Maneuver> maneuversSinceLocalization_;

	// Traffic controller settings (to be replayed on traffic controller connection establishment)
	//{
	std::size_t targetLane_{std::numeric_limits<std::size_t>::max()};
	DirectionType targetDirection_{DIRECTION_NONE};
	//}

#if MEASURE_SEND_RATE
	Timestamp timestampMeasureSendRateBase_;
	std::size_t numberOfMessagesSent_{0};
	std::size_t numberOfMessagesSentEffectively_{0};
	std::size_t numberOfMessagesDropped_{0};
#endif

	~Vehicle();

	std::string getMACAddress() const;
	uint16_t getVersion() const;

	bool connect(std::size_t maxConnectionTries = 1, bool addToWaitList = false);
	bool connectNow(HciDevice *hciDevice);
	void disconnect();
	void disconnectWithDelay();
	static gboolean dispatchDisconnect(gpointer userData);


	bool disruptAlienConnection();
	static int findHandleOfAlienConnection(int socket, int deviceId, long vehicleIndex);
	uint16_t handleOfAlienConnection_;

	void setState(ConnectionState newState);
	void sendConnectionStateMessage();
	void sendDisconnectMessage();

	Maneuver getLastManeuver();
	void reportManeuver(const Maneuver &maneuver);

	bool operator==(const Vehicle &vehicle);

	bool shouldConnect() const;
	bool isConnecting() const;
	bool isConnected() const;
	bool isDisconnected() const;

	bool isDriveFirmware() const;
	bool isOverdriveFirmware() const;

	static gboolean ledOff(gpointer user_data);
	bool ping(bool enqueue = false);
	bool pingWithIdentifier(uint32_t id, bool enqueue = false);
	void onPingResponse(Timestamp timestamp);
	bool read();
	bool disconnectPolitely();
	bool setSDKMode(int state);
	bool requestVersion(bool enqueue = false);
	void onVersionResponse(Timestamp timestamp, uint16_t version);
	bool setSpeed(uint16_t speed, uint16_t acceleration = 25000, bool enqueue = false);
	bool changeLane(uint16_t speed, uint16_t acceleration, float offset);
	bool changeLaneAbs(uint16_t speed, uint16_t acceleration, float offset);
	bool cancelLaneChange(bool enqueue = false);
	bool setOffset(float offset = 0.0f);
	bool correctOffset(float delta);
	void setLane(std::size_t lane = std::numeric_limits<std::size_t>::max());
	void setDirection(DirectionType targetDirection);
	bool uturn();
	bool requestVoltage(bool enqueue = false);
	void onVoltageResponse(Timestamp timestamp, uint16_t voltage);
	bool setLights(uint8_t state, bool enqueue = false);
	bool setLightsPattern(anki_vehicle_light_channel_t channel, anki_vehicle_light_effect_t effect, uint8_t start, uint8_t end, uint16_t cyclesPerMinute);
	bool setConfigParameters(uint8_t superCodeParseMask = SUPERCODE_ALL, anki_track_material_t trackMaterial = TRACK_MATERIAL_VINYL);
	bool configureTrack(uint8_t numberOfLanes);

	void forwardMotionPlanDirective(const std::string &directive);

private:

	bool openSocket(HciDevice *hciDevice);

	static gboolean dispatchOnSocketConnectTimeout(gpointer userData);
	void onSocketConnectTimeout();
	
	static gboolean dispatchOnSocketConnect(GIOChannel *source, GIOCondition condition, gpointer userData);
	void onSocketConnect(GIOCondition condition);

	static void dispatchOnGattConnect(bool success, uint8_t attEcode, void *userData);
	void onGattConnect(bool success, uint8_t attEcode);

	static void dispatchOnAttDisconnect(int err, void *user_data);
	void onAttDisconnect(int err);

	static void dispatchParseService(struct gatt_db_attribute *attr, void *userData);
	void parseService(struct gatt_db_attribute *attr);

	static void dispatchParseCharacteristic(struct gatt_db_attribute *attr, void *userData);
	void parseCharacteristic(struct gatt_db_attribute *attr);

	void continueConnection(bool retry);
	static gboolean unblockHciDeviceAndContinueConnection(gpointer userData);

	static void dispatchOnNotifyEventRegistration(uint16_t attEcode, void *user_data);
	void onNotifyEventRegistration(uint16_t attEcode);

	static void dispatchOnNotifyEvent(uint16_t value_handle, const uint8_t *pdu, uint16_t len, gpointer userData);
	void onNotifyEvent(const guint8 *pdu, guint16 len);

	static void dispatchOnReadEvent(bool success, uint8_t attEcode, const guint8 *pdu, guint16 plen, gpointer userData);
	void onReadEvent(bool success, uint8_t attEcode, const guint8 *pdu, guint16 plen);
	
	void onMessage(const uint8_t *data, uint16_t len);

	void onLocalization(Timestamp timestamp, uint8_t block, uint8_t segment, float offset, uint16_t speed, bool clockwise, uint8_t readinglen);
	void onTransition(Timestamp timestamp, float offset, bool clockwise);
	void onDelocalization(Timestamp timestamp);
	void onWheelMovement(Timestamp timestamp, bool flag1, bool flag2);
	void onStateChange(Timestamp timestamp, bool flag1, bool flag2, bool flag3, bool flag4);

	void removeSocketConnectTimeout();
	void removeSocketConnectIOWatch();
	void removeLatencyChangeCheckTimer();

	bool setLatency(uint16_t value);
	const std::future<int> &setLatencyAsync(uint16_t value);
	static int setLatencyHelper(int fd, int deviceId, uint16_t value);

	static gboolean dispatchOnLatencyChangeCheck(gpointer userData);
	bool onLatencyChangeCheck();

	int8_t getRSSI();
	uint8_t getLinkQuality();

	void cleanupBelowAtt();
	void cleanupFromGattDownToAtt();

	bool validateConnectionStateVariables() const;

	std::future<int> resultSetLatencyAsync_;
	bool ankicontrolReceivedConnect_{false};

	uint8_t messageBuffer_[20];
	int messageBufferLength_ = 0;
	bool sendMessageBuffer();
	bool sendToVehicle(const uint8_t *message, int length, bool enqueue = false);
};

bool operator==(const bdaddr_t &addr1, const bdaddr_t &addr2);
bool operator<(const bdaddr_t &addr1, const bdaddr_t &addr2);

inline bool Vehicle::isDriveFirmware() const
{
	return (version_ <= 0x2666);
}

inline bool Vehicle::isOverdriveFirmware() const
{
	return (version_ > 0x2666);
}

inline uint16_t Vehicle::getVersion() const
{
	return version_;
}

inline std::string Vehicle::getMACAddress() const
{
	char address[24];
	ba2str(&address_, address);

	return std::string(address);
}

inline bool Vehicle::operator==(const Vehicle &vehicle)
{
	return vehicle.address_ == address_;
}

inline bool Vehicle::shouldConnect() const
{
	return (connectionState_ == STATE_SHOULD_CONNECT);
}

inline bool Vehicle::isConnecting() const
{
	return (connectionState_ == STATE_CONNECTING);
}

inline bool Vehicle::isConnected() const
{
	return (connectionState_ == STATE_CONNECTED);
}

inline bool Vehicle::isDisconnected() const
{
	return (connectionState_ < STATE_CONNECTING);
}

inline Vehicle::Maneuver Vehicle::getLastManeuver()
{
	if (maneuversSinceLocalization_.empty())
		return maneuverBeforeLocalization_;

	return maneuversSinceLocalization_.back();
}

#endif
