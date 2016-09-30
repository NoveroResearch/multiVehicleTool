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

/*
 *  vehicle-tool
 *
 *  Example vehicle control tool for Anki Drive SDK
 *
 *  Copyright (C) 2014 Anki, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  Portions of this software are derived from BlueZ, a Bluetooth protocol stack for
 *  Linux. The license for BlueZ is included below.
 */

/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2011  Nokia Corporation
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <readline/readline.h>
#include <readline/history.h>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <iostream>
#include <chrono>
#include <cinttypes>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <glib.h>
#include <multiVehicleTool/ankidrive.h>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/CommandPrompt.h>
#include <multiVehicleTool/util/AnkiDriveProtocol.h>
#include <multiVehicleTool/VehicleScanner.h>
#include <limits>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/signalfd.h>
#include <sys/time.h>

std::shared_ptr<CommandPrompt> CommandPrompt::singleton_;

extern bool runInBackground_;

static GMainLoop *event_loop;
static GString *prompt;

VehicleScanner vehicleScanner_;

struct VehicleSelection
{
	bdaddr_t address_{{0, 0, 0, 0, 0, 0}};

	struct iterator
	{
		std::size_t index_;

		iterator(std::size_t index) : index_(index) {}

		bool operator!=(const iterator &it)
		{
			return index_ != it.index_;
		}
		iterator &operator++()
		{
			++index_;
			return *this;
		}
		Vehicle *operator*() const
		{
			return CommandPrompt::getInstance().getVehicleManager()->getVehicleByIndex(index_);
		}
		Vehicle *operator->() const
		{
			return CommandPrompt::getInstance().getVehicleManager()->getVehicleByIndex(index_);
		}
	};

	void selectAll()
	{
		address_ = bdaddr_t{{0, 0, 0, 0, 0, 0}};
	}

	bool isBroadcast() const
	{
		return address_ == bdaddr_t{{0, 0, 0, 0, 0, 0}};
	}

	void selectDummy()
	{
		address_ = bdaddr_t{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
	}

	bool isDummy() const
	{
		return address_ == bdaddr_t{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
	}

	iterator begin()
	{
		if (isBroadcast() || isDummy())
			return iterator(0);
		else
			return iterator(CommandPrompt::getInstance().getVehicleManager()->getVehicleIndexByAddress(address_));
	}

	iterator end()
	{
		if (isBroadcast())
			return iterator(CommandPrompt::getInstance().getVehicleManager()->size());
		else if (isDummy())
		{
			return iterator(0);
		}
		else
		{
			std::size_t index = CommandPrompt::getInstance().getVehicleManager()->getVehicleIndexByAddress(address_);

			return iterator(index == std::numeric_limits<std::size_t>::max() ? index : index + 1);
		}
	}
};

VehicleSelection selection_;

guint stdin_source = 0;

static void cmd_help(int argcp, char **argvp);
static void usage(void (*func)(int argcp, char **argvp));

static void parse_line(char *line_read, bool addHistory);
static void parse_line(char *line_read)
{
	parse_line(line_read, true);
}

#define error(fmt, arg...) rl_printf(COLOR_RED "Error: " COLOR_OFF fmt, ##arg)
#define failed(fmt, arg...) rl_printf(COLOR_RED "Command Failed: " COLOR_OFF fmt, ##arg)

void rl_printf(const char *fmt, ...)
{
	va_list args;
	bool save_input;
	char *saved_line;
	int saved_point;
	struct timeval ts_now;

	save_input = !RL_ISSTATE(RL_STATE_DONE);

	if (save_input)
	{
		saved_point = rl_point;
		saved_line = rl_copy_text(0, rl_end);
		rl_save_prompt();
		rl_replace_line("", 0);
		rl_redisplay();
	}

	va_start(args, fmt);
	gettimeofday(&ts_now, NULL);
	printf("%i.%03i: ", static_cast<int>(ts_now.tv_sec), static_cast<int>(ts_now.tv_usec / 1000));
	vprintf(fmt, args);
	va_end(args);

	if (save_input)
	{
		rl_restore_prompt();
		rl_replace_line(saved_line, 0);
		rl_point = saved_point;
		rl_redisplay();
		free(saved_line);
	}
}

char *get_prompt(void)
{
	g_string_assign(prompt, "");

	if (runInBackground_)
		return prompt->str;

	if (selection_.isBroadcast())
	{
		if (CommandPrompt::getInstance().getVehicleManager()->getNumConnectedVehicles() > 0)
		{
			g_string_append(prompt, COLOR_BLUE);
		}

		g_string_append_printf(prompt, "[    Broadcast (0)]");
	}
	else if (selection_.isDummy())
	{
		g_string_append_printf(prompt, "[     Dummy (0xFF)]");
	}
	else
	{
		Vehicle *vehicle = CommandPrompt::getInstance().getVehicleManager()->getVehicleByAddress(selection_.address_);

		if (vehicle)
		{
			if (vehicle->isConnected())
			{
				g_string_append(prompt, COLOR_BLUE);
			}

			g_string_append_printf(prompt, "[%17s]", vehicle->name_.c_str());
		}
		else
		{
			g_string_append(prompt, COLOR_RED);
			g_string_append_printf(prompt, "[%17s]", vehicle->getMACAddress().c_str());
		}
	}

	g_string_append(prompt, COLOR_OFF);

	g_string_append(prompt, "> ");

	return prompt->str;
}

void CommandPrompt::updatePrompt()
{
	if (prompt != nullptr)
		rl_set_prompt(get_prompt());
}

static void cmd_scan(int argcp, char **argvp)
{
	if (argcp > 2)
	{
		usage(cmd_scan);
		return;
	}

	int8_t devId = -1;

	if (argcp > 1)
	{
		devId = (int8_t)atoi(argvp[1]);
	}

	vehicleScanner_.scan(devId);
	vehicleScanner_.updateVehicleManager(*CommandPrompt::getInstance().getVehicleManager());
}

static void cmd_exit(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_exit);
		return;
	}

	g_main_loop_quit(event_loop);
}

static void cmd_connect(int argcp, char **argvp)
{
	if (argcp > 2)
	{
		usage(cmd_connect);
		return;
	}

	if (argcp == 2) // connect was called with the vehicle name, so we need to select the vehicle first
	{
		auto vehicleManager = CommandPrompt::getInstance().getVehicleManager();

		bool found = false;
		for (std::size_t i = 0; i < vehicleManager->size(); ++i)
		{
			if (boost::iequals((*vehicleManager)[i].name_, argvp[1]))
			{
				selection_.address_ = (*vehicleManager)[i].address_;
				found = true;
				break;
			}
		}

		if (!found)
		{
			selection_.selectDummy();
			error("Invalid vehicle name.\n");
		}

		CommandPrompt::getInstance().updatePrompt();
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->connect(3, true);

	if (CommandPrompt::getInstance().getVehicleManager()->isWaitingForPendingConnections())
		CommandPrompt::getInstance().disableStandardInput();
}

static void cmd_disconnect(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_disconnect);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->disconnect();
}

static void cmd_disrupt(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_disrupt);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->disruptAlienConnection();
}

static void cmd_anki_vehicle_read(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_read);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->read();
}

static void cmd_ankiwatch_set_lights_pattern(int argcp, char **argvp)
{
	if (argcp != 6)
	{
		usage(cmd_ankiwatch_set_lights_pattern);
		return;
	}

	try
	{
		const char *channelsByName[] = {"RED", "TAIL", "BLUE", "GREEN", "FRONTL", "FRONTR"};
		std::size_t numChannels = sizeof(channelsByName) / sizeof(const char *);
		assert(numChannels <= LIGHT_COUNT);

		std::size_t channel = std::find_if(channelsByName, &channelsByName[numChannels], [argvp](const char *x) { return boost::iequals(x, argvp[1]); }) - channelsByName;
		if (channel >= numChannels)
			throw std::invalid_argument("Channel must be one of RED, TAIL, BLUE, GREEN, FRONTL, FRONTR.");

		const char *effectsByName[] = {"STEADY", "FADE", "THROB", "FLASH", "RANDOM"};
		std::size_t numEffects = sizeof(effectsByName) / sizeof(const char *);
		assert(numEffects <= EFFECT_COUNT);

		std::size_t effect = std::find_if(effectsByName, &effectsByName[numEffects], [argvp](const char *x) { return boost::iequals(x, argvp[2]); }) - effectsByName;
		if (effect >= numEffects)
			throw std::invalid_argument("Effect must be one of STEADY, FADE, THROB, FLASH, RANDOM.");

		uint8_t start = boost::numeric_cast<uint8_t>(boost::lexical_cast<std::size_t>(argvp[3]));
		uint8_t end = boost::numeric_cast<uint8_t>(boost::lexical_cast<std::size_t>(argvp[4]));
		uint16_t cyclesPerMinute = boost::numeric_cast<uint16_t>(boost::lexical_cast<std::size_t>(argvp[5]));

		for (auto it = selection_.begin(); it != selection_.end(); ++it)
			it->setLightsPattern(static_cast<anki_vehicle_light_channel_t>(channel), static_cast<anki_vehicle_light_effect_t>(effect), start, end, cyclesPerMinute);
	}
	catch (const std::exception &err)
	{
		error("%s\n", err.what());
	}
}
static void cmd_anki_vehicle_disconnect(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_disconnect);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->disconnectPolitely();
}

static void cmd_anki_vehicle_sdk_mode(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_anki_vehicle_sdk_mode);
		return;
	}

	int arg = atoi(argvp[1]);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->setSDKMode(arg);
}

static void cmd_anki_vehicle_ping(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_ping);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->ping();
}

static void cmd_anki_vehicle_get_version(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_get_version);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->requestVersion();
}

static void cmd_anki_vehicle_set_speed(int argcp, char **argvp)
{
	if (argcp < 2 || argcp > 3)
	{
		usage(cmd_anki_vehicle_set_speed);
		return;
	}

	uint16_t speed = (uint16_t)atoi(argvp[1]);
	uint16_t acceleration = argcp > 2 ? atoi(argvp[2]) : 25000;

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->setSpeed(speed, acceleration);
}

static void cmd_anki_vehicle_change_lane(int argcp, char **argvp)
{
	if (argcp != 4)
	{
		usage(cmd_anki_vehicle_change_lane);
		return;
	}

	uint16_t speed = (uint16_t)atoi(argvp[1]);
	uint16_t acceleration = (uint16_t)atoi(argvp[2]);
	float offset = strtof(argvp[3], NULL);

	rl_printf("changing lane at %d (acceleration = %d | offset from position = %1.2f)\n", speed, acceleration, offset);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->changeLane(speed, acceleration, offset);
}

static void cmd_anki_vehicle_change_lane_abs(int argcp, char **argvp)
{
	if (argcp != 4)
	{
		usage(cmd_anki_vehicle_change_lane_abs);
		return;
	}

	uint16_t speed = (uint16_t)atoi(argvp[1]);
	uint16_t acceleration = (uint16_t)atoi(argvp[2]);
	float offset = strtof(argvp[3], NULL);

	rl_printf("changing lane at %d (acceleration = %d | offset from road center = %1.2f)\n", speed, acceleration, offset);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->changeLaneAbs(speed, acceleration, offset);
}

static void cmd_anki_vehicle_cancel_lane_change(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_cancel_lane_change);
		return;
	}

	rl_printf("cancelling lane change\n");

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->cancelLaneChange();
}

static void cmd_anki_vehicle_set_offset(int argcp, char **argvp)
{
	if (argcp != 1 && argcp != 2)
	{
		usage(cmd_anki_vehicle_set_offset);
		return;
	}

	float offset = (argcp == 1 ? 0.0f : strtof(argvp[1], NULL));

	rl_printf("set road offset (offset = %1.2f)\n", offset);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->setOffset(offset);
}

static void cmd_anki_vehicle_correct_offset(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_anki_vehicle_correct_offset);
		return;
	}

	float delta = strtof(argvp[1], NULL);

	rl_printf("correct offset (delta = %1.2f)\n", delta);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->correctOffset(delta);
}

static void cmd_anki_vehicle_configure_track(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_anki_vehicle_configure_track);
		return;
	}

	uint8_t numberOfLanes = (uint8_t)atoi(argvp[1]);

	rl_printf("configure track (number of lanes = %d)\n", numberOfLanes);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->configureTrack(numberOfLanes);
}

static void cmd_anki_vehicle_turn_180(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_turn_180);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->uturn();
}

static void cmd_anki_vehicle_get_battery(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_anki_vehicle_get_battery);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->requestVoltage();
}

static void cmd_anki_vehicle_lights(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_anki_vehicle_lights);
		return;
	}

	// 0x44 = turn on front lights
	// 0x04 = turn off front lights
	// 0x22 = turn on back lights
	// 0x02 = turn off back lights
	// 0x88 = let back light blink
	// 0x08 = stop to blink back light
	// 0xff = turn on all lights
	// 0x0f = turn off all lights

	uint8_t val = strtoul(argvp[1], NULL, 16);

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->setLights(val);
}

static void cmd_set_verbose(int argcp, char **argvp)
{

	if (argcp < 1 || argcp > 2)
	{
		usage(cmd_set_verbose);
		return;
	}

	if (argcp == 1)
	{
		rl_printf("Verbosity level is set to %d.\n", CommandPrompt::getInstance().getVerbose());
		return;
	}

	errno = 0;
	int verbose = strtoll(argvp[1], NULL, 0);
	if (errno != 0 || verbose > 2 || verbose < 0)
	{
		error("Invalid value. Maximum verbosity is 2.\n");
		return;
	}

	CommandPrompt::getInstance().setVerbose(verbose);
}

static void cmd_list_vehicles(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_list_vehicles);
		return;
	}

	for (std::size_t i = 0; i < CommandPrompt::getInstance().getVehicleManager()->getNumVehicles(); ++i)
	{
		Vehicle *curvehicle = CommandPrompt::getInstance().getVehicleManager()->getVehicleByIndex(i);

		GString *line = g_string_new(NULL);

		g_string_printf(line, "%2zu %12s %18s  v%04x", i + 1, curvehicle->name_.c_str(), curvehicle->getMACAddress().c_str(), curvehicle->getVersion());

		if (curvehicle->getVersion() == 0)
			g_string_append(line, "(U) ");
		else if (curvehicle->getVersion() <= 0x2159)
			g_string_append(line, "(D) ");
		else
			g_string_append(line, "(OD)");

		if (curvehicle->isConnected())
		{
			g_string_append(line, COLOR_BLUE);
			g_string_append_printf(line, "   Connected via hci%i", curvehicle->hciDevice_->devId_);
			g_string_append(line, COLOR_OFF);
		}
		else if (curvehicle->isConnecting())
		{
			g_string_append(line, "   Connecting");
		}
		else if (curvehicle->shouldConnect())
		{
			g_string_append(line, "   Should connect");
		}
		else
		{
			g_string_append(line, "   Disconnected");
		}

		rl_printf("%s\n", line->str);
	}
}

static void cmd_select_vehicle(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_select_vehicle);
		return;
	}

	std::size_t vehicleid;
	try
	{
		vehicleid = boost::lexical_cast<std::size_t>(argvp[1]);
	}
	catch (boost::bad_lexical_cast)
	{
		vehicleid = std::numeric_limits<std::size_t>::max();
	}

	if (vehicleid != std::numeric_limits<std::size_t>::max())
	{
		if (vehicleid > CommandPrompt::getInstance().getVehicleManager()->size())
		{
			selection_.selectDummy();
			error("Invalid vehicle id.\n");
		}
		else if (vehicleid == 0)
		{
			selection_.selectAll();
		}
		else
		{
			selection_.address_ = CommandPrompt::getInstance().getVehicleManager()->getVehicleByIndex(vehicleid - 1)->address_;
		}
	}
	else if (strlen(argvp[1]) == 17)
	{
		str2ba(argvp[1], &selection_.address_);
	}
	else
	{
		auto vehicleManager = CommandPrompt::getInstance().getVehicleManager();

		bool found = false;
		for (std::size_t i = 0; i < vehicleManager->size(); ++i)
		{
			if (boost::iequals((*vehicleManager)[i].name_, argvp[1]))
			{
				selection_.address_ = (*vehicleManager)[i].address_;
				found = true;
				break;
			}
		}

		if (!found)
		{
			selection_.selectDummy();
			error("Invalid vehicle name.\n");
		}
	}

	CommandPrompt::getInstance().updatePrompt();
}

static void cmd_sleep(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_sleep);
		return;
	}

	double sleeptime = -1;

	try
	{
		sleeptime = boost::lexical_cast<double>(argvp[1]);
	}
	catch (const boost::bad_lexical_cast &)
	{
	}

	if (sleeptime < 0)
	{
		error("%s: First argument needs to be a non-negative integer.\n", argvp[0]);
		return;
	}

	CommandPrompt::getInstance().disableStandardInput();
	g_timeout_add(static_cast<guint>(std::round(sleeptime * 1000.0)), CommandPrompt::dispatchOnResetupStandardInput, &CommandPrompt::getInstance());
}

static std::ifstream inputFile_;

static gboolean parse_file(gpointer user_data)
{
	if (!CommandPrompt::getInstance().isAcceptingInput())
		return TRUE;

	std::string line;
	std::getline(inputFile_, line);
	char *buffer = static_cast<char *>(malloc(line.size() + 1));
	strcpy(buffer, line.c_str());
	if (CommandPrompt::getInstance().getVerbose() > 0)
		rl_printf("exec: %s\n", buffer);
	parse_line(buffer, false);

	return (inputFile_ ? TRUE : FALSE);
}

static gboolean parse_stdin_notty(gpointer user_data)
{
	if (!CommandPrompt::getInstance().isAcceptingInput())
		return TRUE;

	std::string line;
	std::getline(std::cin, line);
	char *buffer = static_cast<char *>(malloc(line.size() + 1));
	strcpy(buffer, line.c_str());
	if (CommandPrompt::getInstance().getVerbose() > 0)
		rl_printf("exec: %s\n", buffer);
	parse_line(buffer, false);

	if (!std::cin)
	{
		g_main_loop_quit(event_loop);
		return FALSE;
	}

	return TRUE;
}

static void cmd_execute(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_execute);
		return;
	}

	inputFile_.close();

	inputFile_.open(argvp[1]);
	if (!inputFile_)
	{
		error("Cannot open '%s' for execution.\n", argvp[1]);
		return;
	}

	g_idle_add(parse_file, nullptr);
}

static void cmd_check(int argcp, char **argvp)
{
	if (argcp <= 1)
	{
		usage(cmd_check);
		return;
	}

	if (std::string(argvp[1]) == "connected-vehicles")
	{
		Vehicle *vehicleToCheck = nullptr;
		for (int i = 2; i < argcp; ++i)
		{
			vehicleToCheck = CommandPrompt::getInstance().getVehicleManager()->getVehicleByName(std::string(argvp[i]));
			if (vehicleToCheck == nullptr)
			{
				error("%s: Not a valid vehicle name.\n", argvp[i]);
				continue;
			}

			vehicleToCheck->connect(0, true);
		}

		if (CommandPrompt::getInstance().getVehicleManager()->isWaitingForPendingConnections())
			CommandPrompt::getInstance().disableStandardInput();
	}
	else
		usage(cmd_check);
}


static void cmd_set_material(int argcp, char **argvp)
{
	if (argcp != 2)
	{
		usage(cmd_set_material);
		return;
	}

	anki_track_material_t material;
	if (boost::iequals("vinyl", argvp[1]))
		material = TRACK_MATERIAL_VINYL;
	else if (boost::iequals("plastic", argvp[1]))
		material = TRACK_MATERIAL_PLASTIC;
	else
	{
		usage(cmd_set_material);
		return;
	}

	for (auto it = selection_.begin(); it != selection_.end(); ++it)
		it->setConfigParameters(SUPERCODE_ALL, material);
}

static void cmd_print_hci(int argcp, char **argvp)
{
	HciManager::printHciState();
}

static struct
{
	const char *cmd;
	void (*func)(int argcp, char **argvp);
	const char *params;
	const char *desc;
} commands[] = {
    {"help", cmd_help, "", "Show this help"},
    {"exit", cmd_exit, "", "Exit interactive mode"},
    {"quit", cmd_exit, "", "Exit interactive mode"},
    {"scan", cmd_scan, "<devid>", "Scan for vehicles"},
	{"connect", cmd_connect, "<vehicle-name>", "Connect to a remote device. Works standalone (connect HADION0) or after select-vehicle"},
    {"disconnect", cmd_disconnect, "", "Disconnect from a remote device"},
    {"disrupt", cmd_disrupt, "", "Disrupt alien connections to remote device"},
    {"list-vehicles", cmd_list_vehicles, "", "Show configured vehicles and connection status"},
    {"select-vehicle", cmd_select_vehicle, "<vehicle-id>", "Select vehicle to control"},
    {"sdk-mode", cmd_anki_vehicle_sdk_mode, "<state>", "Set SDK Mode"},
    {"ping", cmd_anki_vehicle_ping, "", "Send ping message to vehicle."},
    {"get-version", cmd_anki_vehicle_get_version, "", "Request vehicle software version."},
    {"get-battery", cmd_anki_vehicle_get_battery, "", "Request vehicle battery level."},
    {"set-speed", cmd_anki_vehicle_set_speed, "<speed> [acceleration]", "Set vehicle Speed (mm/sec) with acceleration (mm/sec^2)"},
    {"change-lane", cmd_anki_vehicle_change_lane, "<horizontal speed> <horizontal acceleration> <relative offset> (right(+), left(-))", "Change lanes at speed (mm/sec) and acceleration (mm/sec^2) in the specified direction (offset) and sets the current position as the road center."},
    {"change-lane-abs", cmd_anki_vehicle_change_lane_abs, "<horizontal speed> <horizontal acceleration> <relative offset> (right(+), left(-))", "Change lanes at speed (mm/sec) and acceleration (mm/sec^2) to the given lateral position relative to the road center (right(+), left(-)) specified with set-offset."},
    {"cancel-lane-change", cmd_anki_vehicle_cancel_lane_change, "", "Cancels any lane change in progress."},
    {"set-offset", cmd_anki_vehicle_set_offset, "[offset]", "Sets the current lateral position in mm from the road center."},
    {"correct-offset", cmd_anki_vehicle_correct_offset, "[delta]", "Corrects the lateral position by a delta in mm."},
    {"uturn", cmd_anki_vehicle_turn_180, "", "Turn vehicle by 180 degree"},
	{"set-lights", cmd_anki_vehicle_lights, "<hex value>", "Set lights (0x22 = back on, 0x02 = back off, 0x44 = front on, 0x04 = front off, 0x88 = back blinks, 0x08 = back off, 0xff = all on, 0x0f = all off)."},
    {"set-lights-pattern", cmd_ankiwatch_set_lights_pattern,  "<channel> <effect> <start> <end> <cycles_per_min>", "Set lights pattern for vehicle LEDs (channel must be one of RED, TAIL, BLUE, GREEN, FRONTL, FRONTR and effect must be one of STEADY, FADE, THROB, FLASH, RANDOM)."},
    {"vehicle-disconnect", cmd_anki_vehicle_disconnect, "", "Request that the vehicle disconnects (often more reliable than disconnect)"},
    {"read-data", cmd_anki_vehicle_read, "", "Read last message from vehicle"},
    {"verbose", cmd_set_verbose, "[new value]", "Verbosity 0=silent, 1=known updates (default), 2=everything"},
    {"sleep", cmd_sleep, "<seconds>", "buffer commands for <seconds>"},
    {"execute", cmd_execute, "<filename>", "Execute MultiVehicleTool script"},
    {"check", cmd_check, "connected-vehicles <vehicle names>", "check if all vehicles are connected before continuing, if not, attempt to reconnect"},
	{"set-material", cmd_set_material,  "[vinyl/plastic]", "set track material to \"vinyl\" or \"plastic\""},
    {"hci-state", cmd_print_hci, "", "Print state of available HCI Devices"},
    {NULL, NULL, NULL, NULL}};

static void cmd_help(int argcp, char **argvp)
{
	if (argcp != 1)
	{
		usage(cmd_help);
		return;
	}

	for (std::size_t i = 0; commands[i].cmd; ++i)
		rl_printf("%-15s %-20s %s\n", commands[i].cmd, commands[i].params, commands[i].desc);
}

static void usage(void (*func)(int argcp, char **argvp))
{
	for (std::size_t i = 0; commands[i].cmd; ++i)
	{
		if (commands[i].func == func)
		{
			if (strlen(commands[i].params) == 0)
				rl_printf(COLOR_RED "Usage:" COLOR_OFF " %s\n", commands[i].cmd);
			else
				rl_printf(COLOR_RED "Usage:" COLOR_OFF " %s %s\n", commands[i].cmd, commands[i].params);
		}
	}
}

static void parse_line(char *line_read, bool addHistory)
{
	char **argvp;
	int argcp;
	int i;

	if (line_read == NULL)
	{
		printf("\n");
		cmd_exit(1, NULL);
		return;
	}

	line_read = g_strstrip(line_read);

	if (*line_read == '\0')
		goto done;

	if (addHistory)
		add_history(line_read);

	if (g_shell_parse_argv(line_read, &argcp, &argvp, NULL) == FALSE)
		goto done;

	for (i = 0; commands[i].cmd; i++)
		if (strcasecmp(commands[i].cmd, argvp[0]) == 0)
			break;

	if (commands[i].cmd)
		commands[i].func(argcp, argvp);
	else
		error("%s: command not found\n", argvp[0]);

	g_strfreev(argvp);

done:
	free(line_read);
}

static gboolean prompt_read(GIOChannel *chan, GIOCondition cond,
                            gpointer user_data)
{
	if (cond & (G_IO_HUP | G_IO_ERR | G_IO_NVAL))
	{
		g_io_channel_unref(chan);
		return FALSE;
	}

	rl_callback_read_char();

	return TRUE;
}

static char *completion_generator(const char *text, int state)
{
	static int index = 0, len = 0;
	const char *cmd = NULL;

	if (state == 0)
	{
		index = 0;
		len = strlen(text);
	}

	while ((cmd = commands[index].cmd) != NULL)
	{
		index++;
		if (strncmp(cmd, text, len) == 0)
			return strdup(cmd);
	}

	return NULL;
}

static char *vehicle_completion_generator(const char *text, int state)
{
	std::string str(text);
	Vehicle *vehicle = CommandPrompt::getInstance().getVehicleManager()->getVehicleByIncompleteName(str, state);
	if (vehicle != nullptr)
	{
		return strdup(vehicle->name_.c_str());
	}

	return NULL;
}

static char **commands_completion(const char *text, int start, int end)
{
	if (start == 0)
		return rl_completion_matches(text, &completion_generator);
	else if (strncmp("select-vehicle", rl_line_buffer, 14) == 0)
		return rl_completion_matches(text, &vehicle_completion_generator);
	else
		return nullptr;
}

void CommandPrompt::setupStandardInput()
{
	if (!runInBackground_)
	{
		GIOChannel *channel;

		if (stdin_source > 0)
			return;

		if (!isatty(fileno(stdin)))
		{
			g_idle_add(parse_stdin_notty, nullptr);
		}
		else
		{
			channel = g_io_channel_unix_new(fileno(stdin));

			stdin_source = g_io_add_watch(channel,
			                              (GIOCondition)(G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL),
			                              prompt_read, NULL);

			g_io_channel_unref(channel);
		}
	}
	acceptInput_ = true;
}

void CommandPrompt::disableStandardInput()
{
	if (stdin_source > 0)
	{
		g_source_remove(stdin_source);
		stdin_source = 0;
	}

	acceptInput_ = false;
}

static gboolean signal_handler(GIOChannel *channel, GIOCondition condition, gpointer user_data)
{
	static unsigned int __terminated = 0;
	struct signalfd_siginfo si;
	ssize_t result;
	int fd;

	if (condition & (G_IO_NVAL | G_IO_ERR | G_IO_HUP))
	{
		g_main_loop_quit(event_loop);
		return FALSE;
	}

	fd = g_io_channel_unix_get_fd(channel);

	result = read(fd, &si, sizeof(si));
	if (result != sizeof(si))
		return FALSE;

	switch (si.ssi_signo)
	{
	case SIGINT:
	case SIGTERM:
		if (__terminated == 0)
		{
			rl_replace_line("", 0);
			rl_crlf();
			g_main_loop_quit(event_loop);
		}

		__terminated = 1;
		break;
	}

	return TRUE;
}

static guint setup_signalfd(void)
{
	GIOChannel *channel;
	guint source;
	sigset_t mask;
	int fd;

	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGTERM);

	if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0)
	{
		perror("Failed to set signal mask");
		return 0;
	}

	fd = signalfd(-1, &mask, 0);
	if (fd < 0)
	{
		perror("Failed to create signal descriptor");
		return 0;
	}

	channel = g_io_channel_unix_new(fd);

	g_io_channel_set_close_on_unref(channel, TRUE);
	g_io_channel_set_encoding(channel, NULL, NULL);
	g_io_channel_set_buffered(channel, FALSE);

	source = g_io_add_watch(channel,
	                        (GIOCondition)(G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL),
	                        signal_handler, NULL);

	g_io_channel_unref(channel);

	return source;
}

gboolean testResponsiveness(gpointer)
{
	const double period = 20.0 * 0.001;
	static Timestamp t0 = Timestamp::clock::now() - toTimestampDuration(period);
	Timestamp t = Timestamp::clock::now();

	double dt = toSeconds(t - t0);
	if (std::abs(dt - period) > 0.25 * period)
	{
		//rl_printf(COLOR_RED "Error: " COLOR_OFF "Event queue delayed by %0.2lf ms.\n", (dt - period) * 1000.0);
	}

	t0 = t;

	return TRUE;
}

int interactive()
{
	guint signal;

	prompt = g_string_new(NULL);

	event_loop = g_main_loop_new(NULL, FALSE);

	CommandPrompt::getInstance().setupStandardInput();
	signal = setup_signalfd();

	rl_attempted_completion_function = commands_completion;
	rl_erase_empty_line = 1;
	rl_callback_handler_install(get_prompt(), parse_line);

	HciManager::searchForHciDevices();
	CommandPrompt::getInstance().getVehicleManager()->loadVehicleList();

#if 0
	g_timeout_add(20, testResponsiveness, nullptr);
#endif

	g_main_loop_run(event_loop);

	rl_callback_handler_remove();

	// Notify all vehicles of imminent disconnection so that vehicles reliably disconnect.
	std::shared_ptr<VehicleManager> manager = CommandPrompt::getInstance().getVehicleManager();
	for (std::size_t i = 0; i < manager->size(); ++i)
		(*manager)[i].disconnectPolitely();

	// Service bluez and others.
	g_main_context_iteration(nullptr, FALSE);

	// Disconnect local endpoint of bluetooth connection and thus trigger TCP socket disconnect messages.
	for (std::size_t i = 0; i < manager->size(); ++i)
		(*manager)[i].disconnect();

	// Service bluez and others.
	g_main_context_iteration(nullptr, FALSE);

	// Destruct vehicle manager
	manager.reset();

	CommandPrompt::getInstance().disableStandardInput();
	g_source_remove(signal);
	g_main_loop_unref(event_loop);
	g_string_free(prompt, TRUE);
	prompt = nullptr;

	return 0;
}

void CommandPrompt::execute()
{
	interactive();
	printf("\n");
}

gboolean CommandPrompt::dispatchOnResetupStandardInput(gpointer userData)
{
	assert(userData);
	CommandPrompt *commandPrompt = static_cast<CommandPrompt *>(userData);
	commandPrompt->onResetupStandardInput();

	return FALSE;
}
