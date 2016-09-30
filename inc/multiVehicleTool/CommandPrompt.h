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

#ifndef MULTIVEHICLETOOL_COMMANDPROMPT_H
#define MULTIVEHICLETOOL_COMMANDPROMPT_H

#include <multiVehicleTool/VehicleManager.h>
#include <memory>

#define COLOR_OFF "\x1B[0m"
#define COLOR_RED "\x1B[0;91m"
#define COLOR_GREEN "\x1B[0;92m"
#define COLOR_YELLOW "\x1B[0;93m"
#define COLOR_BLUE "\x1B[0;94m"
#define COLOR_BOLDGRAY "\x1B[1;30m"
#define COLOR_BOLDWHITE "\x1B[1;37m"

void rl_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

class CommandPrompt
{
private:
	CommandPrompt();

	static std::shared_ptr<CommandPrompt> singleton_;

	std::shared_ptr<VehicleManager> manager_;

	int verbose_ = 0;
	bool acceptInput_{true};

public:
	static CommandPrompt &getInstance();

	std::shared_ptr<const VehicleManager> getVehicleManager() const;
	std::shared_ptr<VehicleManager> getVehicleManager();
	void setVehicleManager(std::shared_ptr<VehicleManager> manager);

	void setVerbose(int verbose);
	int getVerbose() const;

	bool isAcceptingInput() const;

	void updatePrompt();
	void execute();

	void setupStandardInput();
	void disableStandardInput();
	void onResetupStandardInput();
	static gboolean dispatchOnResetupStandardInput(gpointer data);
};

inline CommandPrompt::CommandPrompt()
{
}

inline CommandPrompt &CommandPrompt::getInstance()
{
	if (!singleton_)
		singleton_.reset(new CommandPrompt());

	return *singleton_;
}

inline std::shared_ptr<const VehicleManager> CommandPrompt::getVehicleManager() const
{
	return manager_;
}

inline std::shared_ptr<VehicleManager> CommandPrompt::getVehicleManager()
{
	return manager_;
}

inline void CommandPrompt::setVehicleManager(std::shared_ptr<VehicleManager> manager)
{
	manager_ = manager;
}

inline void CommandPrompt::setVerbose(int verbose)
{
	verbose_ = verbose;
}

inline int CommandPrompt::getVerbose() const
{
	return verbose_;
}

inline bool CommandPrompt::isAcceptingInput() const
{
	return acceptInput_;
}

inline void CommandPrompt::onResetupStandardInput()
{
	setupStandardInput();
}

#endif
