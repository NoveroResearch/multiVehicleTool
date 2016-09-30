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

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/CommandPrompt.h>
#include <multiVehicleTool/util/PidFile.h>


namespace po = boost::program_options;

bool runInBackground_ = false;

std::vector<HciDevice> HciManager::hciDevices_;

void handleCommandlineArguments(int argc, char *argv[])
{
	std::string config_file;
	std::string appname{boost::filesystem::basename(argv[0])};
	po::options_description options("Usage");
	// clang-format off
	options.add_options()
	("help,h", "produce help message")
	("background,b", "run without input prompt");
	// clang-format on
	po::positional_options_description positionalOptions;
	po::options_description config_file_options;
	config_file_options.add(options);

	po::variables_map vm;
	try
	{
		// values from the command line are stored first, so they override values from the config file if both are present
		po::store(po::command_line_parser(argc, argv).options(options).positional(positionalOptions).run(), vm);

		if (vm.count("config-file"))
		{
			config_file = vm["config-file"].as<std::string>();
			std::ifstream ifs(config_file);
			if (!ifs)
				std::cout << "no config file found, using values from the command line" << std::endl;
			else
			{
				store(parse_config_file(ifs, config_file_options, true), vm);
				notify(vm);
			}
		}

		if (vm.count("help"))
		{
			std::cout << appname << "\n\n";
			std::cout << options << "\n";
			std::exit(EXIT_FAILURE);
		}

		if (vm.count("background"))
			runInBackground_ = true;

		po::notify(vm);
	}
	catch (po::error &err)
	{
		std::cout << err.what() << std::endl;
		std::exit(EXIT_FAILURE);
	}
}


int main(int argc, char *argv[])
{
	PidFile pidFile(argv[0]);

	handleCommandlineArguments(argc, argv);

	auto vehicleManager = std::make_shared<VehicleManager>();

	CommandPrompt &prompt = CommandPrompt::getInstance();
	prompt.setVehicleManager(vehicleManager);

	prompt.execute();
}
