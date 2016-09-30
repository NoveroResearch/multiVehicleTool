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

#include <boost/filesystem.hpp>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/util/PidFile.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include <signal.h>

PidFile::PidFile(std::string binaryPath)
{
	boost::filesystem::path binaryName(binaryPath);

	std::stringstream sstr;
	sstr << "/tmp/" << binaryName.filename().string() << ".pid";
	pidfilePath_ = sstr.str();

	killRunningProcess();

	createPidFile();
}

PidFile::~PidFile()
{
	boost::filesystem::remove(pidfilePath_);
}

void PidFile::killRunningProcess()
{
	assert(pidfilePath_.size() > 0);

	std::ifstream infile(pidfilePath_);
	if (infile.is_open())
	{
		std::cerr << "MultiVehicleTool is already running, killing it..." << std::endl;

		pid_t pid = 0;
		infile >> pid;
		infile.close();

		if (pid > 0 && kill(pid, SIGTERM) == 0)
		{
			int timeout = 0;
			while (boost::filesystem::exists(pidfilePath_))
			{
				usleep(1000);
				if (++timeout > 5000)
				{
					std::cerr << "pidfile still existing after 5s despite kill. is the process a zombie?" << std::endl;
					std::exit(EXIT_FAILURE);
				}
			}
		}
		else
		{
			if (pid == 0 || errno == ESRCH)
			{
				// process not existing, removing pid
				boost::filesystem::remove(pidfilePath_);
			}
			else
			{
				std::cerr << "killing " << pid << " failed with error " << errno << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
	}
}

void PidFile::createPidFile()
{
	assert(pidfilePath_.size() > 0);

	std::ofstream outfile(pidfilePath_);
	if (!outfile.is_open())
	{
		std::cerr << "couldn't create pidfile" << std::endl;
		std::exit(EXIT_FAILURE);
	}

	outfile << getpid() << std::endl;
}
