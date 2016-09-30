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

#ifndef MULTIVEHICLETOOL_UTIL_PIDFILE_H
#define MULTIVEHICLETOOL_UTIL_PIDFILE_H

#include <string>

/* creates a /tmp/processName.pid file
 * if file already exists checks if the process is existing
 * if process exists --> kill it, wait for termination and start
 */

class PidFile
{
public:
	PidFile(std::string binaryPath);
	~PidFile();

private:
	std::string pidfilePath_ = "";

	PidFile(const PidFile &) = delete;
	PidFile &operator=(const PidFile &) = delete;

	void killRunningProcess();
	void createPidFile();
};

#endif //MULTIVEHICLETOOL_UTIL_PIDFILE_H
