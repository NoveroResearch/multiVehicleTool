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

#ifndef MULTIVEHICLETOOL_UTIL_TIMESTAMP_H
#define MULTIVEHICLETOOL_UTIL_TIMESTAMP_H

#include <chrono>

typedef std::chrono::time_point<std::chrono::steady_clock> Timestamp;

template <class Rep, class Period>
inline double toSeconds(const std::chrono::duration<Rep, Period> &duration)
{
	return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

template <class Rep, class Period>
inline double toMilliSeconds(const std::chrono::duration<Rep, Period> &duration)
{
	return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration).count();
}

inline Timestamp::duration toTimestampDuration(double duration)
{
	return std::chrono::duration_cast<Timestamp::duration>(std::chrono::duration<double>(duration));
}

#endif
