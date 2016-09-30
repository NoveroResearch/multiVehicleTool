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

#ifndef MULTIVEHICLETOOL_VEHICLEMODEL_H
#define MULTIVEHICLETOOL_VEHICLEMODEL_H

#include <cstdint>
#include <string>

struct VehicleModel
{
	enum VehicleModelID
	{
		VEHICLEMODEL_UNKNOWN = 0,
		VEHICLEMODEL_KOURAI = 1,
		VEHICLEMODEL_BOSON = 2,
		VEHICLEMODEL_RHO = 3,
		VEHICLEMODEL_KATAL = 4,
		VEHICLEMODEL_CORAX = 5,
		VEHICLEMODEL_HADION = 6,
		VEHICLEMODEL_SPEKTRIX = 7
	};

	VehicleModel(uint8_t model = VEHICLEMODEL_UNKNOWN);
	VehicleModel(const std::string &name);

	uint8_t getID() const;
	std::string getName() const;
#if 0
	cv::Scalar getColor() const;
#endif

private:
	uint8_t model_;
};

inline VehicleModel::VehicleModel(uint8_t model) : model_(model)
{
}

#endif
