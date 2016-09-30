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

#include <boost/algorithm/string.hpp>
#include <multiVehicleTool/config.h>
#include <multiVehicleTool/VehicleModel.h>

VehicleModel::VehicleModel(const std::string &name)
{
	if (boost::iequals(name, "kourai"))
		model_ = VEHICLEMODEL_KOURAI;
	else if (boost::iequals(name, "boson"))
		model_ = VEHICLEMODEL_BOSON;
	else if (boost::iequals(name, "rho"))
		model_ = VEHICLEMODEL_RHO;
	else if (boost::iequals(name, "katal"))
		model_ = VEHICLEMODEL_KATAL;
	else if (boost::iequals(name, "corax"))
		model_ = VEHICLEMODEL_CORAX;
	else if (boost::iequals(name, "hadion"))
		model_ = VEHICLEMODEL_HADION;
	else if (boost::iequals(name, "spektrix"))
		model_ = VEHICLEMODEL_SPEKTRIX;
	else
		model_ = VEHICLEMODEL_UNKNOWN;
}

uint8_t VehicleModel::getID() const
{
	return model_;
}

std::string VehicleModel::getName() const
{
	switch (model_)
	{
	default:
	case VEHICLEMODEL_UNKNOWN:
		return "unknown";
	case VEHICLEMODEL_KOURAI:
		return "kourai";
	case VEHICLEMODEL_BOSON:
		return "boson";
	case VEHICLEMODEL_RHO:
		return "rho";
	case VEHICLEMODEL_KATAL:
		return "katal";
	case VEHICLEMODEL_CORAX:
		return "corax";
	case VEHICLEMODEL_HADION:
		return "hadion";
	case VEHICLEMODEL_SPEKTRIX:
		return "spektrix";
	}
}
