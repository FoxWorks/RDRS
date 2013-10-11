////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
///
/// This program is free software; you can redistribute it and/or modify it under
/// the terms of the GNU Lesser General Public License as published by the Free Software
/// Foundation; either version 2 of the License, or (at your option) any later
/// version.
///
/// This program is distributed in the hope that it will be useful, but WITHOUT
/// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
/// FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
/// details.
///
/// You should have received a copy of the GNU Lesser General Public License along with
/// this program; if not, write to the Free Software Foundation, Inc., 59 Temple
/// Place - Suite 330, Boston, MA  02111-1307, USA.
///
/// Further information about the GNU Lesser General Public License can also be found on
/// the world wide web at http://www.gnu.org.
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include "rdrs.h"



////////////////////////////////////////////////////////////////////////////////
/// @brief Get library version
////////////////////////////////////////////////////////////////////////////////
int RDRS_Version(int* version, char* version_string) {
	if (version) *version = RDRS_VERSION;
	if (version_string) strcpy(version_string,RDRS_VERSION_STRING);
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create RDRS system
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_Create(RDRS_SYSTEM** p_system) {
	RDRS_SYSTEM* system;
	if (!p_system) return RDRS_ERROR_BAD_PARAMETER;

	//Create new system
	system = (RDRS_SYSTEM*)malloc(sizeof(RDRS_SYSTEM));
	*p_system = system;
	if (!system) return RDRS_ERROR_MEMORY;
	memset(system,0,sizeof(RDRS_SYSTEM));

	//Initialize resources
	SIMC_Thread_Initialize();
	SIMC_List_Create(&system->antennas,1);

	//Start at current time
	system->time = SIMC_Thread_GetMJDTime();
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy RDRS system and all resources
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_Destroy(RDRS_SYSTEM* system) {
	SIMC_LIST_ENTRY* entry;
	if (!system) return RDRS_ERROR_BAD_PARAMETER;

	//Destroy resources
	/*entry = SIMC_List_GetFirst(system->antennas);
	while (entry) {
		RDRS_Antenna_Destroy((RDRS_ANTENNA*)SIMC_List_GetData(system->antennas,entry));
		entry = SIMC_List_GetFirst(system->antennas);
	}
	SIMC_List_Destroy(system->antennas);*/

	//Deinitialize system
	SIMC_Thread_Deinitialize();
	free(system);
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Start simulation
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_Start(RDRS_SYSTEM* system) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;
	return RDRS_ERROR_NOT_IMPLEMENTED;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Stop/pause simulation
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_Stop(RDRS_SYSTEM* system) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;
	return RDRS_ERROR_NOT_IMPLEMENTED;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set system MJD time (system asynchronously simulates to this set time)
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_SetTime(RDRS_SYSTEM* system, double time) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;

	system->is_realtime = 0;
	system->time = time;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set system to be realtime (synchronized with system clock)
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_SetRealTime(RDRS_SYSTEM* system, int is_realtime) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;

	system->is_realtime = is_realtime;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get MJD time from system
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_GetTime(RDRS_SYSTEM* system, double* time) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;
	if (!time) return RDRS_ERROR_BAD_PARAMETER;

	if (system->is_realtime) {
		*time = SIMC_Thread_GetMJDTime();
	} else {
		*time = system->time;
	}
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set userdata
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_SetUserdata(RDRS_SYSTEM* system, void* userdata) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;

	system->userdata = userdata;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get userdata
////////////////////////////////////////////////////////////////////////////////
int RDRS_System_GetUserdata(RDRS_SYSTEM* system, void** p_userdata) {
	if (!system) return RDRS_ERROR_BAD_PARAMETER;
	if (!p_userdata) return RDRS_ERROR_BAD_PARAMETER;

	*p_userdata = system->userdata;
	return RDRS_OK;
}