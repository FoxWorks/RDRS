////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///   - Redistributions of source code must retain the above copyright
///     notice, this list of conditions and the following disclaimer.
///   - Redistributions in binary form must reproduce the above copyright
///     notice, this list of conditions and the following disclaimer in the
///     documentation and/or other materials provided with the distribution.
///   - Neither the name of the author nor the names of the contributors may
///     be used to endorse or promote products derived from this software without
///     specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
	entry = SIMC_List_GetFirst(system->antennas);
	while (entry) {
		RDRS_Antenna_Destroy((RDRS_ANTENNA*)SIMC_List_GetData(system->antennas,entry));
		entry = SIMC_List_GetFirst(system->antennas);
	}
	SIMC_List_Destroy(system->antennas);

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