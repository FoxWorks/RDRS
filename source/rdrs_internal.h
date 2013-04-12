////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief Realtime Digital Radio Simulator (internal structures)
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
#ifndef RDRS_INTERNAL_H
#define RDRS_INTERNAL_H
#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
/// @ingroup RDRS_ANTENNA
/// @struct RDRS_ANTENNA
/// @brief Represents a single world, in which antennas may send signals to eachother.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct RDRS_ANTENNA_TAG {
	//Antenna state
	RDRS_VECTOR velocity;			//Current velocity in space
	RDRS_VECTOR position;			//Current position in space
	RDRS_QUATERNION orientation;	//Current orientation in space
	RDRS_CHANNEL channel;			//Channel information
	int flags;						//Antenna flags

	//Physical parameters
	int type;						//Antenna type
	double power;					//Transmitter power (watts)
	double efficiency;				//Transmission efficiency
	double size;					//Principial size of an antenna (diameter or length)

	//Virtual modem settings
	double write_time;				//Virtual modem writing clock
	double read_time;				//Virtual modem reading clock

	//Buffers and connection lists
	int is_destroyed;				//Antenna is destroyed (will be cleaned up later)
	SIMC_QUEUE*	send_buffer;		//Data from this buffer will be sent
	SIMC_QUEUE*	recv_buffer;		//Data will be received into this buffer
	SIMC_LIST_ENTRY* system_entry;	//Entry in system list of antennas

	SIMC_LIST* from_this;			//List of connections (over which this antenna sends data)
	SIMC_LIST* to_this;				//List of connections (over which this antenna receives data)

	RDRS_SYSTEM* system;			//Pointer to RDRS system
	void* userdata;					//Pointer to user data
};
#endif



////////////////////////////////////////////////////////////////////////////////
/// @ingroup RDRS_CONNECTION
/// @struct RDRS_CONNECTION
/// @brief Represents a single connection between sender and receiver
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct RDRS_DATA_PACKET_TAG {
	RDRS_SYMBOL symbol;		//Bits of the symbol

	float power;			//Transmission power (watts)
	double time;			//Time when this symbol was emitted
	float q1,q2,q3;			//Quaternion of attitude in which this symbol was emitted
	double x,y,z;			//Coordinates of transmission
	float vx,vy,vz;			//Speed at which source was moving
};

struct RDRS_CONNECTION_TAG {
	double				intersection;	//Factor of intersection between two antennas

	SIMC_QUEUE*			buffer;			//Data in mid-flight
	RDRS_ANTENNA*		sender;			//Sending antenna
	RDRS_ANTENNA*		receiver;		//Receiving antenna
	SIMC_LIST_ENTRY*	sender_entry;	//Entry in senders "from_this"
	SIMC_LIST_ENTRY*	receiver_entry;	//Entry in senders "to_this"
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup RDRS_SYSTEM
/// @struct RDRS_SYSTEM
/// @brief Represents a single world, in which antennas may send signals to eachother.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct RDRS_SYSTEM_TAG {
	double time;				//Current simulation time (MJD)
	int is_realtime;			//Should everything run realtime

	SIMC_LIST* antennas;		//List of all antennas
	void* userdata;				//User-defined data
};
#endif
	

#ifdef __cplusplus
}
#endif
#endif