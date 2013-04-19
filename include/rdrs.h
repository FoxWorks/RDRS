////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief Realtime Digital Radio Simulator
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
#ifndef RDRS_H
#define RDRS_H
#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
// Library management
////////////////////////////////////////////////////////////////////////////////
#define RDRS_VERSION_STRING		"1a"
#define RDRS_VERSION			101
#ifndef RDRS_DYNAMIC
#	define RDRS_API
#else
#	ifdef RDRS_LIBRARY
#		define RDRS_API __declspec(dllexport)
#	else
#		define RDRS_API __declspec(dllimport)
#	endif
#endif

#include "stddef.h"
#include "sim_core.h"




////////////////////////////////////////////////////////////////////////////////
/// @defgroup RDRS_BASIC Basic definitions
/// @brief List of basic types, macros, callbacks used in RDRS
///
/// @{
////////////////////////////////////////////////////////////////////////////////

/// Single symbol
typedef unsigned char RDRS_SYMBOL;
/// Symbol size in bits
#define RDRS_SYMBOL_SIZE 8

/// Smallest meaningful number represented by RDRS_REAL
#define RDRS_EPS 1e-15
/// Smallest meaningful number represented by a single-precision float
#define RDRS_EPSf 1e-6f

/// Speed of light
#define RDRS_LIGHTSPEED 299792458.0
/// Boltzmann constant
#define RDRS_K			1.39e-23

/// PI constant (used internally)
#define RDRS_PI 3.14159265358979323846264338327950288419716939937510
/// PI constant (floating point, used internally)
#define RDRS_PIf 3.14159265358979323846f
/// Convert from degrees to radians
#define RDRS_RAD(x) (RDRS_PI*(x)/180.0)
/// Convert from radians to degrees
#define RDRS_DEG(x) (180.0*(x)/RDRS_PI)

/// Check function for errors
#define RDRS_ERRCHECK(expr) error_code = expr; if (error_code != RDRS_OK) return error_code;

/// Basic full-precision vector type
typedef struct RDRS_VECTOR_TAG {
	double x;			///< X component of the vector
	double y;			///< Y component of the vector
	double z;			///< Z component of the vector
} RDRS_VECTOR;

/// Basic full-precision quaternion type
typedef struct RDRS_QUATERNION_TAG {
	double q[4];		///< Real and imaginary parts of the quaternion
} RDRS_QUATERNION;

/// Channel description
typedef struct RDRS_CHANNEL_TAG {
	int modulation;		///< Modulation type
	double frequency;	///< Frequency in MHz
	double bandwidth;	///< Bandwidth in MHz
	double data_rate;	///< In bits per second

	double lambda;		///< Wavelength in m
	double k;			///< Wavenumber in m^-1
} RDRS_CHANNEL;

/// Extended structure including symbol information
typedef struct RDRS_SYMBOL_INFO_TAG {
	RDRS_SYMBOL data;		///< Bits of the symbol
	float power;			///< Received power (watts)
	double time;			///< Time of arrival (MJD)
	float doppler_factor;	///< Doppler shift of sender relative to source
	float recv_frequency;	///< True received frequency due to doppler shift
} RDRS_SYMBOL_INFO;

/// Automatic bandwidth
#define RDRS_AUTO_BANDWIDTH		0.0
/// Automatic data rate
#define RDRS_AUTO_RATE			0.0

// Phase-shift keyed modulations
#define RDRS_CHANNEL_BPSK		0
#define RDRS_CHANNEL_QPSK		1
#define RDRS_CHANNEL_8PSK		2

// Quadrature amplitude modulations
#define RDRS_CHANNEL_QAM4		1
#define RDRS_CHANNEL_QAM16		3
#define RDRS_CHANNEL_QAM64		4
#define RDRS_CHANNEL_QAM256		5

/// Omni/dipole antenna
#define RDRS_TYPE_DIPOLE					0
/// Horizontal dipole above surface
#define RDRS_TYPE_GROUND_HORIZONTAL_DIPOLE	1
/// Vertical dipole above surface
#define RDRS_TYPE_GROUND_VERTICAL_DIPOLE	2
/// Ideal isentropic antenna
#define RDRS_TYPE_IDEAL_ISENTROPIC			3
/// Parabolic antenna
#define RDRS_TYPE_PARABOLIC					4


/// Completed successfully
#define RDRS_OK								0
/// Internal error (used for errors in internal APIs)
#define RDRS_ERROR_INTERNAL					1
/// Error in memory allocation
#define RDRS_ERROR_MEMORY					4
/// Parameter invalid or out of range
#define RDRS_ERROR_BAD_PARAMETER			5
/// API call for given set of parameters is not implemented
#define RDRS_ERROR_NOT_IMPLEMENTED			11

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////


// Forward structure declarations
typedef struct RDRS_SYSTEM_TAG RDRS_SYSTEM;
typedef struct RDRS_ANTENNA_TAG RDRS_ANTENNA;
typedef struct RDRS_DATA_PACKET_TAG RDRS_DATA_PACKET;
typedef struct RDRS_CONNECTION_TAG RDRS_CONNECTION;

// Include data structures themselves
#ifdef RDRS_LIBRARY
#	include "rdrs_internal.h"
#endif


////////////////////////////////////////////////////////////////////////////////
/// @defgroup RDRS_SYSTEM System API
/// @brief
///
/// @{
////////////////////////////////////////////////////////////////////////////////

// Get library version
RDRS_API int RDRS_Version(int* version, char* version_string);

// Create RDRS system
RDRS_API int RDRS_System_Create(RDRS_SYSTEM** p_system);
// Destroy RDRS system and all resources
RDRS_API int RDRS_System_Destroy(RDRS_SYSTEM* system);

// Start simulation
RDRS_API int RDRS_System_Start(RDRS_SYSTEM* system);
// Stop/pause simulation
RDRS_API int RDRS_System_Stop(RDRS_SYSTEM* system);

// Set system MJD time (system asynchronously simulates to this set time)
RDRS_API int RDRS_System_SetTime(RDRS_SYSTEM* system, double time);
// Set system to be realtime (synchronized with system clock)
RDRS_API int RDRS_System_SetRealTime(RDRS_SYSTEM* system, int is_realtime);
// Get MJD time from system
RDRS_API int RDRS_System_GetTime(RDRS_SYSTEM* system, double* time);

// Set userdata
RDRS_API int RDRS_System_SetUserdata(RDRS_SYSTEM* system, void* userdata);
// Get userdata
RDRS_API int RDRS_System_GetUserdata(RDRS_SYSTEM* system, void** p_userdata);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @defgroup RDRS_ANTENNA Radio antenna API
/// @brief Create and manage radio antennas, send data over them, specify geometry parameters
///
/// @{
////////////////////////////////////////////////////////////////////////////////

/// Antenna cannot receive data
#define RDRS_ANTENNA_NO_RECEIVE			1
/// Antenna cannot send data
#define RDRS_ANTENNA_NO_SEND			2
/// Antenna stores extended packet information (receive timing, frequency)
#define RDRS_ANTENNA_EXTENDED_MODEM		4

// Create new radio antenna
RDRS_API int RDRS_Antenna_Create(RDRS_SYSTEM* system, RDRS_ANTENNA** p_antenna, RDRS_CHANNEL channel, int flags);
// Destroy radio antenna
RDRS_API int RDRS_Antenna_Destroy(RDRS_ANTENNA* antenna);

// Update antenna channel (expensive operation)
RDRS_API int RDRS_Antenna_SetChannel(RDRS_ANTENNA* antenna, RDRS_CHANNEL channel);

// Set antenna type
RDRS_API int RDRS_Antenna_SetType(RDRS_ANTENNA* antenna, int type);
// Set diameter for a parabolic antenna or length for a dipole antenna
RDRS_API int RDRS_Antenna_SetSize(RDRS_ANTENNA* antenna, double size);
// Set antenna transmission power (in watts)
RDRS_API int RDRS_Antenna_SetPower(RDRS_ANTENNA* antenna, double power);
// Set antenna frequency
RDRS_API int RDRS_Antenna_SetChannel(RDRS_ANTENNA* antenna, RDRS_CHANNEL channel);
// Set radio antenna position
RDRS_API int RDRS_Antenna_SetPosition(RDRS_ANTENNA* antenna, RDRS_VECTOR* vector);
// Set radio antenna velocity
RDRS_API int RDRS_Antenna_SetVelocity(RDRS_ANTENNA* antenna, RDRS_VECTOR* vector);
// Set radio antenna orientation
RDRS_API int RDRS_Antenna_SetOrientation(RDRS_ANTENNA* antenna, RDRS_QUATERNION* quaternion);
// Set antenna parent (physical parameters are read from parent)
RDRS_API int RDRS_Antenna_SetParent(RDRS_ANTENNA* antenna, RDRS_ANTENNA* parent);

// Send a byte
RDRS_API int RDRS_Antenna_Send(RDRS_ANTENNA* antenna, RDRS_SYMBOL value);
// Receive a byte
RDRS_API int RDRS_Antenna_Receive(RDRS_ANTENNA* antenna, RDRS_SYMBOL* p_value);

// Simulate antenna sending data over all connections
RDRS_API int RDRS_Antenna_SimulateSend(RDRS_ANTENNA* antenna);
// Simulate antenna receiving data over all connections
RDRS_API int RDRS_Antenna_SimulateReceive(RDRS_ANTENNA* antenna);

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @defgroup RDRS_PHYSICS Implementation of radiophysics
/// @brief Various functions to compute physical parameters of radio link
///
/// @{
////////////////////////////////////////////////////////////////////////////////

// Returns how much two antennas overlap by bandwidth
RDRS_API double RDRS_Physics_GetBandwidthIntersection(RDRS_ANTENNA* sender, RDRS_ANTENNA* receiver);
// Get bit error rate for modulation
RDRS_API double RDRS_Physics_GetBER(int modulation, double SNR);
// Get antenna directivity for given angles
RDRS_API double RDRS_Physics_GetDirectionalGain(RDRS_ANTENNA* antenna, double theta, double phi);
// Returns arrival time of the data packet relative to receiver antenna
RDRS_API double RDRS_Physics_PacketArrivalTime(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet, double distance2);
// Returns power of arriving signal (sans gains)
RDRS_API double RDRS_Physics_PacketArrivalPower(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet, double distance2);
// Returns total gain between two antennas accounting for directivity
RDRS_API double RDRS_Physics_PacketArrivalGain(RDRS_ANTENNA* receiver, RDRS_ANTENNA* sender, 
											   RDRS_DATA_PACKET* packet, double distance2);
// Returns doppler factor for the received packet
RDRS_API double RDRS_Physics_PacketArrivalDopplerFactor(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet);
// Returns squared distance between source and receive
RDRS_API double RDRS_Physics_PacketDistance2(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet);
// Returns symbol with noise specified by BER
RDRS_API RDRS_SYMBOL RDRS_Physics_GetNoisySymbol(RDRS_SYMBOL symbol, double BER);
// Fills out bandwidth or data rate in channel information
RDRS_API int RDRS_Physics_ComputeChannelParameters(RDRS_CHANNEL* channel);

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
}
#endif
#endif
