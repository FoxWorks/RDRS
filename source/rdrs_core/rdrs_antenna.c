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
#include <math.h>
#include "rdrs.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Create new connection
////////////////////////////////////////////////////////////////////////////////
int RDRS_Connection_Create(RDRS_CONNECTION** p_connection) {
	RDRS_CONNECTION* connection;
	if (!p_connection) return RDRS_ERROR_BAD_PARAMETER;

	//Create new connection object
	connection = (RDRS_CONNECTION*)malloc(sizeof(RDRS_CONNECTION));
	*p_connection = connection;
	if (!connection) return RDRS_ERROR_MEMORY;
	memset(connection,0,sizeof(RDRS_CONNECTION));

	//Create data queue for the data 'flying' between sending and receiving antenna
	SIMC_Queue_Create(&connection->buffer,16384,sizeof(RDRS_DATA_PACKET));
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy connection
////////////////////////////////////////////////////////////////////////////////
int RDRS_Connection_Destroy(RDRS_CONNECTION* connection) {
	//Remove connection from antenna connection lists
	if (connection->sender_entry) {
		SIMC_List_GetFirst(connection->sender->from_this);
		SIMC_List_Remove(connection->sender->from_this,connection->sender_entry);
	}
	if (connection->receiver_entry) {
		SIMC_List_GetFirst(connection->receiver->from_this);
		SIMC_List_Remove(connection->receiver->from_this,connection->receiver_entry);
	}

	//Remove buffer and connection object
	SIMC_Queue_Destroy(connection->buffer);
	free(connection);
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create new antenna
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_Create(RDRS_SYSTEM* system, RDRS_ANTENNA** p_antenna, RDRS_CHANNEL channel, int flags) {
	RDRS_ANTENNA* antenna;
	if (!p_antenna) return RDRS_ERROR_BAD_PARAMETER;

	//Create new antenna
	antenna = (RDRS_ANTENNA*)malloc(sizeof(RDRS_ANTENNA));
	*p_antenna = antenna;
	if (!antenna) return RDRS_ERROR_MEMORY;
	memset(antenna,0,sizeof(RDRS_ANTENNA));

	//Add antenna to the system
	antenna->type = RDRS_TYPE_DIPOLE;
	antenna->flags = flags;
	antenna->system = system;
	antenna->system_entry = SIMC_List_Append(system->antennas,antenna);

	//Default orientation
	antenna->orientation.q[0] = 1.0;
	antenna->orientation.q[1] = 0.0;
	antenna->orientation.q[2] = 0.0;
	antenna->orientation.q[3] = 0.0;

	//Create buffers for the virtual modem
	SIMC_Queue_Create(&antenna->send_buffer,16384,sizeof(void*));
	SIMC_Queue_Create(&antenna->recv_buffer,16384,sizeof(void*));

	//Create connection lists
	SIMC_List_Create(&antenna->from_this,1);
	SIMC_List_Create(&antenna->to_this,1);

	//Set channel and default antenna size, efficiency
	RDRS_Antenna_SetChannel(antenna,channel);
	antenna->size = 2.0*antenna->channel.lambda;
	antenna->efficiency = 1.00;//0.55;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy antenna
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_Destroy(RDRS_ANTENNA* antenna) {
	//FIXME
	return RDRS_ERROR_NOT_IMPLEMENTED;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Change the antenna's transmit/receive channel.
///
/// This function can be used to update antennas channel information. It will change
/// antennas channels frequency, bandwidth, modulation type.
///
/// @bug This function will clear incoming data queue, effectively removing data that
/// may still arrive for the antenna in the future.
///
/// @param[in] antenna Pointer to RDRS_ANTENNA
/// @param[in] channel A copy of RDRS_CHANNEL
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetChannel(RDRS_ANTENNA* antenna, RDRS_CHANNEL channel) {
	SIMC_LIST_ENTRY* entry;

	//Compute bandwidth or data rate
	RDRS_Physics_ComputeChannelParameters(&channel);
	antenna->channel = channel;

	//Remove all current connections from this antenna
	entry = SIMC_List_GetFirst(antenna->from_this);
	while (entry) {
		RDRS_CONNECTION* connection = SIMC_List_GetData(antenna->from_this,entry);
		SIMC_List_Stop(antenna->from_this,entry);

		RDRS_Connection_Destroy(connection);
		entry = SIMC_List_GetFirst(antenna->from_this);
	}

	//Remove all current connections to this antenna
	entry = SIMC_List_GetFirst(antenna->to_this);
	while (entry) {
		RDRS_CONNECTION* connection = SIMC_List_GetData(antenna->to_this,entry);
		SIMC_List_Stop(antenna->to_this,entry);

		RDRS_Connection_Destroy(connection);
		entry = SIMC_List_GetFirst(antenna->to_this);
	}

	//Create connections to antennas which may receive data from this one
	if (!(antenna->flags & RDRS_ANTENNA_NO_SEND)) {
		entry = SIMC_List_GetFirst(antenna->system->antennas);
		while (entry) {
			double intersection;
			RDRS_ANTENNA* receiver = SIMC_List_GetData(antenna->system->antennas,entry);
			intersection = RDRS_Physics_GetBandwidthIntersection(antenna,receiver);

			//Create a new connection
			if (intersection > 0.0) {
				RDRS_CONNECTION* connection;
				RDRS_Connection_Create(&connection);
				connection->sender = antenna;
				connection->receiver = receiver;
				connection->sender_entry = SIMC_List_Append(antenna->from_this,connection);
				connection->receiver_entry = SIMC_List_Append(receiver->to_this,connection);
				connection->intersection = intersection;
			}

			entry = SIMC_List_GetNext(antenna->system->antennas,entry);
		}
	}

	//Create connections in antennas which may send data to this one
	if (!(antenna->flags & RDRS_ANTENNA_NO_RECEIVE)) {
		entry = SIMC_List_GetFirst(antenna->system->antennas);
		while (entry) {
			double intersection;
			RDRS_ANTENNA* sender = SIMC_List_GetData(antenna->system->antennas,entry);
			intersection = RDRS_Physics_GetBandwidthIntersection(sender,antenna);

			//Create a new connection
			if (intersection > 0.0) {
				RDRS_CONNECTION* connection;
				RDRS_Connection_Create(&connection);
				connection->sender = sender;
				connection->receiver = antenna;
				connection->sender_entry = SIMC_List_Append(sender->from_this,connection);
				connection->receiver_entry = SIMC_List_Append(antenna->to_this,connection);
				connection->intersection = intersection;
			}

			entry = SIMC_List_GetNext(antenna->system->antennas,entry);
		}
	}
	return RDRS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Write a symbol to virtual modems outgoing buffer.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_Send(RDRS_ANTENNA* antenna, RDRS_SYMBOL value) {
	void* data;

	SIMC_Queue_EnterWrite(antenna->send_buffer,&data);
	*((RDRS_SYMBOL*)data) = value;
	SIMC_Queue_LeaveWrite(antenna->send_buffer);
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Read a symbol from virtual modems incoming buffer.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_Receive(RDRS_ANTENNA* antenna, RDRS_SYMBOL* p_value) {
	void* data;

	SIMC_Queue_EnterRead(antenna->recv_buffer,&data);
	*p_value = *((RDRS_SYMBOL*)data);
	SIMC_Queue_LeaveRead(antenna->recv_buffer);
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set antenna transmission power.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetPower(RDRS_ANTENNA* antenna, double power) {
	antenna->power = power;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set antenna type.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetType(RDRS_ANTENNA* antenna, int type) {
	antenna->type = type;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set antenna size (diameter or length).
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetSize(RDRS_ANTENNA* antenna, double size) {
	antenna->size = size;
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set radio antenna position in inertial space.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetPosition(RDRS_ANTENNA* antenna, RDRS_VECTOR* vector) {
	memcpy(&antenna->position,vector,sizeof(RDRS_VECTOR));
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set radio antenna velocity in inertial space.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetVelocity(RDRS_ANTENNA* antenna, RDRS_VECTOR* vector) {
	memcpy(&antenna->velocity,vector,sizeof(RDRS_VECTOR));
	return RDRS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set radio antenna orientation in inertial space.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SetOrientation(RDRS_ANTENNA* antenna, RDRS_QUATERNION* quaternion) {
	memcpy(&antenna->orientation,quaternion,sizeof(RDRS_QUATERNION));
	return RDRS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Simulate virtual modem sending data from the outgoing buffer.
///
/// This function causes a data burst or supports the data stream via the
/// virtual modem.
///
/// If buffer becomes empty, the antenna will cease data stream (there will be a
/// discontinuity in streamed data). In order to keep streaming there must
/// always be enough data in buffer.
///
/// Internal virtual modems buffer is 500 msec long by default.
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SimulateSend(RDRS_ANTENNA* antenna) {
	double time;
	int used_slots;
	int index = 0;

	//Get time reference. Either send all data, or send until this time is reached
	RDRS_System_GetTime(antenna->system,&time);
	if (antenna->write_time == 0.0) antenna->write_time = time;

	//Send no more than 'used_slots' symbols, or no more than internal virtual modems buffer
	SIMC_Queue_State(antenna->send_buffer,0,&used_slots); 
	while ((used_slots > 0) && (antenna->write_time < time+0.5)) {
		void* data = 0;
		RDRS_SYMBOL symbol;
		SIMC_LIST_ENTRY* entry;
		RDRS_DATA_PACKET packet_prototype;

		//Generate timing for the symbol
		double symbol_time = antenna->write_time + index * (RDRS_SYMBOL_SIZE/antenna->channel.data_rate);

		//Generate symbol
		if (!SIMC_Queue_EnterRead(antenna->send_buffer,&data)) return RDRS_ERROR_INTERNAL;
		symbol = *((RDRS_SYMBOL*)data);
		SIMC_Queue_LeaveRead(antenna->send_buffer);

		//Create packet
		packet_prototype.power = (float)(antenna->power*antenna->efficiency);
		packet_prototype.symbol = symbol;
		packet_prototype.time = symbol_time;
		packet_prototype.x = antenna->position.x;
		packet_prototype.y = antenna->position.y;
		packet_prototype.z = antenna->position.z;
		packet_prototype.vx = (float)antenna->velocity.x;
		packet_prototype.vy = (float)antenna->velocity.y;
		packet_prototype.vz = (float)antenna->velocity.z;
		packet_prototype.q1 = (float)antenna->orientation.q[1];
		packet_prototype.q2 = (float)antenna->orientation.q[2];
		packet_prototype.q3 = (float)antenna->orientation.q[3];

		//Send over every connection
		entry = SIMC_List_GetFirst(antenna->from_this);
		while (entry) {
			RDRS_DATA_PACKET* packet;
			RDRS_CONNECTION* connection = (RDRS_CONNECTION*)SIMC_List_GetData(antenna->from_this,entry);

			SIMC_Queue_EnterWrite(connection->buffer,&packet);
			memcpy(packet,&packet_prototype,sizeof(RDRS_DATA_PACKET));
			SIMC_Queue_LeaveWrite(connection->buffer);

			entry = SIMC_List_GetNext(antenna->from_this,entry);			
		}

		//Move to next symbol
		index++;
	}

	//If there is no more data in buffer, create a discontinuity
	if (used_slots == 0) {
		antenna->write_time = time;
	}
	return RDRS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Simulate virtual modem receiving data from antenna.
///
/// 
///
/// @returns Error code
/// @retval RDRS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int RDRS_Antenna_SimulateReceive(RDRS_ANTENNA* antenna) {
	SIMC_LIST_ENTRY* entry;
	double time;
	int received_data;
	int index = 0;

	//Get time reference, which is assumed to be current time
	RDRS_System_GetTime(antenna->system,&time);

	//If no known read time, start from earliest possible. Read time shows time,
	//by which virtual modems internal receive buffer is valid (so all data prior to
	//read time was already processed).
	if (antenna->read_time == 0.0) {
		antenna->read_time = time;
		entry = SIMC_List_GetFirst(antenna->to_this);
		while (entry) {
			RDRS_DATA_PACKET* packet;
			RDRS_CONNECTION* connection = (RDRS_CONNECTION*)SIMC_List_GetData(antenna->to_this,entry);
			if (SIMC_Queue_Peek(connection->buffer,&packet)) {
				double distance2 = RDRS_Physics_PacketDistance2(antenna,packet);
				double arrival_time = RDRS_Physics_PacketArrivalTime(antenna,packet,distance2);
				if (arrival_time < antenna->read_time) antenna->read_time = arrival_time;
			}
			entry = SIMC_List_GetNext(antenna->to_this,entry);			
		}
	}

	//If not possible to read anything, quit
	if (antenna->read_time == 0.0) return RDRS_OK; 

	//Receive over every connection until no more connections have data
	do {
		double power = 0; //Maximum received signal power
		double noise_power = 0; //Power of noise for this symbol
		double doppler_factor = 1; //Ratio of received frequency to antenna frequency
		//double interference_power = 0; //Extra power from interfering signals
		RDRS_SYMBOL symbol = 0; //Received symbol (possibly a result of interference between several signals)
		double symbol_start_time,symbol_end_time; //Symbol timing
		double earliest_time = time; //Time of earliest symbol in connection, used to skip forward in time
		received_data = 0;

		//Generate timing (bounds within this symbol may be received)
		//Timing indicates precise time when symbol starts and symbol ends, as relative to
		//previous received symbols
		symbol_start_time = antenna->read_time + index * (RDRS_SYMBOL_SIZE/antenna->channel.data_rate);
		symbol_end_time = antenna->read_time + (index+1) * (RDRS_SYMBOL_SIZE/antenna->channel.data_rate);

		//If next symbol to be received is in the future, do not receive it and quit loop
		if (symbol_end_time > time) break;

		//Receive a single symbol over every connection
		entry = SIMC_List_GetFirst(antenna->to_this);
		while (entry) {
			//It is possible that data rates are different between two streams, hence
			//sometimes a few symbols will fit into receiving antennas symbol)
			int peek_again;
			RDRS_DATA_PACKET* packet;
			RDRS_CONNECTION* connection = (RDRS_CONNECTION*)SIMC_List_GetData(antenna->to_this,entry);

			do {
				peek_again = 0;
				if (SIMC_Queue_Peek(connection->buffer,&packet)) {
					//Precise arrival time, accounting for travel path
					double distance2 = RDRS_Physics_PacketDistance2(antenna,packet);
					double arrival_time = RDRS_Physics_PacketArrivalTime(antenna,packet,distance2);

					//Skip packets from the past which must have been received earlier
					if (arrival_time < antenna->read_time) {
						SIMC_Queue_EnterRead(connection->buffer,0);
						continue;
					}

					//Measure earliest arrival time (if earlier arrival time is later
					//than current read time, this difference can be skipped over)
					if (arrival_time < earliest_time) earliest_time = arrival_time;

					//Check if this symbol falls inside symbol timing
					if ((arrival_time >= symbol_start_time) && (arrival_time < symbol_end_time)) {
						//Compute arrival power and doppler effect
						double arrival_power = RDRS_Physics_PacketArrivalPower(antenna,packet,distance2);
						double arrival_doppler = RDRS_Physics_PacketArrivalDopplerFactor(antenna,packet);

						//Compute directivity and antenna gains
						arrival_power *= RDRS_Physics_PacketArrivalGain(antenna,connection->sender,packet,distance2);

						//Compute how much power falls within antennas incoming bandwidth
						arrival_power *= connection->intersection;

						//FIXME: offset in central frequency must shift some power from signal to noise

						//If modulation is correct, select strongest signal, otherwise add this power to noise
						if ((arrival_power > power) && 
							(antenna->channel.modulation == connection->sender->channel.modulation)) {
							//Decode this symbol
							symbol = packet->symbol;

							//Remember strongest received signal power
							noise_power += power; //Old signal is now considered noise
							power = arrival_power;
						} else {
							//FIXME: multiple bad symbols should not stack up with power						
							noise_power += arrival_power; //Add this symbol as extra noise
						}
						
						SIMC_Queue_EnterRead(connection->buffer,0); //Push out of the buffer

						//See if there are any more symbols in stream to be matched to received symbol
						peek_again = 1;
						//At least one symbol was received from some stream
						received_data = 1;
					}
				}
			} while (peek_again);

			entry = SIMC_List_GetNext(antenna->to_this,entry);			
		}

		//If any data was received and there is carrier, add it to buffer
		if (received_data && (power > 0.0)) {
			void* data;
			double SNR,BER;

			//Add free space noise and receiver noise
			//bandwidth = 1e6 * antenna->channel.bandwidth;
			//noise_power += RDRS_FreeSpace_NoiseModel() * bandwidth;
			//noise_power += RDRS_K * 468.0 * bandwidth; //Low-gain amplifier
			//noise_power += RDRS_K * 2610.0 * bandwidth; //Down converter
			//noise_power += RDRS_K * 28710.0; //High-gain amplifier
			noise_power += RDRS_K * 600.0 * (1e6 * antenna->channel.bandwidth);

			//Calculate bit error rate
			SNR = power / noise_power;
			BER = RDRS_Physics_GetBER(antenna->channel.modulation,SNR);

			//Add symbol to data
			SIMC_Queue_EnterWrite(antenna->recv_buffer,&data);
			*((RDRS_SYMBOL*)data) = RDRS_Physics_GetNoisySymbol(symbol,BER); //Add noisy symbol
			SIMC_Queue_LeaveWrite(antenna->recv_buffer);

			printf("Recv %.5f %02X %.1f db %.1f db\n",symbol_start_time,*((RDRS_SYMBOL*)data),10*log10(power),10*log10(SNR));
		}

		//Check whether to read next symbol or skip some time
		if ((earliest_time < time) && (earliest_time > symbol_end_time)) {
			//Earliest time of next symbol is way after expected end time,
			//so reset reading and start from scratch (discontinuity)
			antenna->read_time = earliest_time;
			index = 0;

			//No data was received, but there is still potential for some
			received_data = 1;
		} else { //Just move to next symbol
			index++;
		}
	} while (received_data);

	//Finished this iteration
	return RDRS_OK;
}