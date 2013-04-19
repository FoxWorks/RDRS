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
#include "rdrs_lookup.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Returns how much two antennas overlap by bandwidth
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_GetBandwidthIntersection(RDRS_ANTENNA* sender, RDRS_ANTENNA* receiver) {
	double s_min = sender->channel.frequency   - sender->channel.bandwidth*0.5;
	double s_max = sender->channel.frequency   + sender->channel.bandwidth*0.5;
	double r_min = receiver->channel.frequency - receiver->channel.bandwidth*0.5;
	double r_max = receiver->channel.frequency + receiver->channel.bandwidth*0.5;

	if (receiver->flags & RDRS_ANTENNA_NO_RECEIVE) return 0.0;
	if (sender->flags & RDRS_ANTENNA_NO_SEND) return 0.0;

	if (((s_max >= r_min) && (s_max <= r_max)) || //Max point in range
		((s_min >= r_min) && (s_min <= r_max)) || //Min point in range
		((s_min <= r_min) && (s_max >= r_max))) { //Entire segment covers range
		return 1.0;
	}
	return 0.0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get bit error rate for modulation
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_GetBER(int modulation, double SNR) {
	double* data;
	double SNRdb;
	double min,max,step,t;
	int i,n;
	switch (modulation) {
		case RDRS_CHANNEL_BPSK:		data = RDRS_InternalLookup_BPSK; break;
		case RDRS_CHANNEL_QPSK:		data = RDRS_InternalLookup_QAM4; break;
		case RDRS_CHANNEL_8PSK:		data = RDRS_InternalLookup_8PSK; break;
		case RDRS_CHANNEL_QAM16:	data = RDRS_InternalLookup_QAM16; break;
		case RDRS_CHANNEL_QAM64:	data = RDRS_InternalLookup_QAM64; break;
		case RDRS_CHANNEL_QAM256:	data = RDRS_InternalLookup_QAM256; break;
		default: return 0.5;
	}

	//Check if no signal available
	if (SNR == 0.0) {
		return 0.5;
	}

	//Read interpolation parameters
	min = data[0];
	max = data[1];
	n = (int)data[2];
	step = (max-min)/data[2];

	//Compute SNR in decibels
	SNRdb = 10*log10(SNR);

	//Compute indexes for interpolated segments
	i = (int)((SNRdb-min)/step);
	t = (SNRdb-(i*step+min))/step;

	//Limits for indexes
	if (i < 0) { //Too low
		i = 0;
	}
	if (i > n-2) { //Too high, probably zero
		return 0.0;
	}

	//Return
	//printf("LERP %.4f\n",t,data[3+i],data[4+i],t*data[3+i] + (1-t)*data[4+i]);
	return (1-t)*data[3+i] + t*data[4+i];
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get antenna directivity for given angles
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_GetDirectionalGain(RDRS_ANTENNA* antenna, double theta, double phi) {
	double kL2 = antenna->channel.k * antenna->size / 2.0;

	//return ((4*RDRS_PI)/(8*RDRS_PI*RDRS_PI)) * 
		//pow(cos(kL2 * cos(theta)) - cos(kL2),2) / (sin(theta)*sin(theta) + RDRS_EPS);
	/*{
		double h = 0.5;
		return (antenna->size*antenna->size / (2*antenna->channel.lambda*antenna->channel.lambda)) *
			pow(1-sin(theta)*sin(theta)*sin(phi)*sin(phi),2)*
			pow(sin(antenna->channel.k*h*cos(theta)),2);
	}*/
	/*{
		double r = antenna->size*antenna->size;
		double k = antenna->channel.k;
		double lambda = antenna->channel.lambda;
		double krsin = k*r*sin(theta);
		return (2*RDRS_PI*RDRS_PI*r*r / (lambda*lambda))*  pow((cos(theta)/krsin)*j1(krsin),2);
	}*/
	return 1.0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns arrival time of the data packet relative to receiver antenna
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_PacketArrivalTime(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet, double distance2) {
	return packet->time + (sqrt(distance2)/RDRS_LIGHTSPEED);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns power of the arriving signal (Friis equation sans gains)
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_PacketArrivalPower(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet, double distance2) {
	double lambda2 = receiver->channel.lambda*receiver->channel.lambda;
	return packet->power * lambda2 / ((4.0*RDRS_PI)*(4.0*RDRS_PI) * distance2);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns total gain between two antennas accounting for directivity (FIXME: and polarization)
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_PacketArrivalGain(RDRS_ANTENNA* receiver, RDRS_ANTENNA* sender, 
									  RDRS_DATA_PACKET* packet, double distance2) {
	double Gt,Gr;

	//Get normalized direction vector (receiver -> sender)
	double dnorm = sqrt(distance2);
	double dx = (packet->x - receiver->position.x)/dnorm; //Normalized direction
	double dy = (packet->y - receiver->position.y)/dnorm;
	double dz = (packet->z - receiver->position.z)/dnorm;

	//Convert to spherical coordinates (angles at which sender sees receiver)
	//double theta_sender = acos(-dz);
	//double phi_sender = atan2(dy,dx);
	double theta_sender = acos(-dx);
	double phi_sender = atan2(dy,dz);

	//Convert to spherical coordinates (angles at which receiver sees sender)
	//double theta_receiver = acos(dz);
	//double phi_receiver = phi_sender; //atan((-dy)/(-dx));
	double theta_receiver = acos(dx);
	double phi_receiver = phi_sender; //atan((-dy)/(-dz));

	//Calculate gains
	Gt = sender->efficiency * RDRS_Physics_GetDirectionalGain(sender,theta_sender,phi_sender);
	Gr = receiver->efficiency * RDRS_Physics_GetDirectionalGain(sender,theta_receiver,phi_receiver);
	return Gt * Gr;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns doppler factor for the received packet
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_PacketArrivalDopplerFactor(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet) {
	return 1.0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns squared distance between source and receiver
////////////////////////////////////////////////////////////////////////////////
double RDRS_Physics_PacketDistance2(RDRS_ANTENNA* receiver, RDRS_DATA_PACKET* packet) {
	double dx = packet->x - receiver->position.x;
	double dy = packet->y - receiver->position.y;
	double dz = packet->z - receiver->position.z;
	return dx*dx+dy*dy+dz*dz+RDRS_EPS;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns symbol with noise specified by BER
////////////////////////////////////////////////////////////////////////////////
RDRS_SYMBOL RDRS_Physics_GetNoisySymbol(RDRS_SYMBOL symbol, double BER) {
	int bit,threshold;

#define RAND_THRESHOLD (1.0/RAND_MAX)
	
	if (BER > 64.0*RAND_THRESHOLD) { //High error rate
		threshold = (int)(RAND_MAX * BER)+1;
		for (bit = 0; bit < 8; bit++) {
			int mask = rand() < threshold;
			symbol = symbol ^ (mask << bit);
		}
	} else if (BER > RAND_THRESHOLD*RAND_THRESHOLD) { //Low error rate
		threshold = (int)(RAND_MAX * sqrt(BER))+1;
		for (bit = 0; bit < 8; bit++) {
			int mask = (rand() < threshold) && (rand() < threshold);
			symbol = symbol ^ (mask << bit);
		}
	}
	return symbol;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Fills out bandwidth or data rate in channel information
////////////////////////////////////////////////////////////////////////////////
int RDRS_Physics_ComputeChannelParameters(RDRS_CHANNEL* channel) {
	if (channel->bandwidth == RDRS_AUTO_BANDWIDTH) {
		double symbol_rate;
		switch (channel->modulation) {
			case RDRS_CHANNEL_BPSK:		symbol_rate = channel->data_rate; break;
			case RDRS_CHANNEL_QPSK:		symbol_rate = channel->data_rate/2; break;
			case RDRS_CHANNEL_8PSK:		symbol_rate = channel->data_rate/3; break;
			case RDRS_CHANNEL_QAM16:	symbol_rate = channel->data_rate/4; break;
			case RDRS_CHANNEL_QAM64:	symbol_rate = channel->data_rate/6; break;
			case RDRS_CHANNEL_QAM256:	symbol_rate = channel->data_rate/8; break;
			default:					symbol_rate = channel->data_rate; break;
		}
		channel->bandwidth = channel->data_rate*1.25*1e-6;
	} else if (channel->data_rate == RDRS_AUTO_RATE) {
		double symbol_rate = channel->bandwidth*1e6/1.25;
		switch (channel->modulation) {
			case RDRS_CHANNEL_BPSK:		channel->data_rate = symbol_rate; break;
			case RDRS_CHANNEL_QPSK:		channel->data_rate = symbol_rate*2; break;
			case RDRS_CHANNEL_8PSK:		channel->data_rate = symbol_rate*3; break;
			case RDRS_CHANNEL_QAM16:	channel->data_rate = symbol_rate*4; break;
			case RDRS_CHANNEL_QAM64:	channel->data_rate = symbol_rate*6; break;
			case RDRS_CHANNEL_QAM256:	channel->data_rate = symbol_rate*8; break;
			default:					channel->data_rate = symbol_rate; break;
		}
	}

	//Compute additional information about the channel
	channel->lambda = RDRS_LIGHTSPEED / (1e6 * channel->frequency);
	channel->k = 2.0*RDRS_PI / channel->lambda;
	return RDRS_OK;
}