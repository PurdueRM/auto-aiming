#ifndef _MESSAGES_H
#define _MESSAGES_H

#include <stdint.h>

#define FRAME_TYPE_AUTO_AIM 0
#define FRAME_TYPE_NAV 1
#define FRAME_TYPE_HEART_BEAT 2
#define FRAME_TYPE_OTHER 3

typedef struct  _AutoAimPackage
{
	float yaw;	 // yaw (deg)
	float pitch; // pitch (deg)
	uint8_t fire;   // 0 = no fire, 1 = fire
} AutoAimPackage;

typedef struct  _NavPackage
{
	float x_vel;	// m/s 
	float y_vel;	// m/s
	float yaw_rad; // rad/s
	uint8_t state;	// 0 = stationary, 1 = moving, 2 = spin
} NavPackage;

typedef struct  _HeartBeatPackage
{
	uint8_t _a; // blank
	uint8_t _b;
	uint8_t _c;
	uint8_t _d;
} HeartBeatPackage;

typedef struct  _PackageOut
{
	uint8_t frame_id;
	uint8_t frame_type;
	union
	{
		AutoAimPackage autoAimPackage;
		NavPackage navPackage;
		HeartBeatPackage heartBeatPackage;
	};
	// uint8_t crc8;
} PackageOut;

typedef struct __attribute__((__packed__)) _PackageIn
{
	uint8_t head; 					// header byte 0xAA
	uint8_t enemy_color_is_red; 	// 1 for red and 0 for blue
	uint8_t game_status; 			// 0 for not started, 1 for preperation stage, 2 for 15 seconds referee check, 3 for 5 seconds count down, 4 for match going, 5 for calculating match result
	uint8_t rfid; 					// bit 0 for resupply, bit 1 for center zone
	float pitch;        			// rad
	float pitch_vel; 				// rad/s
	float yaw_vel;   				// rad/s
	float x;         				// m (global frame)
	float y;         				// m (global frame)
	float orientation;        		// rad (gimbal frame)
	float x_vel;         			// m/s (gimbal frame)
	float y_vel;         			// m/s (gimbal frame)
	uint16_t HP;					// Sentry HP (full health is 400)
	uint16_t reserved; 				// reserved for future use
} __attribute__((packed)) PackageIn;

#endif // _MESSAGES_H
