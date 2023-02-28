// ######################################
// #			START OF GPS			#
// ######################################



/**
 * Message GPS_SPEED id.
 */
#define ID_GPS_SPEED                            0xA0001


/**
 * Get signal Speed_mph from buffer.
 *
 * @param buf
 *	The can message buffer containing the signal
 * @return 
 *	The raw signal
 */
#define GET_GPS_SPEED_SPEED_MPH(buf) (0 \
	| (uword)(+(uword)((buf[0] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[1] >> 0) & 0xff) << 0) \
)


/**
 * Get signal Altitude from buffer.
 */
#define GET_GPS_SPEED_ALTITUDE(buf) (0 \
	| (uword)(+(uword)((buf[2] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[3] >> 0) & 0xff) << 0) \
)

/**
 * Get signal True_course from buffer.
 */
#define GET_GPS_SPEED_TRUE_COURSE(buf) (0 \
	| (uword)(+(uword)((buf[4] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[5] >> 0) & 0xff) << 0) \
)

/**
 * Get signal Satelites from buffer.
 */
#define GET_GPS_SPEED_SATELITES(buf) (0 \
	| (ubyte)(+(ubyte)((buf[6] >> 0) & 0xff) << 0) \
)

/**
 * Get signal Valid from buffer.
 */
#define GET_GPS_SPEED_VALID(buf) (0 \
	| (ubyte)(+(ubyte)((buf[7] >> 0) & 0xff) << 0) \
)

#define CALC_GPS_SPEED_SPEED_MPH(x, fmt) \
	((x) * fmt / 100)
	
#define CALC_GPS_SPEED_SPEED_KPH(x, fmt) \
	((x) * fmt * 1.609344 / 100)
	
#define CALC_GPS_SPEED_ALTITUDE(x, fmt) \
	((x) * fmt * 0.3048 / 1)
	
#define CALC_GPS_SPEED_TRUE_COURSE(x, fmt) \
	((x) * fmt / 100)
	
#define CALC_GPS_SPEED_SATELITES(x, fmt) \
	((x) * fmt / 1)
	
#define CALC_GPS_SPEED_VALID(x, fmt) \
	((x) * fmt / 1)



/**
 * Message GPS_YAW id.
 */
#define ID_GPS_YAW                            	0xA0004

/**
 * Get signal X_yaw from buffer.
 *
 * @param buf
 *	The can message buffer containing the signal
 * @return
 *	The raw signal
 */
#define GET_GPS_YAW_X_YAW(buf) (0 \
	| (uword)(+(uword)((buf[0] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[1] >> 0) & 0xff) << 0) \
)

/**
 * Get signal Y_yaw from buffer.
 */
#define GET_GPS_YAW_Y_YAW(buf) (0 \
	| (uword)(+(uword)((buf[2] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[3] >> 0) & 0xff) << 0) \
)

/**
 * Get signal Z_yaw from buffer.
 */
#define GET_GPS_YAW_Z_YAW(buf) (0 \
	| (uword)(+(uword)((buf[4] >> 0) & 0xff) << 8) \
	| (ubyte)(+(ubyte)((buf[5] >> 0) & 0xff) << 0) \
)

#define CALC_GPS_YAW_X_YAW(x, fmt) \
	((x) * fmt * 0.015258789)
	
#define CALC_GPS_YAW_Y_YAW(x, fmt) \
	((x) * fmt * 0.015258789)
	
#define CALC_GPS_YAW_Z_YAW(x, fmt) \
	((x) * fmt * 0.015258789)



// ##################################
// #			END OF GPS			#
// ##################################
