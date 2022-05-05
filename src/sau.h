/* ntnu.h */

/** System states */
enum system_states {
	/** State for setting a new GPS fence */
	SET_FENCE,
	/** Normal operating mode, sheep in fence */
	SHEEP_IN_FENCE,
	/** Higher accuracy and periodic fixes, lte connected */
	FIND_SHEEP,
	/** Sheep is dead. Idle state. LTE connects once per day. */
	SHEEP_DEADW
};

bool cloud_connected = false;