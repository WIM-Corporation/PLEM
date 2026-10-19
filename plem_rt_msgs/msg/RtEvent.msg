# RtEvent.msg - discrete RT event (an RT host's `~/rt_events`)
#
# One event of the RT loop: a deadline miss, a communication error, a state
# transition or a fault. Published latched (transient_local): a subscriber
# that attaches later still receives the retained events.
#
# header.stamp is the publish time. The event's time is monotonic_ns.

std_msgs/Header header

# Timing and sequence information
uint64 monotonic_ns                 # Event time (CLOCK_MONOTONIC)
uint64 event_sequence               # Event's own sequence number
uint64 ref_sample_sequence          # Related RT sample sequence (0 = N/A)

# Event classification
uint8 event_type                    # Event type (see constants below)
uint8 source_id                     # Source identifier (see SOURCE_* below)
uint8 severity                      # Severity level (see constants below)
uint8 joint_id                      # Axis index (0-7, below kMaxBusAxes), or JOINT_ID_NOT_APPLICABLE

# Event details
uint16 error_code                   # Per event_type (see the constants below): a statusword, an AL state,
                                    # a wc_state, a LIMIT_STOP_REASON_*, 1 for a refused protective-stop
                                    # clear, else 0. Not the drive's error code (SERVO_FAULT carries it in value)
float32 value                       # Meaning per event_type (see the constants below)

# event_type values. Groups: safety 0x40-0x4F, error 0x50-0x6F. A number,
# once assigned, is never reused. Unlisted numbers are free unless reserved:
# never reused (0x20, 0x21, 0x30, 0x44, 0x48, 0x49, 0x51, 0x5A, 0x5B, 0x5D, 0x5E, 0xF0)
# or left to events a higher layer defines (0x45-0x47, 0x4A-0x4D, 0x55-0x57).
uint8 PROTECTIVE_STOP=64                 # 0x40 protective stop triggered (error_code 0) or a clear refused (error_code 1)
uint8 PROTECTIVE_STOP_CLEAR=65           # 0x41 protective stop cleared
uint8 BRAKE_ENGAGE=66                    # 0x42 brake engaged
uint8 BRAKE_RELEASE=67                   # 0x43 brake released
uint8 LIMIT_STOP=78                      # 0x4E an axis or its command crossed a limit; error_code = LIMIT_STOP_REASON_*, value in its unit
uint8 RAMP_STOP_TIMEOUT=79               # 0x4F a ramp stop not confirmed in time: Quick stop; value = s since the request
uint8 SERVO_FAULT=80                     # 0x50 drive fault; error_code = statusword, value = the drive's error code read
                                         # through the configured error-code PDO role (normally CiA 402 0x603F, at the
                                         # axis's channel address); 0 when no error-code PDO is mapped
uint8 WKC_MISMATCH=82                    # 0x52 working counter mismatch; error_code = wc_state, value = consecutive count
uint8 DEADLINE_MISS=83                   # 0x53 RT loop deadline exceeded; value = loop exec time in us
uint8 COMMAND_STALE=84                   # 0x54 command stream stale; value = age in s
uint8 SERVO_FAULT_RESET=88               # 0x58 automatic fault-reset request accepted; error_code = statusword
uint8 SERVO_FAULT_RESET_EXHAUSTED=89     # 0x59 automatic fault-reset allowance spent; error_code = statusword
uint8 BRAKE_SEQUENCE_STALLED=92          # 0x5C brake sequence made no progress; error_code = statusword
uint8 MODES_OF_OPERATION_MISMATCH=95     # 0x5F axis not in its declared mode; error_code = statusword, value = 0x6061
uint8 SERVO_FAULT_RESET_RESULT=96        # 0x60 what a fault-reset request did; error_code = statusword
uint8 OPERATION_ENABLED_LOST=97          # 0x61 a released axis left Operation enabled; error_code = statusword
uint8 SLAVE_NOT_OPERATIONAL=98           # 0x62 release refused, slave not in OP; error_code = AL state, value = ring position

# error_code of LIMIT_STOP: why the axis was stopped (value's unit in brackets)
uint16 LIMIT_STOP_REASON_NONE=0
uint16 LIMIT_STOP_REASON_POSITION=1      # measured position outside the limits [rad]
uint16 LIMIT_STOP_REASON_VELOCITY=2      # measured |velocity| over the limit [rad/s]
uint16 LIMIT_STOP_REASON_UNDECODABLE=3   # position word does not decode [NaN]
uint16 LIMIT_STOP_REASON_CSP_RANGE=4     # csp target outside the limits [rad]
uint16 LIMIT_STOP_REASON_CSP_STEP=5      # csp target over one cycle's travel [rad]
uint16 LIMIT_STOP_REASON_CSV_VELOCITY=6  # csv |target| over the limit [rad/s]
uint16 LIMIT_STOP_REASON_UNENCODABLE=7   # command does not encode into drive counts [command unit]

# severity values
uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_ERROR=2
uint8 SEVERITY_CRITICAL=3

# source_id values: SOURCE_RT_LOOP is the only one; 2 and 3 are reserved, never reused
uint8 SOURCE_RT_LOOP=1

# joint_id when the event is not joint-specific
uint8 JOINT_ID_NOT_APPLICABLE=255
