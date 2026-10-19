# DriveStatus.msg - CiA 402 drive-layer level report (plem_drive)
#
# Level confirmation, not a timing measurement: published at a constant low
# rate, so a transition's time belongs to RtEvent (RT monotonic stamps), never
# to this message.
#
# header.stamp is the publish time. It ages a retained sample (this topic is
# transient_local, so a late subscriber receives a snapshot that may predate
# it). It is not the time of any transition the fields report.

std_msgs/Header header

uint8 protective_stop           # PROTECTIVE_STOP_SOURCE_* bits (0 = none)
bool operation_enabled          # every drive of this bus reports Operation enabled in its declared mode
                                # (0x6061 = DriveInfo.modes_of_operation)
uint8 brake_engaged             # bit i = axis i engaged
uint8 brake_operation_pending   # bit i = axis i release/engage unsettled; while set,
                                # brake_engaged bit i is the last confirmed value
uint16[] statusword             # CiA 402 0x6041 Statusword per axis, axes[] order

# protective_stop bits: who asserted the stop. Every set bit must clear before motion resumes.
uint8 PROTECTIVE_STOP_SOURCE_INTERNAL=1   # the control loop's own limit checks; cleared by reset_error
uint8 PROTECTIVE_STOP_SOURCE_EXTERNAL=2   # fieldbus health (working counter loss, a stale command, a drive losing
                                          # Operation enabled or leaving its declared mode); cleared on reset, set again while the cause persists
