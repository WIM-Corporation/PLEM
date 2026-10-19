# DriveInfo.msg - CiA 402 drive-layer static facts (plem_drive `~/info`)
#
# Published once after the node built its bus, latched (transient_local): a
# subscriber that attaches later still receives it. Nothing here changes
# while the node runs; the dynamic level report is DriveStatus.
#
# header.stamp is the publish time.

std_msgs/Header header

string[] pdo_mapping_digest # per slave, the model file's slaves[] order (one entry per slave):
                            # a `pdo_mapping: scanned` slave carries "sha256:<64 hex>" of the
                            # mapping this boot scanned, the value an approval record's
                            # pdo_mapping_digest names; a declared slave carries ""
int8[] modes_of_operation   # per axis, the model file's axes order: the declared 0x6060 Modes of
                            # operation (INTEGER8): 10 CST, 8 CSP, 9 CSV. The unit of that
                            # axis's command: Nm, rad, rad/s respectively
