# RtRawBatch.msg - K-sample batch of full RT cycle data, undecimated at the source
#
# Type of the rt_raw topic (under the publishing node's namespace, /<bus.name>
# for plem drive): every RT cycle is captured and batched (e.g. 25 msg/s at
# 1 kHz / K=40; message rate scales with the bus cycle time, bus.cycle_time_ns
# in a drive model file). Delivery is BestEffort: undecimated production, not a
# guaranteed-lossless transport; per-sample `sequence` makes any loss
# (producer SPSC overflow or DDS drop) measurable instead of silent.
# Batches split only on a strict monotonic clock regression or a non-increasing
# sequence (duplicate = corrupt input, restart = a new emission session); a
# forward sequence gap or a timestamp gap stays inside the batch: that is the
# observed bus jitter/loss, not a batching artifact.

std_msgs/Header header              # ROS epoch stamp paired with anchor_monotonic_ns:
                                    # the batcher's publish time, or on rt_raw_relay's
                                    # one-sample output the sample's RT time
uint64 anchor_monotonic_ns          # CLOCK_MONOTONIC of the same instant as header.stamp
                                    # (batcher: read adjacent at flush, µs-scale skew;
                                    # relay: the sample's monotonic_ns), the epoch<->mono
                                    # correlation anchor for bridging header-stamped
                                    # topics onto the mono time axis
uint32 period_ns                    # nominal cycle period = configured RT period
                                    # (e.g. 1000000 @1 kHz, 200000 @5 kHz)
uint16 batch_size                   # samples in this message (<= 40; partial on
                                    # split/idle/max-age flush)
uint8 axis_count                    # joints (axes) per sample, 1..8, the producer's own
                                    # count (a 2-axis drive bus: 2).
                                    # Every joint array below holds batch_size x axis_count
                                    # values

# Per-sample arrays [batch_size], sample-major. Every array in this message is
# unbounded on the wire; its length follows batch_size (<= 40) and axis_count
# (<= 8), and rt_raw_relay refuses a batch whose arrays disagree with them. Not
# `[<=N]`: PlotJuggler's ROS 2 plugin reads a bounded sequence as a fixed array
# of N and fails to decode it.
uint64[] monotonic_ns               # per-sample RT wake time [ns] (CLOCK_MONOTONIC, absolute)
uint64[] sequence                   # per-sample RT sequence (starts at 0, +1 per sample; gap = loss)
float32[] loop_exec_us              # processing time of one RT loop cycle [us]
float32[] loop_jitter_us            # period deviation [us]: actual_period - target_period
uint8[] deadline_miss               # 1: exec > period

# Per-sample joint arrays, flat batch_size x axis_count, sample-major
# (sample i, joint j = index i*axis_count+j; axis_count values per sample)
float32[] actual_position           # actual joint position [rad] (TxPDO)
float32[] actual_velocity           # actual joint velocity [rad/s] (TxPDO)
float32[] actual_torque             # actual joint torque [Nm]: the drive's torque word (0x6077) converted with the model's torque factor
                                    # (drive layer: the file's factors.torque)
float32[] command_position          # commanded joint position [rad]. CSP axis: the host command as received; other axes: the motion
                                    # target (robot tier), 0 on the drive layer
float32[] command_velocity          # commanded joint velocity [rad/s]. CSV axis: the host command; other axes: the motion target's
                                    # velocity (robot tier; 0 when upstream sends none), 0 on the drive layer
float32[] command_torque            # commanded joint torque [Nm], CST axes only (NaN on CSP·CSV axes), before the servo clamp
float32[] applied_command           # the target word actually carried in that cycle's frame, converted back to the axis's declared mode unit
                                    # (CST Nm · CSP rad · CSV rad/s; on the drive layer the declared mode is ~/info modes_of_operation, not modes_of_operation_display).
                                    # Includes clamping, rounding, flip and saturation. NaN when a CSP word does not resolve to joint coordinates
uint16[] statusword                 # CiA 402 0x6041 Statusword
int8[] modes_of_operation_display   # CiA 402 0x6061 Modes of operation display: the mode the drive reports
                                    # (8=CSP, 9=CSV, 10=CST; not the 0x6060 command)

# Per-sample fieldbus status
uint16[] wkc                        # Working Counter, live every cycle
uint8[] wkc_mismatch                # WKC mismatch (1: abnormal)
uint32[] dc_sync_monitor_ns         # DC sync monitor deviation [ns] (IgH ecrt_master_sync_monitor_process, raw value).
                                    # 0 = DC off or a true 0; this message does not tell them apart. The bus configuration is authoritative:
                                    # for plem drive, the model file's bus.sync_mode (always 0 when drive_default); for another publisher, whether its bus enables DC.
                                    # UINT32_MAX = no monitor value that cycle (IgH error sentinel); skip it when aggregating
