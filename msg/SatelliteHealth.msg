uint64 timestamp		# time since system start (microseconds)

int16 temp_x			# +X Panel (all temperature multiplied by 100)
int16 temp_x1			# -X Panel
int16 temp_y			# +Y Panel
int16 temp_y1			# -Y Panel
int16 temp_z			# +Z Panel
int16 temp_z1			# -Z Panel

int16 temp_bpb			# Backplane Board temperature
int16 temp_obc			# OBC temperature
int16 temp_com			# Communication board temperature
int16 temp_batt			# Temperature of battery

float32 accl_x			# Accelerometer x axis
float32 accl_y			# Accelerometer y axis
float32 accl_z			# Accelerometer z axis

float32 gyro_x			# Gyroscope x axis
float32 gyro_y			# Gyroscope y axis
float32 gyro_z			# Gyroscope z axis

int16 mag_x			# Magnetometer x axis
int16 mag_y			# Magnetometer y axis
int16 mag_z			# Magnetometer z axis

uint16 batt_volt		# Current battery voltage in mV
int16 sol_p1_v			# Solar panel 1 voltage
int16 sol_p2_v			# Solar panel 2 voltage
int16 sol_p3_v			# Solar panel 3 voltage
int16 sol_p4_v			# Solar panel 4 voltage
int16 sol_p5_v			# Solar panel 5 voltage
int16 sol_t_v			# Solar panel total voltage
int16 raw_v			# raw voltage

int16 sol_p1_c			# Solar panel 1 Current
int16 sol_p2_c			# Solar panel 2 Current
int16 sol_p3_c			# Solar panel 3 Current
int16 sol_p4_c			# Solar panel 4 Current
int16 sol_p5_c			# Solar panel 5 Current
int16 sol_t_c			# Solar panel total Current
int16 rst_3v3_c			# Reset 3v3 Current
int16 raw_c			# Raw Current
uint16 v3_1_c			# 3v3 1 Current
uint16 v3_2_c			# 3v3 2 Current
uint16 v5_c			# 5V Current
uint16 unreg1_c			# Unreg line 1 Current
uint16 unreg2_c			# Unreg line 2 Current

uint8 rsv_cmd			# No of reservation commands stored
int16 batt_c			# Current drawn from Battery

uint8 ant_dep_stat		# antenna deployment status
uint8 ul_state			# command up-link status flag (set after first uplink)
uint8 oper_mode			# power mode of satellite
uint8 msn_flag			# flag to indicate whether any mission is being executed
uint8 rsv_flag			# reservation command flag (1 indicating that there is reservation command)
uint8 kill_switch		# indicates whether kill switch is turned on or off (extremely critical)







