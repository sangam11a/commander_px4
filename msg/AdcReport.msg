uint64 timestamp		# time since system start (microseconds)
uint32 device_id		# unique device ID for the sensor that does not change between power cycles
int16[12] channel_id		# ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
int32[12] raw_data		# ADC channel raw value, accept negative value, valid if channel ID is positive
uint32 resolution		# ADC channel resolution
float32 v_ref			# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)

# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)

float32 v_ref_ext		# External ADC channel voltage reference

int16[12] channel_id_ext1	# External ADC1 Channel ID
int16[12] channel_id_ext2	# External ADC2 Channel ID

float32[12] raw_data_ext1	# External ADC1 raw data (need float data type; actual ADC data after basic formula)
float32[12] raw_data_ext2	# External ADC2 raw data (need float data type; actual ADC data after basic formula)

# TOPICS adc_report
# TOPICS external_adc
