/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/led_control.h>


static inline constexpr int ucdr_topic_size_led_control()
{
	return 13;
}

static inline bool ucdr_serialize_led_control(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const led_control_s& topic = *static_cast<const led_control_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.led_mask) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.led_mask, sizeof(topic.led_mask));
	buf.iterator += sizeof(topic.led_mask);
	buf.offset += sizeof(topic.led_mask);
	static_assert(sizeof(topic.color) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.color, sizeof(topic.color));
	buf.iterator += sizeof(topic.color);
	buf.offset += sizeof(topic.color);
	static_assert(sizeof(topic.mode) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.mode, sizeof(topic.mode));
	buf.iterator += sizeof(topic.mode);
	buf.offset += sizeof(topic.mode);
	static_assert(sizeof(topic.num_blinks) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.num_blinks, sizeof(topic.num_blinks));
	buf.iterator += sizeof(topic.num_blinks);
	buf.offset += sizeof(topic.num_blinks);
	static_assert(sizeof(topic.priority) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.priority, sizeof(topic.priority));
	buf.iterator += sizeof(topic.priority);
	buf.offset += sizeof(topic.priority);
	return true;
}

static inline bool ucdr_deserialize_led_control(ucdrBuffer& buf, led_control_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.led_mask) == 1, "size mismatch");
	memcpy(&topic.led_mask, buf.iterator, sizeof(topic.led_mask));
	buf.iterator += sizeof(topic.led_mask);
	buf.offset += sizeof(topic.led_mask);
	static_assert(sizeof(topic.color) == 1, "size mismatch");
	memcpy(&topic.color, buf.iterator, sizeof(topic.color));
	buf.iterator += sizeof(topic.color);
	buf.offset += sizeof(topic.color);
	static_assert(sizeof(topic.mode) == 1, "size mismatch");
	memcpy(&topic.mode, buf.iterator, sizeof(topic.mode));
	buf.iterator += sizeof(topic.mode);
	buf.offset += sizeof(topic.mode);
	static_assert(sizeof(topic.num_blinks) == 1, "size mismatch");
	memcpy(&topic.num_blinks, buf.iterator, sizeof(topic.num_blinks));
	buf.iterator += sizeof(topic.num_blinks);
	buf.offset += sizeof(topic.num_blinks);
	static_assert(sizeof(topic.priority) == 1, "size mismatch");
	memcpy(&topic.priority, buf.iterator, sizeof(topic.priority));
	buf.iterator += sizeof(topic.priority);
	buf.offset += sizeof(topic.priority);
	return true;
}
