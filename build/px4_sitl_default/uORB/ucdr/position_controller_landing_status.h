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
#include <uORB/topics/position_controller_landing_status.h>


static inline constexpr int ucdr_topic_size_position_controller_landing_status()
{
	return 14;
}

static inline bool ucdr_serialize_position_controller_landing_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const position_controller_landing_status_s& topic = *static_cast<const position_controller_landing_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.lateral_touchdown_offset) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.lateral_touchdown_offset, sizeof(topic.lateral_touchdown_offset));
	buf.iterator += sizeof(topic.lateral_touchdown_offset);
	buf.offset += sizeof(topic.lateral_touchdown_offset);
	static_assert(sizeof(topic.flaring) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.flaring, sizeof(topic.flaring));
	buf.iterator += sizeof(topic.flaring);
	buf.offset += sizeof(topic.flaring);
	static_assert(sizeof(topic.abort_status) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.abort_status, sizeof(topic.abort_status));
	buf.iterator += sizeof(topic.abort_status);
	buf.offset += sizeof(topic.abort_status);
	return true;
}

static inline bool ucdr_deserialize_position_controller_landing_status(ucdrBuffer& buf, position_controller_landing_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.lateral_touchdown_offset) == 4, "size mismatch");
	memcpy(&topic.lateral_touchdown_offset, buf.iterator, sizeof(topic.lateral_touchdown_offset));
	buf.iterator += sizeof(topic.lateral_touchdown_offset);
	buf.offset += sizeof(topic.lateral_touchdown_offset);
	static_assert(sizeof(topic.flaring) == 1, "size mismatch");
	memcpy(&topic.flaring, buf.iterator, sizeof(topic.flaring));
	buf.iterator += sizeof(topic.flaring);
	buf.offset += sizeof(topic.flaring);
	static_assert(sizeof(topic.abort_status) == 1, "size mismatch");
	memcpy(&topic.abort_status, buf.iterator, sizeof(topic.abort_status));
	buf.iterator += sizeof(topic.abort_status);
	buf.offset += sizeof(topic.abort_status);
	return true;
}
