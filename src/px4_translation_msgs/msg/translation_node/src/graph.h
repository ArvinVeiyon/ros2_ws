/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "util.h"
#include <algorithm>
#include <bitset>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

// This implements a directed graph with potential cycles used for translation.
// There are 2 types of nodes: messages (e.g. publication/subscription endpoints) and
// translations. Translation nodes are always in between message nodes, and can have N input messages
// and M output messages.

struct MessageIdentifier {
	std::string topic_name;
	MessageVersionType version;

	bool operator==(const MessageIdentifier& other) const {
		return topic_name == other.topic_name && version == other.version;
	}
	bool operator!=(const MessageIdentifier& other) const {
		return !(*thi