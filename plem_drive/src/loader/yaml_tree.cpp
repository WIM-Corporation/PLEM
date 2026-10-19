// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file yaml_tree.cpp
/// @brief libyaml event stream → YamlNode tree (see yaml_tree.hpp). The only
/// translation unit that includes <yaml.h>.

#include "loader/yaml_tree.hpp"

#include <yaml.h>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace plem::drive::detail {

const YamlNode* YamlNode::find(std::string_view key) const {
    const auto it = std::find_if(members.begin(), members.end(), [&](const auto& m) { return m.first == key; });
    return it == members.end() ? nullptr : &it->second;
}

namespace {

/// The parser and one event, each released however the walk ends.
struct Parser {
    yaml_parser_t handle{};
    explicit Parser(std::string_view text) {
        yaml_parser_initialize(&handle);
        yaml_parser_set_input_string(&handle, reinterpret_cast<const unsigned char*>(text.data()), text.size());
        // The file format is UTF-8 (schema §2): no encoding detection, so a
        // UTF-16 file is refused as invalid UTF-8 rather than transcoded.
        yaml_parser_set_encoding(&handle, YAML_UTF8_ENCODING);
    }
    ~Parser() {
        yaml_parser_delete(&handle);
    }
    Parser(const Parser&) = delete;
    Parser& operator=(const Parser&) = delete;
};

struct Event {
    yaml_event_t handle{};
    ~Event() {
        yaml_event_delete(&handle);
    }
};

/// A collection still being read, and (for a mapping) the key whose value
/// comes next.
struct Frame {
    YamlNode node;
    std::optional<std::string> key;
};

std::string place(const yaml_mark_t& mark) {
    return "line " + std::to_string(mark.line + 1) + ", column " + std::to_string(mark.column + 1);
}

std::string at(const yaml_mark_t& mark) {
    return place(mark) + ": ";
}

const char* kSubset = " — the drive model file is a strict YAML subset (schema §2)";

}  // namespace

core::Result<YamlNode> parse_tree(std::string_view text) {
    using Result = core::Result<YamlNode>;
    Parser parser(text);
    std::vector<Frame> open;
    std::optional<YamlNode> root;
    int documents = 0;

    // Hands a finished node to its parent: a sequence item, a mapping key, or
    // a mapping value. A key is checked for repetition before its value is read.
    const auto attach = [&](YamlNode node, const yaml_mark_t& mark) -> std::string {
        if (open.empty()) {
            root = std::move(node);
            return {};
        }
        Frame& parent = open.back();
        if (parent.node.kind == YamlNode::Kind::SEQUENCE) {
            parent.node.items.push_back(std::move(node));
        } else if (!parent.key) {
            if (parent.node.find(node.scalar) != nullptr) {
                return at(mark) + "the key '" + node.scalar + "' repeats in one mapping — write each key once" +
                       kSubset;
            }
            parent.key = std::move(node.scalar);
        } else {
            parent.node.members.emplace_back(std::move(*parent.key), std::move(node));
            parent.key.reset();
        }
        return {};
    };

    for (;;) {
        Event event;
        if (!yaml_parser_parse(&parser.handle, &event.handle)) {
            const yaml_parser_t& p = parser.handle;
            const std::string problem = p.problem != nullptr ? p.problem : "unknown error";
            if (p.error == YAML_READER_ERROR) {
                return Result::error(
                    "byte " + std::to_string(p.problem_offset) + ": " + problem +
                    " — the file must be UTF-8 text, with no control characters but tab and line breaks (schema §2)");
            }
            // libyaml also marks where the construct it was reading began. Print that only when it differs from the
            // problem's place (an unclosed bracket's opening); for a tab where the next token should start, both are
            // the same place.
            const bool began_elsewhere = p.context != nullptr && p.context_mark.index != p.problem_mark.index;
            return Result::error(at(p.problem_mark) + "YAML syntax error: " + problem +
                                 (p.context != nullptr ? " " + std::string(p.context) : "") +
                                 (began_elsewhere ? " started at " + place(p.context_mark) : ""));
        }
        const yaml_event_t& e = event.handle;
        const yaml_mark_t& mark = e.start_mark;
        std::string error;
        switch (e.type) {
            case YAML_STREAM_START_EVENT:
            case YAML_DOCUMENT_END_EVENT:
                break;
            case YAML_DOCUMENT_START_EVENT:
                if (++documents > 1) {
                    return Result::error(at(mark) + "a second document — the file holds exactly one" + kSubset);
                }
                if (e.data.document_start.version_directive != nullptr ||
                    e.data.document_start.tag_directives.start != e.data.document_start.tag_directives.end) {
                    return Result::error(at(mark) + "a %YAML or %TAG directive — delete it" + kSubset);
                }
                break;
            case YAML_STREAM_END_EVENT:
                if (!root) {
                    return Result::error("the file holds no document — write the drive model's keys");
                }
                return Result::ok(std::move(*root));
            case YAML_ALIAS_EVENT:
                return Result::error(at(mark) + "an alias (*name) — write the value out in place" + kSubset);
            case YAML_SCALAR_EVENT: {
                const auto& s = e.data.scalar;
                if (s.anchor != nullptr) {
                    return Result::error(at(mark) + "an anchor (&name) — write the value out in place" + kSubset);
                }
                if (s.tag != nullptr) {
                    return Result::error(at(mark) + "a tag (" + reinterpret_cast<const char*>(s.tag) +
                                         ") — no value takes a tag; delete it" + kSubset);
                }
                if (s.style == YAML_LITERAL_SCALAR_STYLE || s.style == YAML_FOLDED_SCALAR_STYLE) {
                    return Result::error(at(mark) + "a block scalar (| or >) — write the value on one line" + kSubset);
                }
                YamlNode node;
                node.scalar.assign(reinterpret_cast<const char*>(s.value), s.length);
                node.plain = s.style == YAML_PLAIN_SCALAR_STYLE;
                error = attach(std::move(node), mark);
                break;
            }
            case YAML_SEQUENCE_START_EVENT:
            case YAML_MAPPING_START_EVENT: {
                const bool mapping = e.type == YAML_MAPPING_START_EVENT;
                const yaml_char_t* anchor = mapping ? e.data.mapping_start.anchor : e.data.sequence_start.anchor;
                const yaml_char_t* tag = mapping ? e.data.mapping_start.tag : e.data.sequence_start.tag;
                if (anchor != nullptr) {
                    return Result::error(at(mark) + "an anchor (&name) — write the value out in place" + kSubset);
                }
                if (tag != nullptr) {
                    return Result::error(at(mark) + "a tag (" + reinterpret_cast<const char*>(tag) +
                                         ") — no value takes a tag; delete it" + kSubset);
                }
                if (!open.empty() && open.back().node.kind == YamlNode::Kind::MAPPING && !open.back().key) {
                    return Result::error(at(mark) + "a " + (mapping ? "mapping" : "sequence") +
                                         " used as a key — keys are plain names" + kSubset);
                }
                if (open.size() == kMaxYamlDepth) {
                    return Result::error(at(mark) + "nesting deeper than " + std::to_string(kMaxYamlDepth) +
                                         " levels — no drive model key sits that deep" + kSubset);
                }
                Frame frame;
                frame.node.kind = mapping ? YamlNode::Kind::MAPPING : YamlNode::Kind::SEQUENCE;
                open.push_back(std::move(frame));
                break;
            }
            case YAML_SEQUENCE_END_EVENT:
            case YAML_MAPPING_END_EVENT: {
                YamlNode node = std::move(open.back().node);
                open.pop_back();
                error = attach(std::move(node), mark);
                break;
            }
            case YAML_NO_EVENT:
                break;
        }
        if (!error.empty()) {
            return Result::error(error);
        }
    }
}

}  // namespace plem::drive::detail
