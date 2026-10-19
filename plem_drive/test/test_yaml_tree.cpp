// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_yaml_tree.cpp
/// @brief The strict-YAML reader under the drive model file: what it keeps
/// (structure, file order, plain vs quoted) and every syntax-level refusal of
/// schema §2, one inline input each.

#include <gtest/gtest.h>

#include <string>
#include <string_view>

#include "loader/yaml_tree.hpp"

namespace plem::drive::detail {
namespace {

std::string refusal(std::string_view text) {
    const auto result = parse_tree(text);
    return result ? std::string("<accepted>") : result.message();
}

TEST(YamlTree, KeepsStructureFileOrderAndQuoting) {
    const auto result = parse_tree("b: 1\na: [x, \"2\", {k: null}, \"é\"]\n");
    ASSERT_TRUE(result) << result.message();
    const YamlNode& root = result.value();
    ASSERT_EQ(root.kind, YamlNode::Kind::MAPPING);
    ASSERT_EQ(root.members.size(), 2u);
    EXPECT_EQ(root.members[0].first, "b") << "file order, not sorted";
    EXPECT_EQ(root.find("zz"), nullptr);
    const YamlNode* a = root.find("a");
    ASSERT_NE(a, nullptr);
    ASSERT_EQ(a->kind, YamlNode::Kind::SEQUENCE);
    ASSERT_EQ(a->items.size(), 4u);
    EXPECT_TRUE(a->items[0].plain);
    EXPECT_EQ(a->items[1].scalar, "2");
    EXPECT_FALSE(a->items[1].plain) << "a quoted scalar stays marked as text";
    ASSERT_EQ(a->items[2].kind, YamlNode::Kind::MAPPING);
    EXPECT_EQ(a->items[2].find("k")->scalar, "null") << "the reader types nothing — null is the schema layer's";
    EXPECT_EQ(a->items[3].scalar, "é");
}

TEST(YamlTree, AcceptsOneLeadingDocumentMarker) {
    EXPECT_EQ(refusal("---\na: 1\n"), "<accepted>");
}

TEST(YamlTree, RefusesEverythingOutsideTheSubset) {
    struct Row {
        const char* name;
        std::string_view text;
        const char* says;
    };
    const Row rows[] = {
        {"directive", "%YAML 1.2\n---\na: 1\n", "a %YAML or %TAG directive"},
        {"second document", "a: 1\n---\nb: 2\n", "a second document"},
        {"scalar anchor", "a: &x 1\n", "an anchor"},
        {"collection anchor", "a: &x {b: 1}\n", "an anchor"},
        {"alias", "a: 1\nb: *x\n", "an alias"},
        {"scalar tag", "a: !!int 5\n", "a tag (tag:yaml.org,2002:int)"},
        {"collection tag", "a: !!map {b: 1}\n", "a tag"},
        {"non-specific tag", "a: ! x\n", "a tag"},
        {"duplicate key", "a: 1\na: 2\n", "line 2, column 1: the key 'a' repeats"},
        {"nested duplicate key", "a: {b: 1, c: 2, b: 3}\n", "the key 'b' repeats"},
        {"literal block scalar", "a: |\n  x\n", "a block scalar"},
        {"folded block scalar", "a: >\n  x\n", "a block scalar"},
        {"sequence as key", "? [a, b]\n: 1\n", "a sequence used as a key"},
        {"mapping as key", "? {a: 1}\n: 2\n", "a mapping used as a key"},
        {"UTF-16LE with BOM",
         std::string_view("\xff\xfe"
                          "a\0:\0 \0"
                          "1\0",
                          10),
         "must be UTF-8 text, with no control characters but tab and line breaks"},
        {"syntax error", "a: [1, 2\n",
         "line 2, column 1: YAML syntax error: did not find expected ',' or ']' while parsing a flow sequence started "
         "at line 1, column 4"},
        {"no document", "# only a comment\n", "holds no document"},
    };
    for (const auto& row : rows) {
        SCOPED_TRACE(row.name);
        const std::string message = refusal(row.text);
        EXPECT_NE(message.find(row.says), std::string::npos) << message;
    }
}

TEST(YamlTree, SaysWhereAConstructBeganOnlyWhenThatIsElsewhere) {
    // A tab where the next token should start is its own context; the place is said once.
    EXPECT_EQ(refusal("a:\n\tb: 1\n"),
              "line 2, column 1: YAML syntax error: found character that cannot start any token while scanning for "
              "the next token");
    // An error libyaml reports with no context says one place too.
    EXPECT_EQ(refusal("a: 1\n...\nzz\n"),
              "line 3, column 1: YAML syntax error: did not find expected <document start>");
    // Elsewhere on the same line, or in the same column, is still elsewhere.
    EXPECT_EQ(refusal("a: \""),
              "line 1, column 5: YAML syntax error: found unexpected end of stream while scanning a quoted scalar "
              "started at line 1, column 4");
    EXPECT_EQ(refusal("a:\n  b: 1\n  c\n  d: 2\n"),
              "line 4, column 3: YAML syntax error: could not find expected ':' while scanning a simple key started "
              "at line 3, column 3");
}

TEST(YamlTree, CapsTheNestingDepth) {
    const auto nested = [](size_t depth) { return std::string(depth, '[') + std::string(depth, ']'); };
    EXPECT_EQ(refusal(nested(kMaxYamlDepth)), "<accepted>");
    EXPECT_NE(refusal(nested(kMaxYamlDepth + 1)).find("nesting deeper than 16 levels"), std::string::npos);
}

}  // namespace
}  // namespace plem::drive::detail
