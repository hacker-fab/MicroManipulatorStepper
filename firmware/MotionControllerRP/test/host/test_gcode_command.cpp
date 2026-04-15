// Tests for GCodeCommand (parsing) and CommandParser
#include <gtest/gtest.h>
#include "command_parser/command_parser.h"
#include <cmath>

using Status = GCodeCommand::EParseStatus;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static GCodeCommand parse(const char* line) {
    GCodeCommand cmd;
    cmd.from_command_str(line);
    return cmd;
}

static Status parse_status(const char* line) {
    GCodeCommand cmd;
    return cmd.from_command_str(line);
}

// ---------------------------------------------------------------------------
// Parse status
// ---------------------------------------------------------------------------
TEST(GCodeCommand, OkOnValidCommand) {
    EXPECT_EQ(parse_status("G0 X1.0 Y2.0 Z3.0 F100"), Status::OK);
}

TEST(GCodeCommand, OkOnCommandWithNoParams) {
    EXPECT_EQ(parse_status("M57"), Status::OK);
}

TEST(GCodeCommand, MalformedOnEmptyString) {
    EXPECT_EQ(parse_status(""), Status::MALFORMED_COMMAND);
}

TEST(GCodeCommand, MalformedOnLowercaseCommand) {
    EXPECT_EQ(parse_status("g0 X1.0"), Status::MALFORMED_COMMAND);
}

TEST(GCodeCommand, InvalidParameterOnLowercaseWord) {
    EXPECT_EQ(parse_status("G0 x1.0"), Status::INVALID_PARAMETER);
}

// ---------------------------------------------------------------------------
// get_command
// ---------------------------------------------------------------------------
TEST(GCodeCommand, CommandStringG0) {
    auto cmd = parse("G0 X1.0");
    EXPECT_EQ(cmd.get_command(), "G0");
}

TEST(GCodeCommand, CommandStringM56) {
    auto cmd = parse("M56 J1 S1");
    EXPECT_EQ(cmd.get_command(), "M56");
}

// ---------------------------------------------------------------------------
// has_word / get_value
// ---------------------------------------------------------------------------
TEST(GCodeCommand, HasWordDetectsPresent) {
    auto cmd = parse("G0 X1.5 Y2.5");
    EXPECT_TRUE(cmd.has_word('X'));
    EXPECT_TRUE(cmd.has_word('Y'));
    EXPECT_FALSE(cmd.has_word('Z'));
}

TEST(GCodeCommand, GetValueReturnsCorrectFloat) {
    auto cmd = parse("G1 X-1.5 Y0.0 Z3.14 F200");
    EXPECT_FLOAT_EQ(cmd.get_value('X'), -1.5f);
    EXPECT_FLOAT_EQ(cmd.get_value('Y'),  0.0f);
    EXPECT_NEAR(cmd.get_value('Z'), 3.14f, 1e-4f);
    EXPECT_FLOAT_EQ(cmd.get_value('F'), 200.0f);
}

TEST(GCodeCommand, GetValueReturnsDefaultForMissingWord) {
    auto cmd = parse("G0 X1.0");
    EXPECT_FLOAT_EQ(cmd.get_value('Z', 99.0f), 99.0f);
}

TEST(GCodeCommand, GetValueReturnsNaNForMissingWordNoDefault) {
    auto cmd = parse("G0 X1.0");
    EXPECT_TRUE(std::isnan(cmd.get_value('Z')));
}

TEST(GCodeCommand, WordWithNoValueTreatedAsZero) {
    // "A" alone after the command should parse as 0.0
    auto cmd = parse("G28 A");
    EXPECT_TRUE(cmd.has_word('A'));
    EXPECT_FLOAT_EQ(cmd.get_value('A'), 0.0f);
}

// ---------------------------------------------------------------------------
// contains_unsupported_words
// ---------------------------------------------------------------------------
TEST(GCodeCommand, NoUnsupportedWords) {
    auto cmd = parse("G0 X1.0 Y2.0 Z3.0 F10");
    EXPECT_FALSE(cmd.contains_unsupported_words("XYZF"));
}

TEST(GCodeCommand, DetectsUnsupportedWord) {
    auto cmd = parse("G0 X1.0 Y2.0 A5.0");
    EXPECT_TRUE(cmd.contains_unsupported_words("XYZ"));   // A is not in the list
    EXPECT_FALSE(cmd.contains_unsupported_words("XYZA")); // A is allowed
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------
TEST(GCodeCommand, ResetClearsState) {
    GCodeCommand cmd;
    cmd.from_command_str("G0 X1.0 Y2.0");
    cmd.reset();
    // reset() sets command[0]=0 and NaN-fills word_values
    EXPECT_FALSE(cmd.has_word('X'));
    EXPECT_FALSE(cmd.has_word('Y'));
    EXPECT_FALSE(cmd.has_word('Z'));
}

// ---------------------------------------------------------------------------
// Minimal mock so CommandParser::parse_line() can call send_reply() safely
// ---------------------------------------------------------------------------
class NullCommandProcessor : public ICommandProcessor {
  public:
    void send_reply(const char*) override {}
    bool can_process_command(const GCodeCommand&) override { return false; }
    void process_command(const GCodeCommand&, std::string&) override {}
};

// ---------------------------------------------------------------------------
// CommandParser — feed chars one by one
// ---------------------------------------------------------------------------
TEST(CommandParser, ParseLineReturnsTrue) {
    NullCommandProcessor proc;
    CommandParser parser;
    parser.set_command_processor(&proc);
    EXPECT_TRUE(parser.parse_line("G0 X1.0 Y2.0 Z3.0 F100"));
}

TEST(CommandParser, ParseLineMalformedReturnsFalse) {
    NullCommandProcessor proc;
    CommandParser parser;
    parser.set_command_processor(&proc);
    // Starts with a digit — not a valid G-code command letter
    EXPECT_FALSE(parser.parse_line("123 X1.0"));
}

TEST(CommandParser, FeedCharsBuildCommand) {
    NullCommandProcessor proc;
    CommandParser parser;
    parser.set_command_processor(&proc);
    const char* line = "G0 X1.0\n";
    for (const char* p = line; *p; ++p)
        parser.add_input_character(*p);
    EXPECT_TRUE(parser.is_command_ready());
}
