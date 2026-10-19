// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_cia402_profile.cpp
/// @brief The declaration rules validate_cia402_profile() enforces.
///
/// Every rejection test starts from a profile that validates and breaks
/// exactly one thing, and asserts the message that only that rule can
/// produce: several rules would reject most of these mutations, so
/// "non-empty" would not tell them apart.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>

#include "plem/fieldbus/cia402_profile.hpp"

namespace {

namespace detail = plem::fieldbus::detail;
using detail::Cia402Command;
using plem::fieldbus::cia402_state_name;
using plem::fieldbus::Cia402TransitionProfile;
using plem::fieldbus::ServoState;
using plem::fieldbus::validate_cia402_profile;

/// The NOT_READY + Shutdown table, by value so a test can
/// break it.
Cia402TransitionProfile valid_profile() {
    return plem::fieldbus::kNotReadyShutdownCia402Profile;
}

/// Index of the row declaring (from, to). Tests address rows by meaning, so
/// reordering the fixture cannot silently retarget a mutation.
size_t row_index(const Cia402TransitionProfile& profile, ServoState from, ServoState to) {
    for (size_t i = 0; i < profile.transition_count; ++i) {
        if (profile.transitions[i].from == from && profile.transitions[i].to == to) {
            return i;
        }
    }
    ADD_FAILURE() << "the fixture declares no " << cia402_state_name(from) << " → " << cia402_state_name(to) << " row";
    return 0;
}

void erase_row(Cia402TransitionProfile& profile, size_t index) {
    for (size_t j = index + 1; j < profile.transition_count; ++j) {
        profile.transitions[j - 1] = profile.transitions[j];
    }
    profile.transitions[--profile.transition_count] = {};
}

/// "SWITCHED_ON → OPERATION_ENABLED", the way every message spells a pair.
std::string pair_text(ServoState from, ServoState to) {
    return std::string(cia402_state_name(from)) + " → " + cia402_state_name(to);
}

testing::AssertionResult Mentions(const std::string& message, const std::string& fragment) {
    if (message.find(fragment) != std::string::npos) {
        return testing::AssertionSuccess();
    }
    return testing::AssertionFailure() << "expected a message mentioning \"" << fragment << "\", got: " << message;
}

// The fixture itself

TEST(Cia402ProfileValidation, TheSyntheticDeclarationValidates) {
    // Every mutation test below means nothing unless this holds first: the
    // fixture carries all ten required pairs, and also the extra rows the walk
    // allows without requiring: the NOT_READY compatibility row,
    // READY_TO_SWITCH_ON → OPERATION_ENABLED (transitions 3+4 in one word) and
    // OPERATION_ENABLED → QUICK_STOP_ACTIVE (transition 11).
    const auto valid = validate_cia402_profile(valid_profile());
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(Cia402ProfileValidation, TheStandardTableHoldsNoCompatibilityRowAndTheShippedOneAddsOnlyThat) {
    const Cia402TransitionProfile& standard = plem::fieldbus::kStandardCia402Profile;
    const Cia402TransitionProfile& shipped = plem::fieldbus::kNotReadyShutdownCia402Profile;
    const auto valid = validate_cia402_profile(standard);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    for (size_t i = 0; i < standard.transition_count; ++i) {
        EXPECT_NE(standard.transitions[i].from, ServoState::NOT_READY_TO_SWITCH_ON) << "row " << i;
    }
    ASSERT_EQ(shipped.transition_count, standard.transition_count + 1);
    EXPECT_EQ(shipped.transitions[0], (plem::fieldbus::Cia402Transition{ServoState::NOT_READY_TO_SWITCH_ON,
                                                                        ServoState::READY_TO_SWITCH_ON, 0x0006}));
    for (size_t i = 0; i < standard.transition_count; ++i) {
        EXPECT_EQ(shipped.transitions[i + 1], standard.transitions[i]) << "row " << i;
    }
}

// Structure

TEST(Cia402ProfileValidation, AnAbsentProfileIsRejected) {
    EXPECT_TRUE(Mentions(validate_cia402_profile(Cia402TransitionProfile{}).message(), "absent"));
}

TEST(Cia402ProfileValidation, ACountPastTheFixedCapacityIsRejected) {
    Cia402TransitionProfile profile = valid_profile();
    profile.transition_count = Cia402TransitionProfile::kMaxTransitions + 1;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "exceeds the fixed capacity"));
}

TEST(Cia402ProfileValidation, ARowNamingAnUndecodableStateIsRejected) {
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[0].to = ServoState::UNKNOWN;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "invalid state"));
}

TEST(Cia402ProfileValidation, ASelfTransitionIsRejected) {
    // A self row is not merely redundant: ServoStateMachine consults the
    // profile before its same-state hold word, so the row would replace the
    // core's hold command. The pairing rule would also reject this row, with
    // a message that cannot say that, hence its own diagnostic.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[row_index(profile, ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED)].to =
        ServoState::SWITCHED_ON;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "hold word"));
}

TEST(Cia402ProfileValidation, TwoRowsForOnePairAreRejectedEvenWithDifferentValidCommands) {
    // Transition 7 takes Disable voltage or Quick stop, so a profile can name
    // the same pair twice with two words the pairing rule is happy with. The
    // lookup returns the first match, so which one reaches the wire would
    // depend on declaration order; this is the input the duplicate rule alone
    // can catch, and a copy of a row (same word and all) would not test it.
    Cia402TransitionProfile profile = valid_profile();
    const size_t row = row_index(profile, ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED);
    ASSERT_EQ(profile.transitions[row].controlword, 0x0000) << "premise: the fixture declares Disable voltage";
    profile.transitions[profile.transition_count] = {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED,
                                                     0x0002};  // Quick stop
    profile.transition_count += 1;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "duplicate"));
}

// Controlword bits

TEST(Cia402ProfileValidation, AFaultResetBitIsRejectedAheadOfThePairingRule) {
    // 0x0086 decodes as Fault reset, which no state issues, so the pairing
    // rule would reject the row too, with a message that sends the reader
    // looking for the wrong command word instead of at bit 7.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[row_index(profile, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON)]
        .controlword = 0x0086;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "bit 7"));
}

TEST(Cia402ProfileValidation, AReservedBitIsRejectedEvenWhenTheCommandIsRight) {
    // 0x0406 still decodes as Shutdown and still lands where the row says, so
    // nothing but the reserved-bit rule can catch it.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[row_index(profile, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON)]
        .controlword = 0x0406;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "bit 10"));
}

TEST(Cia402ProfileValidation, ModeSpecificAndManufacturerBitsRideThrough) {
    // The bit rules cover 7 and 10 and nothing else. This pins the stated
    // scope, not an endorsement: bits 4-6, 8, 9 and 11-15 reach the drive
    // unexamined, and a profile carrying them is accepted here.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[row_index(profile, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON)]
        .controlword = 0xFB76;  // every unchecked bit set, low nibble still Shutdown
    const auto valid = validate_cia402_profile(profile);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

// The declared row's command must match the walk

TEST(Cia402ProfileValidation, ACommandThatStateNeverIssuesIsRejected) {
    // The documented MR-JET-G shortcut: Enable operation straight out of
    // SWITCH_ON_DISABLED. Out of scope by decision, and the message says so
    // rather than pretending the standard forbids it.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[profile.transition_count] = {ServoState::SWITCH_ON_DISABLED, ServoState::OPERATION_ENABLED,
                                                     0x000F};
    profile.transition_count += 1;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(),
                         "not one the standard walk issues from "
                         "SWITCH_ON_DISABLED"));
}

TEST(Cia402ProfileValidation, ARowOutsideTheRequiredTenIsCheckedToo) {
    // READY_TO_SWITCH_ON → OPERATION_ENABLED (transitions 3+4 in one word) is
    // not one of the ten, yet the sequencer looks it up and sends it whenever
    // a release head is OPERATION_ENABLED. Checking only the required pairs
    // would let this row be poisoned to a Disable voltage word, and the axis
    // would answer a release by dropping to SWITCH_ON_DISABLED.
    Cia402TransitionProfile profile = valid_profile();
    const size_t row = row_index(profile, ServoState::READY_TO_SWITCH_ON, ServoState::OPERATION_ENABLED);
    for (const auto& required : detail::kCia402RequiredTransitions) {
        ASSERT_FALSE(required.from == profile.transitions[row].from && required.to == profile.transitions[row].to)
            << "premise: this row must be OUTSIDE the required set";
    }
    profile.transitions[row].controlword = 0x0000;

    const std::string message = validate_cia402_profile(profile).message();
    EXPECT_TRUE(Mentions(message, pair_text(ServoState::READY_TO_SWITCH_ON, ServoState::OPERATION_ENABLED)));
    EXPECT_TRUE(Mentions(message, "maps that command to SWITCH_ON_DISABLED"));
}

TEST(Cia402ProfileValidation, TheOnlyRowOutOfNotReadyIsTheShutdownCompatibilityRow) {
    // Transition 1 is automatic, so no command leaves NOT_READY_TO_SWITCH_ON
    // in the standard. The one row that exists is a compatibility row (our
    // FSM starts there and such drives report 0x0020), and
    // the exception covers only that pair with that command.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[profile.transition_count] = {ServoState::NOT_READY_TO_SWITCH_ON, ServoState::SWITCHED_ON,
                                                     0x0007};
    profile.transition_count += 1;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(),
                         "not one the standard walk issues from "
                         "NOT_READY_TO_SWITCH_ON"));
}

// The walk must be declarable end to end

/// The ten pairs, written out independently of the production table. Driving
/// both the mutation and the expectation off detail::kCia402RequiredTransitions
/// would make this suite follow that table wherever it went: replace one entry
/// with a copy of another and a ten-element array still has ten elements, with
/// one pair silently no longer required and every test still green.
struct RequiredPair {
    int number;
    ServoState from;
    ServoState to;
};
constexpr std::array<RequiredPair, 10> kExpectedRequiredPairs{{
    {2, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON},
    {3, ServoState::READY_TO_SWITCH_ON, ServoState::SWITCHED_ON},
    {4, ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED},
    {5, ServoState::OPERATION_ENABLED, ServoState::SWITCHED_ON},
    {6, ServoState::SWITCHED_ON, ServoState::READY_TO_SWITCH_ON},
    {7, ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED},
    {8, ServoState::OPERATION_ENABLED, ServoState::READY_TO_SWITCH_ON},
    {9, ServoState::OPERATION_ENABLED, ServoState::SWITCH_ON_DISABLED},
    {10, ServoState::SWITCHED_ON, ServoState::SWITCH_ON_DISABLED},
    {12, ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED},
}};

TEST(Cia402ProfileValidation, TheRequiredTableIsExactlyThoseTenPairs) {
    ASSERT_EQ(detail::kCia402RequiredTransitions.size(), kExpectedRequiredPairs.size());
    for (const auto& expected : kExpectedRequiredPairs) {
        SCOPED_TRACE("transition " + std::to_string(expected.number));
        const auto match =
            std::find_if(detail::kCia402RequiredTransitions.begin(), detail::kCia402RequiredTransitions.end(),
                         [&](const auto& required) { return required.number == expected.number; });
        ASSERT_NE(match, detail::kCia402RequiredTransitions.end());
        EXPECT_EQ(match->from, expected.from);
        EXPECT_EQ(match->to, expected.to);
    }
}

TEST(Cia402ProfileValidation, EveryRequiredPairIsIndividuallyRequired) {
    // Drop one required row at a time. A rule that checked only some of the
    // ten would leave the dropped pair accepted, and the loop names which.
    for (const auto& expected : kExpectedRequiredPairs) {
        SCOPED_TRACE("transition " + std::to_string(expected.number));
        Cia402TransitionProfile profile = valid_profile();
        erase_row(profile, row_index(profile, expected.from, expected.to));
        const std::string message = validate_cia402_profile(profile).message();
        EXPECT_TRUE(Mentions(message, "transition " + std::to_string(expected.number)));
        EXPECT_TRUE(Mentions(message, pair_text(expected.from, expected.to)));
    }
}

TEST(Cia402ProfileValidation, TheQuickStopExitIsRequiredWithNoQuickStopEntryRow) {
    // A drive reaches QUICK_STOP_ACTIVE without any command of ours (a local
    // or external quick stop does it), so tying transition 12 to the presence
    // of OPERATION_ENABLED → QUICK_STOP_ACTIVE would leave an axis that got
    // there by itself with no commanded way out. Every shipped profile
    // declares the entry row, so only this input tells the two rules apart.
    Cia402TransitionProfile profile = valid_profile();
    erase_row(profile, row_index(profile, ServoState::OPERATION_ENABLED, ServoState::QUICK_STOP_ACTIVE));
    const auto valid = validate_cia402_profile(profile);
    ASSERT_TRUE(valid.is_ok()) << valid.message() << "premise: the entry row itself is optional";

    erase_row(profile, row_index(profile, ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED));
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "transition 12"));
}

TEST(Cia402ProfileValidation, ARowLeftBeyondTheCountDoesNotCountAsDeclared) {
    // Lowering the count without clearing the slot leaves transition 12's row
    // sitting in the array. Runtime lookups stop at transition_count, so that
    // axis has no commanded way out of QUICK_STOP_ACTIVE, but a check that
    // scanned the whole fixed array would see the row and accept the profile.
    // Every other rejection test here erases the slot as well, so this is the
    // only input that separates the two bounds.
    Cia402TransitionProfile profile = valid_profile();
    ASSERT_EQ(profile.transitions[profile.transition_count - 1].from, ServoState::QUICK_STOP_ACTIVE)
        << "premise: the last row is the one transition 12 needs";
    profile.transition_count -= 1;  // the row itself stays in the array
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "transition 12"));
}

TEST(Cia402ProfileValidation, EitherCommandForATwoCommandTransitionIsAccepted) {
    // Transitions 7 and 10 take Disable voltage or Quick stop. Every shipped
    // profile uses 0x0000, so a suite that only validated those profiles would
    // never notice the walk table losing the Quick stop branch, and a vendor
    // declaring 0x0002 would then be falsely refused. (The cell-by-cell walk
    // pin below catches that mutation as well; this one catches it through the
    // validator, on a literal word the table cannot rename.)
    for (const ServoState from : {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCHED_ON}) {
        SCOPED_TRACE(cia402_state_name(from));
        Cia402TransitionProfile profile = valid_profile();
        profile.transitions[row_index(profile, from, ServoState::SWITCH_ON_DISABLED)].controlword = 0x0002;
        const auto valid = validate_cia402_profile(profile);
        EXPECT_TRUE(valid.is_ok()) << valid.message();
    }
}

TEST(Cia402ProfileValidation, TheQuickStopExitIsCheckedWithNoQuickStopEntryRow) {
    // The companion to the requirement test: not just that transition 12's row
    // must be there without an entry row, but that its command is still
    // checked there. A pairing rule that skipped QUICK_STOP_ACTIVE whenever
    // nothing declared the way in would let the one row that gets an axis out
    // of a locally-triggered quick stop carry a word that does not.
    Cia402TransitionProfile profile = valid_profile();
    erase_row(profile, row_index(profile, ServoState::OPERATION_ENABLED, ServoState::QUICK_STOP_ACTIVE));
    profile.transitions[row_index(profile, ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED)].controlword =
        0x000F;  // Enable operation: transition 16, which lands in OPERATION_ENABLED

    const std::string message = validate_cia402_profile(profile).message();
    EXPECT_TRUE(Mentions(message, pair_text(ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED)));
    EXPECT_TRUE(Mentions(message, "maps that command to OPERATION_ENABLED"));
}

TEST(Cia402ProfileValidation, NonCanonicalEncodingsOfTheSameCommandAreAccepted) {
    // The decode is total over the low nibble, so every command has several
    // encodings and this repository writes only one of each (0x0000, 0x0002,
    // 0x0006, 0x0007, 0x000F). A check that recognised just those five would
    // pass every shipped profile and refuse a vendor that leaves a don't-care
    // bit set, which the standard permits.
    struct Case {
        ServoState from;
        ServoState to;
        uint16_t word;
        const char* command;
    };
    const Case kCases[] = {
        {ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON, 0x000E, "Shutdown, bit 3 set"},
        {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED, 0x0004, "Disable voltage, bit 2 set"},
        {ServoState::SWITCHED_ON, ServoState::SWITCH_ON_DISABLED, 0x000A, "Quick stop, bit 3 set"},
    };
    for (const Case& c : kCases) {
        SCOPED_TRACE(c.command);
        Cia402TransitionProfile profile = valid_profile();
        profile.transitions[row_index(profile, c.from, c.to)].controlword = c.word;
        const auto valid = validate_cia402_profile(profile);
        EXPECT_TRUE(valid.is_ok()) << valid.message();
    }
}

TEST(Cia402ProfileValidation, TheOptionalQuickStopReturnRowIsAccepted) {
    // Transition 16 (QUICK_STOP_ACTIVE → OPERATION_ENABLED) is legal but not
    // required: whether the drive stays in QUICK_STOP_ACTIVE to be resumed
    // from depends on its quick-stop option code. No shipped profile declares
    // it, so validating those alone would never notice the walk table losing
    // the row.
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[profile.transition_count] = {ServoState::QUICK_STOP_ACTIVE, ServoState::OPERATION_ENABLED,
                                                     0x000F};
    profile.transition_count += 1;
    const auto valid = validate_cia402_profile(profile);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

// Diagnostic priority: structural checks report before the pairing rule

TEST(Cia402ProfileValidation, AnUndecodableStateOutranksTheSelfRule) {
    Cia402TransitionProfile profile = valid_profile();
    profile.transitions[0].from = ServoState::UNKNOWN;
    profile.transitions[0].to = ServoState::UNKNOWN;  // also a self row
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "invalid state"));
}

TEST(Cia402ProfileValidation, ADuplicateOutranksThePairingRule) {
    // Both copies carry a word the pairing rule rejects, so whichever check
    // runs first on the earlier row decides the message.
    Cia402TransitionProfile profile = valid_profile();
    const size_t row = row_index(profile, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON);
    profile.transitions[row].controlword = 0x0000;  // Disable voltage: no move out of SWITCH_ON_DISABLED
    profile.transitions[profile.transition_count] = profile.transitions[row];
    profile.transition_count += 1;
    EXPECT_TRUE(Mentions(validate_cia402_profile(profile).message(), "duplicate"));
}

// The walk table the pairing rule reads

TEST(Cia402WalkTable, EveryStateAndCommandLandsWhereTheStandardSays) {
    // One cell per (state, command). The pairing rule is only as good as this
    // table, and a single wrong or missing cell either accepts a row that
    // misdrives an axis or refuses a legal vendor declaration; neither shows
    // up in a profile-level test as long as the shipped profiles avoid that
    // cell. Columns follow detail::kCia402Commands.
    constexpr ServoState kU = ServoState::UNKNOWN;
    constexpr ServoState kSod = ServoState::SWITCH_ON_DISABLED;
    constexpr ServoState kRtso = ServoState::READY_TO_SWITCH_ON;
    constexpr ServoState kSo = ServoState::SWITCHED_ON;
    constexpr ServoState kOe = ServoState::OPERATION_ENABLED;
    constexpr ServoState kQsa = ServoState::QUICK_STOP_ACTIVE;
    struct Row {
        ServoState from;
        std::array<ServoState, 6> target;  // Fault reset, Disable voltage, Quick stop, Shutdown, Switch on, Enable
    };
    constexpr std::array<Row, 9> kWalk{{
        // The compatibility row: Shutdown only, transition 1 being automatic.
        {ServoState::NOT_READY_TO_SWITCH_ON, {kU, kU, kU, kRtso, kU, kU}},
        {kSod, {kU, kU, kU, kRtso, kU, kU}},            // 2
        {kRtso, {kU, kSod, kSod, kU, kSo, kOe}},        // 7 · 3 · 3+4
        {kSo, {kU, kSod, kSod, kRtso, kU, kOe}},        // 10 · 6 · 4
        {kOe, {kU, kSod, kQsa, kRtso, kSo, kU}},        // 9 · 11 · 8 · 5
        {kQsa, {kU, kSod, kU, kU, kU, kOe}},            // 12 · 16
        {ServoState::FAULT, {kU, kU, kU, kU, kU, kU}},  // exit 15 is an edge, not a row
        {ServoState::FAULT_REACTION_ACTIVE, {kU, kU, kU, kU, kU, kU}},
        {kU, {kU, kU, kU, kU, kU, kU}},
    }};

    for (const Row& row : kWalk) {
        SCOPED_TRACE(cia402_state_name(row.from));
        for (size_t c = 0; c < detail::kCia402Commands.size(); ++c) {
            EXPECT_EQ(detail::standard_transition_target(row.from, detail::kCia402Commands[c]), row.target[c])
                << detail::cia402_command_name(detail::kCia402Commands[c]);
        }
    }
}

TEST(Cia402WalkTable, TheHintNamesEveryCommandThatReachesTheTarget) {
    // What the "add the row with a … command word" diagnostic offers. The
    // two-command transitions must offer both, or a vendor is steered to the
    // one encoding the walk table happens to have kept.
    const std::string hint =
        detail::cia402_commands_reaching(ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED);
    EXPECT_TRUE(Mentions(hint, "Disable voltage (0x0000)"));
    EXPECT_TRUE(Mentions(hint, "Quick stop (0x0002)"));

    for (const auto& expected : kExpectedRequiredPairs) {
        SCOPED_TRACE("transition " + std::to_string(expected.number));
        EXPECT_FALSE(detail::cia402_commands_reaching(expected.from, expected.to).empty())
            << "a required pair no command reaches would be a rule no profile could satisfy";
    }
}

TEST(Cia402ProfileValidation, TheExampleWordEachDiagnosticQuotesIsOneTheValidatorAccepts) {
    // A vendor follows the message: adds the named row with the quoted word.
    // Decoding back to the right command is not enough to promise that works:
    // 0x0406 also decodes as Shutdown, and the validator would then refuse the
    // very row it just asked for, over the reserved bit. So put each quoted
    // word into the row it is quoted for and require the profile to validate.
    for (const auto& expected : kExpectedRequiredPairs) {
        for (const Cia402Command command : detail::kCia402Commands) {
            if (detail::standard_transition_target(expected.from, command) != expected.to) {
                continue;
            }
            SCOPED_TRACE("transition " + std::to_string(expected.number) + " via " +
                         detail::cia402_command_name(command));
            Cia402TransitionProfile profile = valid_profile();
            profile.transitions[row_index(profile, expected.from, expected.to)].controlword =
                detail::cia402_representative_word(command);
            const auto valid = validate_cia402_profile(profile);
            EXPECT_TRUE(valid.is_ok()) << valid.message();
        }
    }
}

TEST(Cia402WalkTable, NoCommandReachesTheUnknownSentinel) {
    // UNKNOWN is how the table says "no such move", so a naive scan reports
    // every command as reaching it.
    EXPECT_EQ(detail::cia402_commands_reaching(ServoState::FAULT, ServoState::UNKNOWN), "");
}

// The controlword decode the rules rest on

TEST(Cia402CommandDecode, TheLowNibbleIsTotal) {
    // No "set of permitted controlwords" check can exist because every one of
    // the sixteen low-nibble values names a command. This is that table.
    constexpr std::array<Cia402Command, 16> kExpected{
        Cia402Command::DISABLE_VOLTAGE,  Cia402Command::DISABLE_VOLTAGE, Cia402Command::QUICK_STOP,
        Cia402Command::QUICK_STOP,       Cia402Command::DISABLE_VOLTAGE, Cia402Command::DISABLE_VOLTAGE,
        Cia402Command::SHUTDOWN,         Cia402Command::SWITCH_ON,       Cia402Command::DISABLE_VOLTAGE,
        Cia402Command::DISABLE_VOLTAGE,  Cia402Command::QUICK_STOP,      Cia402Command::QUICK_STOP,
        Cia402Command::DISABLE_VOLTAGE,  Cia402Command::DISABLE_VOLTAGE, Cia402Command::SHUTDOWN,
        Cia402Command::ENABLE_OPERATION,
    };
    for (uint16_t word = 0; word < kExpected.size(); ++word) {
        EXPECT_EQ(detail::cia402_command(word), kExpected[word]) << "low nibble 0x" << std::hex << word;
    }
}

TEST(Cia402CommandDecode, BitSevenOutranksEveryNibble) {
    for (uint16_t word = 0; word < 16; ++word) {
        EXPECT_EQ(detail::cia402_command(static_cast<uint16_t>(word | 0x0080)), Cia402Command::FAULT_RESET)
            << "low nibble 0x" << std::hex << word;
    }
}

TEST(Cia402CommandDecode, TheUncheckedBitsDoNotChangeTheCommand) {
    // Mode-specific, halt and manufacturer bits are don't-cares for the
    // decode: 0xFB7F is still Enable operation.
    for (uint16_t word = 0; word < 16; ++word) {
        EXPECT_EQ(detail::cia402_command(static_cast<uint16_t>(word | 0xFB70)), detail::cia402_command(word))
            << "low nibble 0x" << std::hex << word;
    }
}

TEST(Cia402CommandDecode, EveryRepresentativeWordDecodesBackToItsCommand) {
    // The example words the diagnostics quote are the inverse of the decode,
    // so a reader who copies one gets the command the message named.
    for (const Cia402Command command : detail::kCia402Commands) {
        EXPECT_EQ(detail::cia402_command(detail::cia402_representative_word(command)), command)
            << detail::cia402_command_name(command);
    }
}

}  // namespace
