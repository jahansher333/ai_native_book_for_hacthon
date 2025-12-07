# Feature Specification: Chapter Urdu Translation Button

**Feature Branch**: `006-urdu-translation`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Feature: Per-Chapter 'اردو میں پڑھیں' Button - Instant Urdu translation for all chapters using @urdu-translator subagent"

## User Scenarios & Testing

### User Story 1 - Instant Urdu Translation for All Users (Priority: P1)

Any user (logged-in or guest) can instantly translate any chapter from English to natural, technical Urdu by clicking a single button. The translation maintains technical accuracy while providing readable Urdu equivalents for robotics terms.

**Why this priority**: Core feature that delivers immediate value to Urdu-speaking users. This is the primary user journey and must work independently for the feature to be useful.

**Independent Test**: Navigate to any chapter, click "اردو میں پڑھیں" button, verify entire chapter content is replaced with accurate Urdu translation within 5-10 seconds.

**Acceptance Scenarios**:

1. **Given** a user is viewing any chapter in English, **When** they click the "اردو میں پڑھیں" button, **Then** the entire chapter content is replaced with natural Urdu translation maintaining technical accuracy
2. **Given** a chapter is being translated, **When** translation is in progress, **Then** a loading spinner appears with the message "اردو میں دکھایا جا رہا ہے"
3. **Given** a chapter has been translated to Urdu, **When** the translation completes, **Then** a badge appears showing "اردو میں" to indicate current language state

---

### User Story 2 - Toggle Back to English (Priority: P2)

Users can switch back to the original English content after viewing the Urdu translation, allowing them to compare translations or return to the source material.

**Why this priority**: Enables users to validate translations and switch between languages as needed. Depends on US1 being complete.

**Independent Test**: After translating a chapter to Urdu, click "English میں واپس" button and verify original English content is restored immediately.

**Acceptance Scenarios**:

1. **Given** a chapter is displayed in Urdu, **When** user clicks "English میں واپس" button, **Then** original English content is restored instantly without re-fetching
2. **Given** user switches back to English, **When** they click "اردو میں پڑھیں" again, **Then** previously cached Urdu translation loads instantly without re-translation

---

### User Story 3 - Technical Term Translation Quality (Priority: P1)

Urdu translations use natural, readable Urdu while maintaining technical precision for robotics terminology. Technical terms follow established conventions or provide contextual explanations.

**Why this priority**: Translation quality is critical for user trust and learning effectiveness. Poor translations would make the feature unusable.

**Independent Test**: Review translated chapter and verify technical terms are handled appropriately (e.g., "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2", "Latency trap" → "لیٹنسی ٹریپ (خطرناک تاخیر)").

**Acceptance Scenarios**:

1. **Given** a chapter contains the term "ROS 2", **When** translated to Urdu, **Then** it appears as "روبوٹک آپریٹنگ سسٹم 2" with proper context
2. **Given** a chapter contains "URDF", **When** translated to Urdu, **Then** it appears as "یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ" maintaining technical accuracy
3. **Given** a chapter contains "Jetson", **When** translated to Urdu, **Then** it appears as "جیٹسن" (transliterated proper noun)
4. **Given** a chapter contains "Latency trap", **When** translated to Urdu, **Then** it appears as "لیٹنسی ٹریپ (خطرناک تاخیر)" with contextual explanation

---

### Edge Cases

- What happens when translation fails due to API errors?
  - System displays error message in both English and Urdu
  - Original English content remains displayed
  - Retry button appears

- What happens when user navigates to a different chapter while translation is in progress?
  - Current translation request is cancelled
  - New chapter loads in default English state

- What happens when extremely long chapters (>10,000 words) are translated?
  - System chunks content if needed
  - Progress indicator shows translation status
  - Maximum timeout of 60 seconds applies

- What happens when user clicks translate button multiple times rapidly?
  - Only one translation request is processed
  - Button is disabled during translation

## Requirements

### Functional Requirements

- **FR-001**: System MUST display "اردو میں پڑھیں" button at the top of every chapter page
- **FR-002**: Button MUST be visible to all users regardless of authentication status
- **FR-003**: System MUST send entire chapter content to @urdu-translator subagent when button is clicked
- **FR-004**: System MUST replace chapter DOM content with translated Urdu text
- **FR-005**: System MUST display loading spinner with message "اردو میں دکھایا جا رہا ہے" during translation
- **FR-006**: System MUST show badge "اردو میں" after successful translation
- **FR-007**: System MUST provide "English میں واپس" button when chapter is in Urdu
- **FR-008**: System MUST restore original English content immediately when toggle button is clicked
- **FR-009**: System MUST cache Urdu translations in browser localStorage with 7-day TTL
- **FR-010**: System MUST sanitize translated HTML content before rendering
- **FR-011**: System MUST preserve markdown structure and formatting in translations
- **FR-012**: System MUST handle translation errors gracefully with bilingual error messages
- **FR-013**: System MUST disable translate button during active translation
- **FR-014**: System MUST apply 60-second timeout to translation requests
- **FR-015**: @urdu-translator subagent MUST translate technical terms appropriately

### Key Entities

- **Chapter**: Educational content unit to be translated
- **Translation Cache**: Browser-stored Urdu translations with 7-day expiration
- **Translation State**: Current language state (English, Urdu, Translating)

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can translate any chapter to Urdu in under 10 seconds
- **SC-002**: Translated content maintains 100% of original markdown structure
- **SC-003**: Cached translations load instantly (under 1 second)
- **SC-004**: Translation feature works without requiring user authentication
- **SC-005**: Users can toggle between English and Urdu without page reload
- **SC-006**: System handles translation errors gracefully in 100% of failure cases
- **SC-007**: Button remains accessible on all device viewports

## Assumptions

- @urdu-translator subagent exists and is configured
- Frontend has infrastructure for content replacement
- DOMPurify library is available
- localStorage is enabled in browsers
- Gemini API has sufficient quota

## Dependencies

- **Internal**: Chapter personalization infrastructure
- **Internal**: @urdu-translator subagent operational
- **External**: Gemini API availability

## Out of Scope

- Translating UI elements (menus, headers, footers)
- Supporting languages other than English-to-Urdu
- Offline translation
- User preference storage for default language
- Translation quality feedback mechanism

## Risks

- **API Quota Exhaustion**: Mitigation via aggressive caching
- **Translation Quality**: Extensive testing needed
- **Large Chapter Timeout**: Content chunking required
- **Cache Storage**: Implement LRU eviction
