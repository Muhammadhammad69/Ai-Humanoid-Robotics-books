# Feature Specification: Internationalization (i18n) Support for Urdu Language

**Feature Branch**: `006-i18n-language`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "Add internationalization (i18n) support to enable English and Urdu translations for the AI-Humanoid Robotics educational website."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Language Selection (Priority: P1)

As a Pakistani student learning robotics, I want to switch the website language to Urdu so that I can understand complex robotics concepts in my native language.

**Why this priority**: This is the core functionality that enables all other i18n features. Without language switching, the entire internationalization effort has no value.

**Independent Test**: Can be fully tested by accessing the website, selecting Urdu from the language dropdown, and verifying that the content reloads in Urdu with proper RTL layout.

**Acceptance Scenarios**:

1. **Given** user is on any page of the website, **When** user clicks the language dropdown in the navbar, **Then** a list of available languages (English, اردو) is displayed
2. **Given** user has selected Urdu language, **When** page content loads, **Then** all UI elements and documentation content appear in Urdu with RTL text direction
3. **Given** user has switched to Urdu, **When** user navigates to different pages, **Then** language preference is maintained across the session

---

### User Story 2 - RTL Support for Urdu Readers (Priority: P1)

As an Urdu reader, I want proper right-to-left text layout so that technical content is naturally readable while preserving code block formatting.

**Why this priority**: Urdu is an RTL language, so proper text direction is essential for readability and user experience.

**Independent Test**: Can be fully tested by viewing any page in Urdu mode and verifying that text flows from right to left while code blocks remain left-to-right.

**Acceptance Scenarios**:

1. **Given** user is viewing content in Urdu, **When** page renders, **Then** text flows from right to left with proper RTL layout
2. **Given** page contains code blocks in Urdu context, **When** page renders, **Then** code blocks maintain LTR direction while surrounding text is RTL
3. **Given** user is on mobile device viewing Urdu content, **When** page loads, **Then** responsive design works correctly with RTL layout

---

### User Story 3 - Content Translation Management (Priority: P2)

As a content manager, I want a clear structure to add and maintain Urdu translations so that educational content stays organized and synchronized with English version.

**Why this priority**: This enables ongoing content management and ensures translations can be maintained efficiently over time.

**Independent Test**: Can be fully tested by examining the file structure and verifying that translations are organized in a clear, maintainable way.

**Acceptance Scenarios**:

1. **Given** translation files exist, **When** content manager reviews structure, **Then** Urdu translations are organized in a clear directory structure mirroring English content
2. **Given** missing Urdu translation exists, **When** user views page, **Then** content gracefully falls back to English version
3. **Given** translation files are updated, **When** build process runs, **Then** changes are properly reflected in the deployed site

---

### User Story 4 - Module-Specific Translation (Priority: P2)

As a learner, I want all 4 robotics modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) available in Urdu so I can follow the complete learning path in my language.

**Why this priority**: The educational value of the website depends on having complete learning modules available in the target language.

**Independent Test**: Can be fully tested by accessing each module in Urdu and verifying that all content is properly translated.

**Acceptance Scenarios**:

1. **Given** user accesses any module in Urdu, **When** module pages load, **Then** all module content (landing pages, chapters, exercises) appears in Urdu
2. **Given** module contains code samples, **When** user views in Urdu, **Then** code samples remain in original language while explanations are translated
3. **Given** user navigates through module in Urdu, **When** they use sidebar navigation, **Then** navigation labels appear in Urdu

---

### User Story 5 - Custom Component Internationalization (Priority: P3)

As a developer, I want custom React components in src/ to support i18n so that the entire user experience is translated.

**Why this priority**: Ensures consistency across the entire user interface, not just documentation content.

**Independent Test**: Can be fully tested by examining each custom component and verifying that it properly displays translated content.

**Acceptance Scenarios**:

1. **Given** custom React component displays text, **When** site is in Urdu mode, **Then** component displays translated text appropriately
2. **Given** homepage components exist, **When** user views homepage in Urdu, **Then** HeroSection and HomepageFeatures display Urdu content
3. **Given** module-specific components exist, **When** user views module pages in Urdu, **Then** components adapt to Urdu language

---

### User Story 6 - Verification & Testing (Priority: P3)

As a QA tester, I want to verify translations work correctly across all modules and components before deployment.

**Why this priority**: Ensures quality and reliability of the internationalization implementation.

**Independent Test**: Can be fully tested by running build processes and verifying that all language modes work correctly.

**Acceptance Scenarios**:

1. **Given** Urdu locale build command is executed, **When** build process completes, **Then** Urdu version of website is generated successfully
2. **Given** development server runs in Urdu mode, **When** user accesses /ur/ routes, **Then** pages load without errors
3. **Given** both languages are available, **When** user switches between them, **Then** language switching works reliably across all pages

---

### Edge Cases

- What happens when a specific page has no Urdu translation available? (Should fall back to English)
- How does the system handle users with browsers configured for different languages? (Should respect user's explicit language selection)
- What if the Urdu translation files are corrupted or incomplete? (Should gracefully degrade to English)
- How does search functionality work when switching between languages? (Should search within the selected language)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a language selector dropdown in the navbar that allows users to switch between English and Urdu
- **FR-002**: System MUST automatically apply RTL (right-to-left) text direction when Urdu language is selected
- **FR-003**: System MUST maintain LTR direction for code blocks even within RTL Urdu pages
- **FR-004**: System MUST preserve English as the default language for first-time visitors
- **FR-005**: System MUST store user language preference in localStorage to persist across browser sessions
- **FR-006**: System MUST update URLs with /ur/ prefix when Urdu is selected (e.g., /ur/docs/intro)
- **FR-007**: System MUST maintain current page context when switching languages between English and Urdu
- **FR-008**: System MUST organize Urdu translation files in i18n/ur/ directory following Docusaurus structure
- **FR-009**: System MUST translate all documentation content including root docs, hardware docs, and all 4 learning modules
- **FR-010**: System MUST preserve markdown formatting, frontmatter, and admonitions in translated content
- **FR-011**: System MUST keep code samples and technical terms untranslated while translating explanatory text
- **FR-012**: System MUST provide translated UI labels for navbar, footer, sidebar categories, and search elements
- **FR-013**: System MUST ensure custom React components properly display translated content
- **FR-014**: System MUST provide npm scripts for managing Urdu translations: write-translations-ur, start:ur, build:ur, build:all
- **FR-015**: System MUST maintain SEO with proper hreflang tags for both languages
- **FR-016**: System MUST gracefully fall back to English for any content not yet translated to Urdu

### Key Entities

- **Language Preference**: User's selected language (English or Urdu) with persistence mechanism
- **Translation Files**: Organized structure of Urdu content mirroring English documentation
- **RTL Layout**: Text direction configuration that affects page rendering for Urdu
- **Language Switching Mechanism**: System that handles transitions between languages while preserving context

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch between English and Urdu languages with less than 2 seconds page reload time
- **SC-002**: All documentation content across 4 learning modules, hardware docs, and root docs is available in Urdu within 3 months of implementation
- **SC-003**: 95% of website pages successfully load in both English and Urdu without errors
- **SC-004**: Language preference persists across browser sessions for at least 30 days
- **SC-005**: Build time increases by less than 20% when generating both English and Urdu versions
- **SC-006**: 90% of Urdu readers can successfully navigate and read content with proper RTL layout
- **SC-007**: Search functionality works effectively in both English and Urdu languages
- **SC-008**: Mobile responsiveness is maintained in both LTR and RTL layouts
