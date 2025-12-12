---
id: 0001
title: i18n Urdu Support Spec
stage: spec
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: 006-i18n-language
branch: 006-i18n-language
user: user
command: /sp.specify 006-i18n-language
labels: ["i18n", "urdu", "translation", "docusaurus"]
links:
  spec: specs/006-i18n-language/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-i18n-language/spec.md
 - specs/006-i18n-language/checklists/requirements.md
tests:
 -
---

## Prompt

006-i18n-language

Must run this command /sp.specify
Branch Name: 006-i18n-language

Add internationalization (i18n) support to enable English and Urdu translations for the AI-Humanoid Robotics educational website.

IMPORTANT INSTRUCTIONS:
- Before planning or implementing, read and understand Docusaurus i18n documentation: https://docusaurus.io/docs/i18n/introduction
- Use Context7 MCP to access official Docusaurus documentation for implementation details
- Follow Docusaurus's filesystem-based translation workflow exactly as documented
- This will be feature spec 006 (previous specs are 001-005 in specs/ folder)

PROJECT CONTEXT:
This Docusaurus website has:
- 4 learning modules in docs/modules/ (module-1 to module-4)
- Hardware documentation in docs/hardware/
- Custom React components in src/components/ (HeroSection, HomepageFeatures, etc.)
- Module-specific components in src/modules/
- Custom CSS in src/css/
- Static assets in static/
- Existing navbar and sidebar configuration
- TypeScript configuration (docusaurus.config.ts, tsconfig.json)

WHAT TO BUILD:
Add i18n feature to translate the Docusaurus website into Urdu (اردو) while keeping English as the default language.

LANGUAGES:
- Default: English (en)
- Translation: Urdu (ur) only

SCOPE OF TRANSLATION:

1. Documentation Content (docs/ folder):
   - Root level: intro.md, preface.md, glossary.md, appendices.md
   - Hardware docs: docs/hardware/
   - All 4 modules: docs/modules/module-1/ through module-4/
     * Each module contains: index.md, chapter files, code-samples/, exercises/

2. Custom Components (src/ folder):
   - Homepage components in src/pages/index.tsx
   - Reusable components in src/components/
   - Module-specific components in src/modules/
   - Any text strings in React components

3. UI Elements:
   - Navbar labels (configured in docusaurus.config.ts)
   - Footer text
   - Sidebar category labels (sidebars.ts)
   - Search placeholder text
   - Theme labels

4. Blog posts (blog/ folder) - if any exist

WHAT NOT TO TRANSLATE:
- Code samples in modules (Python, ROS 2 code)
- Technical terms: ROS 2, NVIDIA Isaac, Gazebo, Unity, VLA, Digital Twin
- File paths, URLs, or Git branch names
- Configuration files (package.json, docusaurus.config.ts, tsconfig.json, sidebars.ts)
- API endpoints, function names, class names
- Command-line commands in documentation
- Existing specs in specs/ folder (001-005)

USER STORIES:

Story 1: Language Selection
As a Pakistani student learning robotics, I want to switch the website language to Urdu so that I can understand complex robotics concepts in my native language.

Acceptance Criteria:
- Language dropdown visible in navbar on all pages (next to GitHub link)
- Selecting Urdu reloads page with Urdu content and RTL layout
- Language preference persists across browser sessions using localStorage
- URL changes to /ur/ prefix when Urdu is selected (e.g., /ur/docs/intro)
- Current page context preserved when switching languages

Story 2: RTL Support for Urdu Readers
As an Urdu reader, I want proper right-to-left text layout so that technical content is naturally readable while preserving code block formatting.

Acceptance Criteria:
- Urdu pages render with RTL text direction automatically
- Navigation sidebar aligns to the right in RTL mode
- Code blocks remain LTR even within RTL pages
- Custom components in src/components/ adapt to RTL
- Module navigation in src/modules/ works correctly in RTL
- Design integrity maintained (no broken layouts or overlapping elements)
- Mobile responsive design works in RTL

Story 3: Content Translation Management
As a content manager, I want a clear structure to add and maintain Urdu translations so that educational content stays organized and synchronized with English version.

Acceptance Criteria:
- Translation files organized in i18n/ur/ folder following Docusaurus structure
- Markdown files translated with preserved formatting and frontmatter
- JSON files contain translated UI labels
- Missing translations fall back to English gracefully
- Translation structure mirrors docs/ organization (hardware/, modules/)
- Custom component strings extracted to i18n/ur/code.json

Story 4: Module-Specific Translation
As a learner, I want all 4 robotics modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) available in Urdu so I can follow the complete learning path in my language.

Acceptance Criteria:
- All module landing pages (module-1/index.md through module-4/index.md) translated
- Chapter content translated while preserving technical accuracy
- Code samples remain in original language with Urdu explanations
- Exercises and learning objectives translated
- Module navigation labels in Urdu in sidebar

Story 5: Custom Component Internationalization
As a developer, I want custom React components in src/ to support i18n so that the entire user experience is translated.

Acceptance Criteria:
- Homepage HeroSection component displays Urdu text
- HomepageFeatures component shows Urdu descriptions
- Module-specific components in src/modules/ handle Urdu
- All hardcoded strings extracted to translation files
- Components use Docusaurus's useDocusaurusContext for translations

Story 6: Verification & Testing
As a QA tester, I want to verify translations work correctly across all modules and components before deployment.

Acceptance Criteria:
- npm run start:ur launches Urdu development server
- All pages load without errors in Urdu
- Language switcher works on every page
- RTL layout tested on desktop, tablet, mobile viewports
- Search functionality works in Urdu
- External links and navigation functional in both languages
- Build process (npm run build) succeeds for both locales

FUNCTIONAL REQUIREMENTS:

FR1: Configuration Setup
- Update docusaurus.config.ts with i18n settings
- Define locales: en (default), ur
- Configure RTL direction for Urdu (direction: 'rtl')
- Set proper HTML lang attributes and hreflang tags
- Add locale dropdown to navbar items array

FR2: Folder Structure Creation
- Create i18n/ur/ directory at project root
- Mirror English content structure in i18n/ur/
- Organize by plugin:
  * i18n/ur/docusaurus-plugin-content-docs/current/
  * i18n/ur/docusaurus-plugin-content-blog/ (if needed)
  * i18n/ur/docusaurus-plugin-content-pages/
  * i18n/ur/docusaurus-theme-classic/

FR3: Translation File Generation
- Generate JSON files for UI labels using CLI
- Create navbar.json with navigation translations
- Create footer.json with footer text
- Create code.json for React component strings
- Translate current.json for sidebar category labels

FR4: Documentation Translation
- Translate root docs: intro.md, preface.md, glossary.md, appendices.md
- Translate hardware documentation in hardware/ folder
- Translate all 4 modules:
  * module-1: ROS 2 Foundations (10+ chapters)
  * module-2: Digital Twin (8+ chapters)
  * module-3: NVIDIA Isaac (12+ chapters)
  * module-4: VLA (15+ chapters)
- Preserve markdown formatting, frontmatter, admonitions
- Keep code blocks and technical terms untranslated

FR5: Custom Component i18n Integration
- Update src/pages/index.tsx to use i18n strings
- Modify src/components/ to support translations
- Update src/modules/ components for i18n
- Extract hardcoded strings to translation files
- Use Translate component from @docusaurus/Translate

FR6: UI Integration
- Add localeDropdown to navbar (position: 'right')
- Display language names: English, اردو
- Highlight current language in dropdown
- Store preference in localStorage
- Implement language toggle functionality

FR7: RTL Layout & Styling
- Apply RTL direction to Urdu pages automatically
- Add RTL-specific CSS overrides in src/css/custom.css if needed
- Ensure code blocks stay LTR using [dir="ltr"] attribute
- Test custom components in RTL mode
- Verify module navigation works in RTL

FR8: Build & Deployment Configuration
- Add npm scripts to package.json:
  * "write-translations-ur": "docusaurus write-translations --locale ur"
  * "start:ur": "docusaurus start --locale ur"
  * "build:ur": "docusaurus build --locale ur"
  * "build:all": "docusaurus build --locale en,ur"
- Configure build to generate separate folders for each locale
- Maintain SEO with proper hreflang tags
- Support incremental builds

CONSTRAINTS:
- Must use Docusaurus 3.9.2's native i18n system (no external libraries)
- English remains the source of truth
- Urdu translations can be done incrementally (prioritize high-traffic pages)
- Technical accuracy more important than literal translation
- Must work offline (no external translation APIs at runtime)
- Existing TypeScript configuration must be preserved
- Current build process and deployment should not break
- All existing 5 feature specs (001-005) remain unchanged

NON-FUNCTIONAL REQUIREMENTS:
- Build time increase should be minimal (<20%)
- Page load performance must remain fast in both languages
- SEO rankings should improve for Urdu-speaking regions
- Accessibility standards (WCAG AA) maintained in both languages
- Mobile responsiveness preserved in RTL
- Hot reload in development should work for both locales

TECHNICAL CONSTRAINTS:
- Node.js version compatibility with existing package.json
- TypeScript strict mode compliance
- No breaking changes to existing components
- Maintain current folder structure and conventions
- Follow existing code style and formatting

PROJECT FILE REFERENCES:
- Main config: docusaurus.config.ts (update i18n section)
- Sidebar config: sidebars.ts (translations in JSON)
- Package config: package.json (add translation scripts)
- Custom styles: src/css/custom.css (RTL overrides)
- Homepage: src/pages/index.tsx (add i18n support)

REFERENCE DOCUMENTATION:
Follow the official Docusaurus i18n documentation: https://docusaurus.io/docs/i18n/introduction
Use Context7 MCP to access latest Docusaurus docs during implementation.

## Response snapshot

Created comprehensive specification for internationalization (i18n) support for Urdu language on the AI-Humanoid Robotics educational website. The specification includes:

- 6 user stories with priorities (P1-P3) covering language selection, RTL support, content management, module translation, component i18n, and verification
- 16 functional requirements (FR-001 to FR-016) covering all aspects of i18n implementation
- 4 key entities (Language Preference, Translation Files, RTL Layout, Language Switching Mechanism)
- 8 success criteria (SC-001 to SC-008) with measurable outcomes
- Edge cases for fallback handling and language switching

The specification follows Docusaurus i18n best practices and maintains all constraints specified in the original requirements, including preserving technical terms and code samples while translating explanatory content.

## Outcome

- ✅ Impact: Created complete feature specification for i18n support with Urdu translation
- 🧪 Tests: Specification includes testable user stories and acceptance scenarios
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification addresses all core requirements while maintaining technical constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed with planning phase
