---
id: 0002
title: i18n Urdu Support Plan
stage: plan
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: 006-i18n-language
branch: 006-i18n-language
user: user
command: /sp.plan 006-i18n-language
labels: ["i18n", "urdu", "translation", "docusaurus", "planning"]
links:
  spec: specs/006-i18n-language/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-i18n-language/plan.md
 - specs/006-i18n-language/research.md
 - specs/006-i18n-language/data-model.md
 - specs/006-i18n-language/quickstart.md
 - specs/006-i18n-language/contracts/i18n-api.yaml
tests:
 -
---

## Prompt

 006-i18n-language

Must run this command /sp.plan

IMPORTANT: Before creating this plan, use Context7 MCP to read the official Docusaurus i18n documentation at https://docusaurus.io/docs/i18n/introduction to ensure accuracy.

Create a detailed technical implementation plan for adding English-Urdu i18n support to this Docusaurus project using the native i18n system.

PROJECT CONTEXT:
- Docusaurus 3.9.2 with TypeScript
- Custom components in src/components/ and src/modules/
- 4 modules with extensive documentation in docs/modules/
- Hardware documentation in docs/hardware/
- Custom CSS in src/css/
- This will be feature spec 006-i18n-language

TECHNOLOGY STACK:
- Docusaurus 3.9.2 (already installed)
- Built-in i18n plugin (no additional npm packages needed)
- TypeScript for configuration and components
- React for custom component translation support
- Node.js scripts for automation
- Filesystem-based translation approach

ARCHITECTURE APPROACH:
Follow Docusaurus's official i18n architecture exactly:
1. Declare locales in docusaurus.config.ts
2. Use i18n/ur/ folder for Urdu translations at project root
3. Leverage automatic RTL support
4. Use write-translations CLI for scaffolding
5. Integrate with existing TypeScript setup
6. Preserve current build process

IMPLEMENTATION PHASES:

Phase 1: Foundation Setup (Configuration)
Files to modify:
- docusaurus.config.ts (add i18n configuration)
- package.json (add translation scripts)
- tsconfig.json (verify compatibility)

Tasks:
1.1 Update docusaurus.config.ts with i18n configuration
1.2 Add translation scripts to package.json
1.3 Create feature branch: 006-i18n-english-urdu
1.4 Create specs/006-i18n-english-urdu/ directory

Validation:
- Config compiles without TypeScript errors
- npm scripts work correctly

Phase 2: Scaffolding (Generate Translation Structure)
Command: npm run write-translations-ur

Expected output: i18n/ur/ folder with JSON files

Tasks:
2.1 Run write-translations command
2.2 Verify folder structure created
2.3 Check JSON files generated

Validation:
- i18n/ur/ folder exists at root
- All plugin folders created correctly

Phase 3: UI String Translation (JSON Files)
Files to translate:
- i18n/ur/docusaurus-theme-classic/navbar.json
- i18n/ur/docusaurus-theme-classic/footer.json
- i18n/ur/docusaurus-plugin-content-docs/current.json
- i18n/ur/code.json

Tasks:
3.1 Translate navbar labels
3.2 Translate footer text
3.3 Translate sidebar category labels
3.4 Extract and translate component strings

Validation:
- All JSON files have valid format
- No empty translation strings

Phase 4: Documentation Translation (High Priority)
Priority files:
- intro.md, preface.md, glossary.md, appendices.md
- Module landing pages (module-1/index.md through module-4/index.md)

Tasks:
4.1 Create directory structure in i18n/ur/
4.2 Translate root documentation files
4.3 Translate module landing pages
4.4 Preserve frontmatter and formatting

Validation:
- Markdown renders correctly
- Links work properly
- Code blocks preserved

Phase 5: Custom Component i18n Integration
Files to modify:
- src/pages/index.tsx
- src/components/ (all components with text)
- src/modules/ (module components)

Tasks:
5.1 Update homepage with Translate component
5.2 Update HeroSection component
5.3 Update HomepageFeatures component
5.4 Update module components
5.5 Extract strings to code.json

Validation:
- Components compile without errors
- Translations display correctly
- Hot reload works

Phase 6: Module Chapter Translation (Incremental)
This phase done incrementally after MVP.

Order:
1. Module 1: ROS 2 chapters
2. Module 2: Digital Twin chapters
3. Module 3: NVIDIA Isaac chapters
4. Module 4: VLA chapters

Tasks:
6.1 Translate module-1 chapters
6.2 Continue with other modules
6.3 Translate code-sample READMEs
6.4 Translate exercises

Validation:
- Chapter navigation works
- Code samples display correctly

Phase 7: RTL Layout & Styling
File to modify: src/css/custom.css

Tasks:
7.1 Test default RTL behavior
7.2 Add RTL CSS overrides if needed
7.3 Ensure code blocks stay LTR
7.4 Fix custom component RTL issues

Validation:
- Text flows right-to-left
- Layout works on all devices
- No visual breaks

Phase 8: Testing & Quality Assurance
Test scenarios:
1. Language switching
2. RTL layout
3. Translation completeness
4. Build process
5. SEO tags

Tasks:
8.1 Manual testing (npm run start:ur)
8.2 Build testing (npm run build:all)
8.3 SEO validation
8.4 Accessibility testing
8.5 Cross-browser testing

Validation:
- All tests pass
- No console errors
- Both languages work perfectly

Phase 9: Documentation & Handoff
Create:
- Translation workflow guide
- Terminology glossary
- Testing checklist
- Implementation notes

Tasks:
9.1 Document translation workflow
9.2 Create terminology glossary
9.3 Write testing guide
9.4 Update main README

Validation:
- Documentation complete
- Guidelines clear

REFERENCE DOCUMENTATION:
Use Context7 MCP to access:
- https://docusaurus.io/docs/i18n/introduction
- https://docusaurus.io/docs/api/docusaurus-config#i18n
- https://docusaurus.io/docs/cli#docusaurus-write-translations

## Response snapshot

Created comprehensive technical implementation plan for adding English-Urdu i18n support to the Docusaurus project. The plan includes:

- Detailed implementation plan document (plan.md) with technical context and project structure
- Research findings on Docusaurus i18n implementation (research.md) covering configuration, directory structure, RTL support, CLI commands, and component translation strategies
- Data model documenting key entities and validation rules (data-model.md)
- Quickstart guide with setup instructions (quickstart.md)
- API contract for language switching functionality (contracts/i18n-api.yaml)
- Updated agent context with new technology stack information

The implementation follows Docusaurus's official i18n architecture with proper RTL support for Urdu, file-based translation workflow, and preservation of technical content accuracy while enabling language switching functionality.

## Outcome

- ✅ Impact: Created complete technical implementation plan for i18n support with Urdu translation
- 🧪 Tests: Plan includes validation steps for each implementation phase
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and contracts/
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan addresses all core requirements while maintaining technical constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed with task generation phase
