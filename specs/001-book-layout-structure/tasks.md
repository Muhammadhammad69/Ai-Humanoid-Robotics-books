# Tasks: Book Layout and Structure

**Input**: Design documents from `/specs/001-book-layout-structure/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test tasks. Therefore, test tasks will not be generated in this document.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths are relative to the repository root unless specified otherwise.
- Book content paths are relative to `/docs/`.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Docusaurus project initialization and basic configuration.

- [X] T001 Initialize a Docusaurus project at the repository root.
- [X] T002 Configure `docusaurus.config.js` with basic site metadata (title, tagline, author).
- [X] T003 Configure `sidebars.js` for initial auto-generated table of contents.
- [X] T004 Install necessary Node.js dependencies for Docusaurus.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create the core directory structure for book content.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Create the main `/docs` directory.
- [X] T006 Create the `/docs/modules` directory.
- [X] T007 [P] Create module folders: `/docs/modules/module-1`, `/docs/modules/module-2`, `/docs/modules/module-3`, `/docs/modules/module-4`.
- [X] T008 [P] Add placeholder `_category_.json` file in each module folder to define module title.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Define Book Structure (Priority: P1) 🎯 MVP

**Goal**: Define the hierarchical structure of a book, including modules, chapters, and sections.

**Independent Test**: The generated directory structure and placeholder markdown files can be verified against the specified hierarchy, and a Docusaurus build should reflect this structure in navigation.

### Implementation for User Story 1

- [X] T009 [US1] For each module folder (`/docs/modules/module-X`), create 2-5 chapter markdown files (e.g., `/docs/modules/module-1/chapter-1-intro.md`, `/docs/modules/module-1/chapter-2.md`).
- [X] T010 [P] [US1] Add Docusaurus frontmatter to each chapter markdown file for sidebar navigation.
- [X] T011 [P] [US1] Add a chapter title and a placeholder summary paragraph to each chapter markdown file.
- [X] T012 [US1] Within each chapter markdown file, outline 3-7 sections using markdown headers (`## Section Title`).
- [X] T013 [P] [US1] For each section, add 1-3 optional subsections using `### Subsection Title` headers.
- [X] T014 [P] [US1] Include placeholder introduction paragraphs for each section and subsection.
- [X] T015 [US1] Add a "Key Takeaways" placeholder at the end of each chapter markdown file.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Define Standard Book Pages (Priority: P2)

**Goal**: Create standard book pages like a title page, preface, and table of contents.

**Independent Test**: The created standard pages (preface, TOC) can be verified for correct content and Docusaurus integration.

### Implementation for User Story 2

- [X] T016 [US2] Create `preface.md` in `/docs/` with book purpose, target audience, and how to use the book.
- [X] T017 [US2] Create a dedicated `toc.md` in `/docs/` or confirm `sidebars.js` auto-generates the TOC correctly. If `toc.md` is created, ensure it links to all modules and chapters.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Define Ancillary Content Pages (Priority: P3)

**Goal**: Include appendices, references, and a glossary.

**Independent Test**: The created ancillary pages (appendices, references, glossary) can be verified for correct content and Docusaurus integration.

### Implementation for User Story 3

- [X] T018 [US3] Create `appendices.md` in `/docs/` for future reference material.
- [X] T019 [US3] Create `glossary.md` in `/docs/` for key terms and definitions.
- [X] T020 [US3] Ensure `docusaurus.config.js` includes links to these new pages in the sidebar or navbar if desired.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, navigation, and quality checks.

- [X] T021 Verify the entire folder structure under `/docs/` matches the layout plan.
- [X] T022 Confirm chapter → section → subsection hierarchy is consistent across all modules.
- [X] T023 Ensure Docusaurus sidebar recognizes all modules and chapters correctly.
- [X] T024 Check all markdown headers, frontmatter, and file naming conventions for consistency.
- [X] T025 Make sure each module and chapter folder has proper placeholders as defined.
- [X] T026 Run a local Docusaurus build (`npm run build`) and verify no errors.
- [X] T027 Run quickstart.md validation (ensure instructions are still accurate).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 (e.g., TOC linking to US1-generated structure) but should be independently testable for its core functionality.
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2.

### Within Each User Story

- Core implementation before integration.
- Story complete before moving to next priority.

### Parallel Opportunities

- All Setup tasks marked [P] (T007, T008) can run in parallel.
- Once Foundational phase completes, user stories can be worked on sequentially by priority or in parallel by different team members.
- Within User Story 1, tasks T010, T011, T013, T014 can run in parallel.

---

## Parallel Example: User Story 1

```bash
# Launch tasks that can run in parallel within User Story 1:
# Task: "Add Docusaurus frontmatter to each chapter markdown file for sidebar navigation."
# Task: "Add a chapter title and a placeholder summary paragraph to each chapter markdown file."
# Task: "For each section, add 1-3 optional subsections using '### Subsection Title' headers."
# Task: "Include placeholder introduction paragraphs for each section and subsection."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
