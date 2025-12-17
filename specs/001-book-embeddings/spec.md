# Feature Specification: Book Content Embedding and Vector Storage in Qdrant using Cohere

**Feature Branch**: `001-book-embeddings`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: " 007-book-embeddings

Must run this command /sp.specify

Title:
Book Content Embedding and Vector Storage in Qdrant using Cohere

Context:
This specification defines a backend-only system for embedding a Docusaurus-based technical book
and storing its vector representations in Qdrant Cloud.

The project repository is named "AI-Humanoid-Robotics-books-4".
All book content lives under the /docs directory and is written in Markdown.

This specification is ONLY about:
- Reading book content
- Chunking it
- Generating embeddings using a Cohere embedding model
- Storing the embeddings in Qdrant

This specification DOES NOT include:
- Query-time retrieval
- Selected-text-only answering
- Mixed queries
- LLM answer generation
- Agents
- FastAPI routes
- Chatbot UI

It is strictly an indexing and storage specification.

---

In Scope:
- Parsing Markdown files under /docs
- Chunking book content into embedding-friendly units
- Generating embeddings using Cohere
- Persisting vectors and metadata in Qdrant Cloud

Out of Scope:
- Any runtime query handling
- Any question answering logic
- Any user-selected text handling
- Any frontend or API logic

---

Book Content Source:
- Root directory: /docs
- Includes:
  - /docs/intro.md
  - /docs/hardware/*
  - /docs/modules/module-1/*
  - /docs/modules/module-2/*
  - /docs/modules/module-3/*
  - /docs/modules/module-4/*
- Only Markdown (.md) files are processed
- Assets, images, diagrams, and code samples are ignored

---

Functional Requirements:

1. The system must recursively scan the /docs directory for Markdown files.
2. Non-content folders (assets, images, diagrams, code-samples) must be excluded.
3. Each Markdown file must be parsed into logical chunks.
4. Chunking must be based on:
   - Markdown headings (H1, H2, H3)
   - Paragraph boundaries
5. Each chunk must:
   - Preserve semantic meaning
   - Be suitable for Cohere embedding input limits
6. Each chunk must be embedded using a Cohere embedding model.
7. Each embedding must be stored in Qdrant Cloud as a vector point.

---

Embedding Requirements:
- Cohere is the ONLY embedding provider.
- Embeddings are generated ONLY for book content.
- No query embeddings are generated in this specification.
- Batching may be used but is not mandated.

---

Vector Storage Requirements (Qdrant):

Each stored vector must include a payload (metadata) containing:

- document_path (relative path inside /docs)
- section_type (intro, hardware, module)
- module_name (if applicable)
- chapter_name or filename
- section_heading (nearest Markdown heading)
- language (default: en)
- source_type = "book"
- indexing_version or timestamp

The vector ID strategy must allow:
- Deterministic re-indexing
- Safe replacement of outdated vectors

---

Non-Functional Requirements:

- The system must be compatible with Qdrant Cloud Free Tier limitations.
- The design must be implementation-agnostic.
- Indexing must be repeatable and idempotent.
- Metadata must allow exact traceability back to the book structure.

---

Constraints:

- Cohere is the only embedding model provider.
- Qdrant Cloud is the only vector database.
- No other databases or vector stores are used.
- This specification must not include retrieval or answering logic.

---

Deliverables:

- A complete specification describing:
  - Book content ingestion rules
  - Chunking strategy
  - Cohere embedding responsibility
  - Qdrant vector and metadata schema (conceptual)

This specification will later be used as the foundation for:
- Retrieval logic
- RAG pipelines
- Chatbot and agent systems

---

Acceptance Criteria:

- Every meaningful section of the book is embedded.
- Every vector stored in Qdrant can be traced back to:
  - File
  - Module
  - Chapter
  - Section
- No non-book or user-provided text is embedded."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Content Indexing (Priority: P1)

As a system administrator, I want to automatically scan and embed all book content from the /docs directory so that the technical book content is available in vector form for later retrieval and analysis.

**Why this priority**: This is the foundational capability that enables all subsequent vector-based operations. Without properly indexed book content, no retrieval or search functionality can work.

**Independent Test**: Can be fully tested by running the indexing process on the /docs directory and verifying that vectors are stored in Qdrant with appropriate metadata. Delivers the core value of making book content searchable and analyzable.

**Acceptance Scenarios**:

1. **Given** book content exists in /docs directory with Markdown files, **When** the indexing process runs, **Then** all Markdown content is converted to vectors and stored in Qdrant with proper metadata
2. **Given** the indexing process is initiated, **When** it encounters non-Markdown files, **Then** those files are ignored and only Markdown content is processed

---

### User Story 2 - Content Chunking and Semantic Preservation (Priority: P2)

As a content engineer, I want the system to intelligently chunk the book content based on semantic boundaries (headings, paragraphs) so that the meaning of content is preserved in the vector embeddings.

**Why this priority**: Proper chunking ensures that when content is retrieved later, it maintains logical and semantic coherence, which is essential for accurate understanding and response generation.

**Independent Test**: Can be tested by examining the chunks generated from sample Markdown files and verifying they align with semantic boundaries like headings and paragraphs. Delivers the value of coherent content retrieval.

**Acceptance Scenarios**:

1. **Given** a Markdown file with headings and paragraphs, **When** the chunking process runs, **Then** content is divided into logical sections based on H1, H2, H3 headings and paragraph boundaries
2. **Given** content that exceeds Cohere embedding input limits, **When** chunking occurs, **Then** the content is split into appropriately sized chunks while preserving semantic meaning

---

### User Story 3 - Vector Storage and Metadata Management (Priority: P3)

As a data engineer, I want the system to store vectors with comprehensive metadata in Qdrant Cloud so that each vector can be traced back to its original source location in the book structure.

**Why this priority**: Proper metadata enables traceability and allows the system to provide accurate source attribution when content is retrieved later.

**Independent Test**: Can be tested by examining stored vectors in Qdrant and verifying all required metadata fields are present and accurate. Delivers the value of reliable source tracking.

**Acceptance Scenarios**:

1. **Given** content has been embedded and stored in Qdrant, **When** metadata is examined, **Then** it includes document_path, section_type, module_name, chapter_name, section_heading, language, source_type, and indexing_version
2. **Given** a vector in Qdrant, **When** its metadata is accessed, **Then** it can be traced back to the exact file, module, chapter, and section in the original book structure

---

### Edge Cases

- What happens when a Markdown file contains malformed content or syntax errors?
- How does the system handle extremely large Markdown files that exceed processing limits?
- What occurs when the same file is processed multiple times during re-indexing?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST recursively scan the /docs directory for Markdown files
- **FR-002**: System MUST exclude non-content folders (assets, images, diagrams, code-samples) from processing
- **FR-003**: System MUST parse each Markdown file into logical chunks based on headings (H1, H2, H3) and paragraph boundaries
- **FR-004**: Each chunk MUST preserve semantic meaning and be suitable for Cohere embedding input limits
- **FR-005**: System MUST embed each chunk using a Cohere embedding model
- **FR-006**: System MUST store each embedding as a vector point in Qdrant Cloud
- **FR-007**: Each stored vector MUST include payload metadata containing document_path, section_type, module_name, chapter_name, section_heading, language, source_type, and indexing_version or timestamp
- **FR-008**: System MUST implement a vector ID strategy that allows deterministic re-indexing and safe replacement of outdated vectors
- **FR-009**: System MUST be compatible with Qdrant Cloud Free Tier limitations
- **FR-010**: Indexing process MUST be repeatable and idempotent
- **FR-011**: Metadata MUST allow exact traceability back to the book structure

### Key Entities

- **Book Content Chunk**: A semantic unit of book content derived from Markdown files, suitable for embedding generation
- **Vector Embedding**: Numerical representation of a content chunk generated by the Cohere embedding model
- **Qdrant Vector Point**: Storage unit in Qdrant containing the vector embedding and associated metadata
- **Metadata Payload**: Information associated with each vector point that enables traceability back to source content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All meaningful sections of the book are embedded with 100% coverage of Markdown content
- **SC-002**: Every vector stored in Qdrant can be traced back to its original file, module, chapter, and section with 100% accuracy
- **SC-003**: No non-book or user-provided text is embedded, achieving 100% content accuracy
- **SC-004**: The indexing process completes successfully within reasonable timeframes for the book size
- **SC-005**: The system can handle re-indexing operations without corrupting or duplicating vectors in Qdrant
