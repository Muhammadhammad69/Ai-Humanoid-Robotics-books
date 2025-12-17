# Implementation Plan: Book Content Embedding and Vector Storage in Qdrant using Cohere

**Branch**: `001-book-embeddings` | **Date**: 2025-12-16 | **Spec**: [link to spec.md](spec.md)
**Input**: Feature specification from `/specs/001-book-embeddings/spec.md`

## Summary

Implementation of a backend system to scan the /docs directory for Markdown files, chunk book content based on semantic boundaries (headings and paragraphs), generate embeddings using Cohere's embed-english-v3.0 model, and store vectors with comprehensive metadata in Qdrant Cloud. The system will support deterministic re-indexing with proper traceability from vectors back to source documents.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: qdrant-client, cohere, python-dotenv, PyYAML, markdown
**Storage**: Qdrant Cloud vector database
**Testing**: pytest
**Target Platform**: Linux server
**Project Type**: single - backend indexing pipeline
**Performance Goals**: Process documents at 1000+ chunks per minute, handle 10k+ book sections
**Constraints**: <200 RMP Cohere rate limit, Qdrant Cloud Free Tier limitations, <1GB memory usage
**Scale/Scope**: Up to 100 documents, 10k+ vector points, 100MB+ total book content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, this implementation plan adheres to the established architectural principles:
1. Uses Cohere as the specified embedding provider
2. Uses Qdrant as the specified vector database
3. Maintains separation between indexing and retrieval systems
4. Implements proper error handling and logging
5. Follows established patterns for configuration management

## Project Structure

### Documentation (this feature)

```text
specs/001-book-embeddings/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── __init__.py
│   │   ├── chunk.py              # Data models for content chunks
│   │   └── embedding.py          # Data models for embeddings
│   ├── services/
│   │   ├── __init__.py
│   │   ├── content_ingestor.py   # Service to scan and read Markdown files
│   │   ├── chunker.py            # Service to split content into semantic chunks
│   │   ├── embedding_service.py  # Service to generate embeddings via Cohere
│   │   └── vector_storage.py     # Service to store vectors in Qdrant
│   ├── cli/
│   │   ├── __init__.py
│   │   └── indexer_cli.py        # Command-line interface for the indexing process
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py           # Configuration loading from environment variables
│   └── utils/
│       ├── __init__.py
│       ├── file_utils.py         # File system utilities
│       └── markdown_parser.py    # Markdown parsing utilities
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   ├── test_embedding_service.py
│   │   └── test_vector_storage.py
│   ├── integration/
│   │   └── test_indexing_pipeline.py
│   └── fixtures/
│       └── sample_markdown.md
├── .env                           # Configuration file with API keys and settings
├── pyproject.toml                 # Project dependencies and build configuration
├── uv.lock                        # Lock file for uv package manager
└── README.md                      # Documentation for running the indexer
```

**Structure Decision**: Backend indexing pipeline with clear separation of concerns. The single project structure supports the indexing functionality with dedicated modules for content ingestion, chunking, embedding generation, and vector storage. CLI interface allows for both manual and scheduled execution of the indexing process.

## 1. Existing Backend Assessment

### Current Backend Structure
The backend directory currently contains a .env file with all required environment variables but no source code yet. The structure is minimal with only configuration present.

### uv Package Manager Integration
- Initialize Python project in backend/ using `uv init`
- Define dependencies in pyproject.toml using uv standards
- Use uv.lock for dependency resolution and reproducible builds
- Leverage uv's fast dependency resolution for faster CI/CD

### Dependencies Management
- Primary dependencies: qdrant-client, cohere, python-dotenv
- Optional dependencies: pytest (for testing), black, isort (for formatting)
- Version constraints to ensure compatibility with Cohere and Qdrant APIs

## 2. Environment Configuration Strategy

### Loading .env Variables
- Use python-dotenv to load configuration from backend/.env
- Centralize configuration access through a settings module
- Validate required environment variables at startup
- Fail fast if required variables are missing

### Configuration Access
- Single source of truth: environment variables from .env file
- Immutable configuration: loaded once at application startup
- Type validation: convert string values to appropriate types (integers, booleans)
- Default fallbacks: sensible defaults where appropriate, explicit errors where not

## 3. Data Flow Plan

### Step-by-step Flow: /docs → ingestion → chunking → Cohere embeddings → Qdrant

1. **Content Ingestion Module**: Scans /docs directory recursively for .md files, excluding non-content folders (assets, images, diagrams, code-samples)
2. **Chunking Module**: Parses each Markdown file into semantic chunks based on H1, H2, H3 headings and paragraph boundaries
3. **Embedding Module**: Sends chunks to Cohere API in batches respecting rate limits and input constraints
4. **Storage Module**: Stores embedding vectors with metadata payload in Qdrant collection

### Responsibility Boundaries
- Content Ingestor: File system scanning and Markdown content reading
- Chunker: Semantic splitting of content while preserving meaning
- Embedding Service: API communication with Cohere and batch processing
- Vector Storage: Qdrant interaction and metadata management

## 4. Chunking Execution Plan

### Markdown Heading Usage
- Parse Markdown AST to identify H1, H2, H3 elements
- Create chunks at each heading level with content following the heading
- Preserve document structure in chunk metadata
- Handle nested sections appropriately

### Paragraph Boundary Detection
- Split content between headings at paragraph boundaries
- Respect CHUNK_SIZE_TOKENS constraint while preserving semantic meaning
- Ensure no content is lost during splitting
- Maintain context around chunk boundaries

### Semantic Integrity Preservation
- Include relevant heading hierarchy in each chunk
- Preserve document context when splitting at boundaries
- Avoid breaking sentences or code blocks across chunks
- Ensure each chunk is self-contained and meaningful

## 5. Cohere Embedding Plan

### Embedding Request Structure
- Use embed-english-v3.0 model as specified in environment
- Set input_type to "search_document" for book content
- Pass texts in batches up to 96 items to maximize efficiency
- Include appropriate truncation strategy (END) for long texts

### Batching with COHERE_BATCH_SIZE
- Group chunks into batches respecting COHERE_BATCH_SIZE limit (10)
- Implement batch processing with proper error handling
- Monitor and adjust batch size based on actual API performance
- Include retry logic for failed batches

### Rate Limit Compliance
- Respect COHERE_RATE_LIMIT_RPM (60 requests per minute)
- Implement rate limiting with appropriate delays between batches
- Track API usage to prevent exceeding limits
- Queue requests when approaching rate limits

### Failure and Retry Considerations
- Implement exponential backoff for API errors
- Log failed requests for debugging and monitoring
- Resume indexing from point of failure
- Validate embedding dimensions match expected size (1024)

## 6. Qdrant Storage Plan

### Collection Strategy
- Use single collection named in QDRANT_COLLECTION_NAME (book_embeddings_en)
- Configure collection with QDRANT_VECTOR_SIZE (1024) and QDRANT_DISTANCE (Cosine)
- Ensure collection exists before attempting to store vectors
- Handle collection creation if it doesn't exist

### Vector Size and Distance Alignment
- Verify embedding vector dimensions match QDRANT_VECTOR_SIZE (1024)
- Use Cosine distance as specified in QDRANT_DISTANCE
- Validate vectors before insertion to prevent dimension mismatches

### Metadata Payload Structure
- Store comprehensive metadata including: document_path, section_type, module_name, chapter_name, section_heading, language (en), source_type (book), indexing_version
- Ensure all payload fields are properly indexed for efficient searching
- Validate payload structure against requirements before insertion

### ID Strategy for Deterministic Re-indexing
- Generate deterministic IDs based on document path and chunk position
- Use consistent hashing to ensure same document generates same IDs
- Enable safe replacement of outdated vectors during re-indexing
- Support deletion of old vectors during re-indexing operations

## 7. Re-indexing & Maintainability Plan

### Re-running Indexing Behavior
- Implement safe re-indexing that doesn't corrupt existing data
- Allow full re-indexing with complete replacement of old vectors
- Support incremental updates for changed documents only
- Maintain data integrity during the re-indexing process

### Outdated Vector Management
- Identify and remove vectors for deleted or modified documents
- Use indexing_version or timestamp to identify outdated vectors
- Implement cleanup process for maintaining vector store quality
- Preserve vectors for unchanged documents during partial updates

### Versioning Approach
- Include timestamp in indexing_version field for tracking updates
- Track indexing run history for debugging and monitoring
- Support rollback to previous indexing versions if needed
- Log indexing progress and completion for operational visibility

## 8. Risk & Constraint Analysis

### Qdrant Cloud Free Tier Limits
- Monitor storage capacity and vector count against free tier limits
- Implement size controls to avoid exceeding storage quotas
- Plan for migration path to paid tier if content grows beyond free tier
- Optimize vector storage efficiency and metadata size

### Cohere Rate Limits
- Implement robust rate limiting to stay within RPM constraints
- Design batching strategy to maximize efficiency within limits
- Monitor API usage and plan for potential quota increases
- Handle rate limit errors gracefully with appropriate delays

### Large Document Handling
- Implement streaming processing for very large Markdown files
- Handle memory usage for large documents to prevent out-of-memory errors
- Chunk large documents appropriately to stay within API limits
- Validate processing time for large files against reasonable limits

### Future Scalability Considerations
- Design for horizontal scaling if content volume increases
- Plan for multiple Qdrant collections if single collection becomes too large
- Consider content sharding strategies for very large book collections
- Design API interfaces to support future retrieval systems
