# Book Content Embedding System

Backend system for embedding book content and storing vectors in Qdrant using Cohere.

## Overview

This system scans the `/docs` directory for Markdown files, chunks book content based on semantic boundaries (headings and paragraphs), generates embeddings using Cohere's embed-english-v3.0 model, and stores vectors with comprehensive metadata in Qdrant Cloud. The system supports deterministic re-indexing with proper traceability from vectors back to source documents.

## Features

- Recursive scanning of `/docs` directory for Markdown files
- Intelligent chunking based on semantic boundaries (headings, paragraphs)
- Embedding generation using Cohere API
- Vector storage in Qdrant Cloud with comprehensive metadata
- Deterministic re-indexing capabilities
- Traceability from vectors back to source documents

## Requirements

- Python 3.11+
- uv package manager
- Access to Cohere API
- Access to Qdrant Cloud

## Installation

1. Clone the repository
2. Navigate to the backend directory: `cd backend`
3. Install dependencies with uv: `uv sync`

## Usage

Run the indexer CLI:
```bash
uv run src/cli/indexer_cli.py
```

### CLI Options

- `--docs-path`: Path to the docs directory (default: from environment)
- `--dry-run`: Run without storing vectors in Qdrant
- `--reindex`: Perform a full re-index, replacing all existing vectors
- `--incremental`: Perform incremental update for changed documents only

Examples:
```bash
# Basic indexing
uv run src/cli/indexer_cli.py

# Dry run (test without storing)
uv run src/cli/indexer_cli.py --dry-run

# Full re-indexing
uv run src/cli/indexer_cli.py --reindex

# Index specific docs path
uv run src/cli/indexer_cli.py --docs-path ./custom_docs
```

## Configuration

The system uses environment variables for configuration. Create a `.env` file in the backend directory with the following variables:

- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings
- `QDRANT_VECTOR_SIZE`: Size of the embedding vectors (typically 1024)
- `QDRANT_DISTANCE`: Distance metric for similarity search (Cosine)
- `COHERE_API_KEY`: API key for Cohere
- `COHERE_MODEL`: Cohere model to use (e.g., embed-english-v3.0)
- `COHERE_OUTPUT_DIM`: Output dimension of embeddings (typically 1024)
- `COHERE_RATE_LIMIT_RPM`: Rate limit in requests per minute
- `COHERE_BATCH_SIZE`: Batch size for embedding requests
- `DOCS_PATH`: Path to the docs directory (default: ./docs)
- `CHUNK_SIZE_TOKENS`: Maximum tokens per chunk

## Architecture

The system follows a modular architecture with clear separation of concerns:

- `src/models/`: Data models for chunks, embeddings, and vector points
- `src/services/`: Business logic modules (content ingestion, chunking, embedding, storage)
- `src/cli/`: Command-line interface
- `src/config/`: Configuration management
- `src/utils/`: Utility functions
- `tests/`: Unit and integration tests

## Running Tests

To run the test suite:
```bash
uv run pytest
```

To run specific tests:
```bash
# Unit tests
uv run pytest tests/unit/

# Integration tests
uv run pytest tests/integration/

# Specific test file
uv run pytest tests/unit/test_embedding_service.py
```