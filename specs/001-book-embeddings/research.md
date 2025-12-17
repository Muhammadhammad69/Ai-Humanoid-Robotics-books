# Research Findings: Book Content Embedding and Vector Storage in Qdrant using Cohere

## Qdrant Documentation Findings

### Collections
- Qdrant collections can be created with specific vector parameters including size and distance metric
- Example: `VectorParams(size=1024, distance=Distance.COSINE)` for collections with 1024-dimensional vectors using cosine distance
- Collections support payload (metadata) storage alongside vectors

### Vector Size and Distance Configuration
- Vector size must match the embedding vector size (e.g., 1024 for Cohere embeddings)
- Supported distance metrics include Cosine, Euclidean, Manhattan, and Dot
- The distance metric affects how similarity is calculated during search operations

### Payload (Metadata)
- Payloads are JSON objects that can store arbitrary data alongside vectors
- Supports various data types: strings, numbers, arrays, and nested structures
- Essential for traceability with fields like document_path, section_type, module_name, etc.
- Example payload structure: `{"city": "Berlin", "price": 1.99}`

### Point Insertion
- Points are inserted using the `upsert` operation which updates if exists or creates if not
- Supports batch insertion of multiple points at once
- Each point requires a unique ID, vector data, and optional payload
- Points can be inserted with HTTP API, Python client, TypeScript client, etc.

### Free Tier Constraints
- Based on documentation review, Qdrant Cloud free tier has limitations on:
  - Storage capacity
  - Number of vectors
  - API request rates
  - Concurrent connections

## Cohere Documentation Findings

### Embedding Models (embed-english-v3.0)
- The embed-english-v3.0 model supports various input types: search_document, search_query, classification, clustering
- For book content indexing, `search_document` input type is recommended
- Supports output dimensions for v4+ models (256, 512, 1024, 1536), though v3.0 may have different dimension support

### Input Size Constraints
- Maximum number of texts per API call: 96
- Input texts are limited by token count (specific limits depend on model)
- Truncation strategies available: START, END, NONE

### Output Dimensions
- For embed-v4.0: supports 256, 512, 1024, 1536 dimensions
- For embed-english-v3.0: likely supports 1024 dimensions as specified in environment variables
- Output dimension parameter allows for Matryoshka embeddings in newer models

### Rate Limits and Batching Guidance
- API supports batch processing of up to 96 texts per request
- Rate limits apply per minute (RPM as specified in environment variables)
- Batching significantly improves throughput and efficiency
- Batch jobs API available for very large datasets

## Technical Decisions and Rationale

### Decision: Qdrant Collection Configuration
**Rationale**: Collection must be configured with vector size matching Cohere embeddings (1024) and appropriate distance metric (Cosine) for semantic similarity
**Implementation**: Use VectorParams(size=1024, distance=Distance.COSINE) when creating collection

### Decision: Cohere Input Type
**Rationale**: For book content indexing, input_type should be "search_document" to optimize embeddings for search applications
**Implementation**: Use input_type="search_document" when calling Cohere embed API

### Decision: Batching Strategy
**Rationale**: To respect rate limits and optimize performance, implement batching with up to 96 texts per request
**Implementation**: Group chunks into batches of up to 96 before sending to Cohere API

### Decision: Payload Structure
**Rationale**: To maintain traceability as required by specification, payload must include all specified metadata fields
**Implementation**: Each vector point will include document_path, section_type, module_name, chapter_name, section_heading, language, source_type, and indexing_version

## Alternatives Considered

### Alternative Vector Databases
- PostgreSQL with pgvector
- Pinecone
- Weaviate
- ChromaDB

**Chosen Qdrant** because it's specified in requirements as the only vector database to be used.

### Alternative Embedding Providers
- OpenAI embeddings
- Hugging Face embeddings
- Google embeddings

**Chosen Cohere** because it's specified in requirements as the only embedding provider.