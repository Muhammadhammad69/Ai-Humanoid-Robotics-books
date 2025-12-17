---
id: 0002
title: Book Embeddings Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: 001-book-embeddings
branch: 001-book-embeddings
user: claude
command: /sp.plan 001-book-embeddings
labels: ["embedding", "qdrant", "cohere", "book-indexing", "implementation-plan"]
links:
  spec: ../specs/001-book-embeddings/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-book-embeddings/plan.md
  - specs/001-book-embeddings/research.md
  - specs/001-book-embeddings/data-model.md
  - specs/001-book-embeddings/quickstart.md
  - specs/001-book-embeddings/contracts/
tests:
  - specs/001-book-embeddings/research.md
  - specs/001-book-embeddings/data-model.md
---

## Prompt

 001-book-embeddings

Must run this command /sp.plan

Objective:
Create a detailed, implementation-ready plan for embedding book content and storing vectors in Qdrant
using Cohere embeddings, strictly based on the approved specification.

This plan MUST respect the existing backend structure and environment configuration.

---

Mandatory Pre-Planning Steps (NON-NEGOTIABLE):

1. First, read and understand the existing /backend folder:
   - Identify current folder structure
   - Identify existing modules (if any)
   - Identify how configuration is expected to be loaded

2. Confirm that a .env file already exists inside /backend
   and contains all required environment variables, including:

   - QDRANT_URL
   - QDRANT_API_KEY
   - QDRANT_COLLECTION_NAME
   - QDRANT_VECTOR_SIZE
   - QDRANT_DISTANCE

   - COHERE_API_KEY
   - COHERE_MODEL
   - COHERE_OUTPUT_DIM
   - COHERE_RATE_LIMIT_RPM
   - COHERE_BATCH_SIZE

   - DOCS_PATH
   - CHUNK_SIZE_TOKENS

3. Assume that all environment variable values are valid and correct.
   Do NOT redesign configuration unless strictly necessary.

---

Documentation Requirement (CRITICAL):

Before writing the plan, you MUST use Context7 MCP to read official documentation for:

1. Qdrant (official docs only), focusing on:
   - Collections
   - Vector size and distance configuration
   - Payload (metadata)
   - Point insertion
   - Free tier constraints

2. Cohere (official docs only), focusing on:
   - Embedding models (specifically embed-english-v3.0)
   - Input size constraints
   - Output dimensions
   - Rate limits and batching guidance

Context7 MCP must be treated as the source of truth.
Any assumption must be explicitly labeled as an assumption.

---

Planning Scope (ONLY THIS):

This plan covers ONLY:
- Backend preparation using uv (uv init)
- Book content ingestion from /docs
- Chunking strategy execution
- Cohere embedding generation
- Qdrant vector storage

Explicitly OUT OF SCOPE:
- Query-time retrieval
- Selected-text-only logic
- Mixed queries
- LLM answer generation
- Agents
- FastAPI routes
- Chatbot UI
- ChatKit usage

---

Planning Requirements:

The plan MUST include the following sections:

1. Existing Backend Assessment
   - Summary of current /backend structure
   - How uv will be initialized inside /backend
   - How dependencies will be managed with uv

2. Environment Configuration Strategy
   - How .env variables will be loaded and accessed
   - How configuration remains centralized and immutable

3. Data Flow Plan
   - Step-by-step flow:
     /docs → ingestion → chunking → Cohere embeddings → Qdrant
   - Clear responsibility boundaries between modules

4. Chunking Execution Plan
   - How Markdown headings and paragraphs are used
   - How CHUNK_SIZE_TOKENS influences chunk boundaries
   - How semantic integrity is preserved

5. Cohere Embedding Plan
   - How embedding requests are structured
   - How batching is applied using COHERE_BATCH_SIZE
   - How rate limits (COHERE_RATE_LIMIT_RPM) are respected
   - Failure and retry considerations

6. Qdrant Storage Plan
   - Collection usage strategy (single collection)
   - Vector size and distance alignment with env vars
   - Metadata (payload) structure planning
   - ID strategy for deterministic re-indexing

7. Re-indexing & Maintainability Plan
   - How re-running indexing behaves
   - How outdated vectors are replaced or removed
   - Versioning or timestamp approach

8. Risk & Constraint Analysis
   - Qdrant Cloud free tier limits
   - Cohere rate limits
   - Large document handling
   - Future scalability considerations

---

Constraints:

- Cohere is the ONLY embedding provider.
- Qdrant Cloud is the ONLY vector database.
- uv is the ONLY package/environment manager.
- The plan must remain implementation-agnostic.
- No code, no pseudo-code, no prompts.

---

Output Expectations:

- A clear, structured, step-by-step plan
- Decisions justified using official docs via Context7 MCP
- Language suitable for direct transition into /sp.implement

---

Success Criteria:

- An engineer can implement the entire indexing pipeline using this plan alone
- The plan aligns exactly with the approved specification
- No assumptions about retrieval, chat, or agents are introduced

## Response snapshot

Successfully created a comprehensive implementation plan for the book content embedding system. The plan includes:

1. Detailed technical context with Python 3.11, qdrant-client, cohere, and other dependencies
2. Complete project structure with models, services, CLI, config, and utils modules
3. Data flow plan outlining the process from /docs → ingestion → chunking → Cohere embeddings → Qdrant
4. Chunking execution plan with Markdown heading and paragraph boundary handling
5. Cohere embedding plan with proper batching, rate limiting, and error handling
6. Qdrant storage plan with collection strategy, vector alignment, and metadata structure
7. Re-indexing and maintainability plan with deterministic ID strategy
8. Risk analysis covering Qdrant Cloud limits, Cohere rate limits, and scalability

Supporting documentation files were also created:
- research.md: Documentation findings from Qdrant and Cohere APIs
- data-model.md: Entity definitions for chunks, embeddings, and vector points
- quickstart.md: Setup and usage instructions
- contracts/: Empty directory as specified

The plan respects all constraints (Cohere as only embedding provider, Qdrant as only vector database, uv as package manager) and stays within scope (backend-only, no retrieval logic).

## Outcome

- ✅ Impact: Created complete implementation plan for book content embedding pipeline with all required components
- 🧪 Tests: Validated plan against specification requirements and technical constraints
- 📁 Files: Generated plan.md, research.md, data-model.md, quickstart.md, and contracts directory
- 🔁 Next prompts: Ready for /sp.tasks to generate implementation tasks
- 🧠 Reflection: Plan provides clear roadmap for implementation while respecting architectural constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All validation checks passed
- Prompt variant (if applicable): Standard implementation planning
- Next experiment (smallest change to try): Proceed to task generation with /sp.tasks
