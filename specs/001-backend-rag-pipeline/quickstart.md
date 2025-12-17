# Quickstart Guide: Backend RAG Query Pipeline

## Overview
This guide provides a quick setup and usage guide for the backend RAG query pipeline. The system processes user queries through 5 steps: Query Intake, Query Embedding, Vector Retrieval, Context Filtering & Ranking, and Context Assembly.

## Prerequisites
- Python 3.11+
- Backend environment properly configured with `.env` file
- Access to Qdrant vector database
- Cohere API access

## Environment Setup
1. Ensure your `.env` file contains the required configuration:
   ```
   QDRANT_URL="your-qdrant-url"
   QDRANT_API_KEY="your-api-key"
   QDRANT_COLLECTION_NAME="your-collection"
   COHERE_API_KEY="your-cohere-api-key"
   COHERE_MODEL="embed-english-v3.0"
   ```

2. Install dependencies (if not already installed):
   ```bash
   cd backend
   uv sync  # or your preferred Python environment setup
   ```

## Running the Service
1. Start the FastAPI server:
   ```bash
   cd backend
   python -m uvicorn src.api.main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`

## API Endpoints
- `POST /query` - Submit a query and receive processed context
- `POST /session/create` - Create a new chat session
- `GET /session/{session_id}` - Retrieve session details

## Example Query Request
```json
{
  "query": "What are the key concepts in quantum computing?",
  "module": "physics",
  "language": "en",
  "session_id": "session-123"
}
```

## Example Response
```json
{
  "context": "Quantum computing is a type of computation that harnesses the physical nature of quantum mechanics...",
  "retrieved_chunks": [
    {
      "content": "Quantum computing uses quantum bits (qubits) which can exist in superposition...",
      "relevance_score": 0.92,
      "metadata": {"module": "physics", "language": "en"},
      "source_document": "quantum_computing_basics.md"
    }
  ],
  "session_id": "session-123",
  "processing_time": 1.24,
  "query_id": "query-456"
}
```

## Skills Integration
- `fastapi-chat-orchestration` handles API endpoint design and request/response handling
- `chatkit-python-integration` manages session state and message history

## Testing
Run the tests to verify functionality:
```bash
cd backend
pytest tests/unit/ -v
pytest tests/integration/ -v
```