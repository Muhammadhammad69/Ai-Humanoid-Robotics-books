# API Contracts: Backend RAG Query Pipeline

## Query Processing Endpoint

### POST /query
Process a user query through the RAG pipeline and return structured context.

**Request**:
```json
{
  "query": "What are the key concepts in quantum computing?",
  "module": "physics",
  "language": "en",
  "session_id": "session-123"
}
```

**Response (200 OK)**:
```json
{
  "context": "Quantum computing is a type of computation that harnesses the physical nature of quantum mechanics...",
  "retrieved_chunks": [
    {
      "content": "Quantum computing uses quantum bits (qubits) which can exist in superposition...",
      "relevance_score": 0.92,
      "metadata": {
        "module": "physics",
        "language": "en"
      },
      "source_document": "quantum_computing_basics.md",
      "chunk_id": "chunk-789"
    }
  ],
  "session_id": "session-123",
  "processing_time": 1.24,
  "query_id": "query-456"
}
```

**Errors**:
- 400: Invalid request format or missing required fields
- 422: Query validation failed (e.g., empty query, invalid format)
- 500: Internal server error during processing

## Session Management Endpoints

### POST /session/create
Create a new chat session with optional initial metadata.

**Request**:
```json
{
  "metadata": {
    "user_id": "user-123",
    "preferences": {
      "language": "en",
      "modules": ["physics", "math"]
    }
  }
}
```

**Response (200 OK)**:
```json
{
  "session_id": "session-456",
  "created_at": "2025-12-17T10:30:00Z",
  "metadata": {
    "user_id": "user-123",
    "preferences": {
      "language": "en",
      "modules": ["physics", "math"]
    }
  }
}
```

### GET /session/{session_id}
Retrieve session details and history.

**Response (200 OK)**:
```json
{
  "session_id": "session-456",
  "created_at": "2025-12-17T10:30:00Z",
  "last_accessed": "2025-12-17T10:35:00Z",
  "history": [
    {
      "role": "user",
      "content": "What are the key concepts in quantum computing?",
      "timestamp": "2025-12-17T10:32:00Z",
      "message_id": "msg-001"
    },
    {
      "role": "assistant",
      "content": "Quantum computing uses quantum bits (qubits) which can exist in superposition...",
      "timestamp": "2025-12-17T10:32:05Z",
      "message_id": "msg-002"
    }
  ],
  "metadata": {
    "user_id": "user-123",
    "preferences": {
      "language": "en",
      "modules": ["physics", "math"]
    }
  }
}
```

## Data Validation Contracts

### QueryRequest Schema
```json
{
  "type": "object",
  "properties": {
    "query": {
      "type": "string",
      "minLength": 1,
      "maxLength": 1000
    },
    "module": {
      "type": "string",
      "pattern": "^[a-zA-Z0-9_-]+$"
    },
    "language": {
      "type": "string",
      "pattern": "^[a-z]{2}$"
    },
    "session_id": {
      "type": "string",
      "pattern": "^[a-zA-Z0-9-]+$"
    }
  },
  "required": ["query"]
}
```

### QueryResponse Schema
```json
{
  "type": "object",
  "properties": {
    "context": {
      "type": "string",
      "minLength": 1
    },
    "retrieved_chunks": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "content": {"type": "string"},
          "relevance_score": {"type": "number", "minimum": 0.0, "maximum": 1.0},
          "metadata": {"type": "object"},
          "source_document": {"type": "string"},
          "chunk_id": {"type": "string"}
        },
        "required": ["content", "relevance_score", "metadata", "source_document", "chunk_id"]
      }
    },
    "session_id": {"type": "string"},
    "processing_time": {"type": "number"},
    "query_id": {"type": "string"}
  },
  "required": ["context", "retrieved_chunks", "session_id", "processing_time", "query_id"]
}
```

## Performance Contracts

### Response Time SLAs
- Query processing: < 5 seconds for 95% of requests
- Session creation: < 100ms for 95% of requests
- Session retrieval: < 100ms for 95% of requests

### Throughput Capacity
- Support up to 100 concurrent queries
- Handle 1000 queries per minute sustained load