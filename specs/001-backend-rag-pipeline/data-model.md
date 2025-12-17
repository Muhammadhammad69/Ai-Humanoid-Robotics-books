# Data Model: Backend RAG Query Pipeline

## Core Entities

### QueryRequest
**Description**: Represents a user query request with optional metadata

**Fields**:
- `query`: string (required) - The natural language query from the user
- `module`: string (optional) - Optional module filter for targeted search
- `language`: string (optional) - Language preference for results
- `session_id`: string (optional) - Chat session identifier for context
- `metadata`: dict (optional) - Additional query metadata

**Validation**:
- Query must be non-empty and not exceed 1000 characters
- Module and language must be from predefined lists if provided
- Session ID must follow UUID format if provided

### QueryResponse
**Description**: Response structure containing processed context for LLM consumption

**Fields**:
- `context`: string (required) - Assembled context ready for LLM
- `retrieved_chunks`: list[RetrievedChunk] (required) - Detailed retrieved chunks
- `session_id`: string (required) - Session identifier
- `processing_time`: float (required) - Time taken for processing in seconds
- `query_id`: string (required) - Unique identifier for this query

**Validation**:
- Context must not be empty
- Retrieved chunks must contain at least one item
- Processing time must be positive

### RetrievedChunk
**Description**: Individual content chunk retrieved from vector database

**Fields**:
- `content`: string (required) - The text content of the chunk
- `relevance_score`: float (required) - Similarity score (0.0 to 1.0)
- `metadata`: dict (required) - Associated metadata (module, language, etc.)
- `chunk_id`: string (required) - Unique identifier for the chunk
- `source_document`: string (required) - Reference to source document

**Validation**:
- Content must not be empty
- Relevance score must be between 0.0 and 1.0
- Metadata must be a valid dictionary

### SessionState
**Description**: Maintains chat session context and history

**Fields**:
- `session_id`: string (required) - Unique session identifier
- `history`: list[Message] (required) - Conversation history
- `created_at`: datetime (required) - Session creation timestamp
- `last_accessed`: datetime (required) - Last access timestamp
- `metadata`: dict (optional) - Additional session metadata

**Validation**:
- Session ID must be unique
- History must not exceed 50 messages
- Created and last accessed must be valid timestamps

### Message
**Description**: Individual message in a conversation

**Fields**:
- `role`: string (required) - Either "user" or "assistant"
- `content`: string (required) - Message content
- `timestamp`: datetime (required) - When the message was created
- `message_id`: string (required) - Unique message identifier

**Validation**:
- Role must be either "user" or "assistant"
- Content must not be empty
- Timestamp must be a valid datetime

## State Transitions

### Query Processing Flow
1. **QueryReceived**: QueryRequest validated and accepted
2. **EmbeddingGenerated**: Query converted to vector embedding
3. **VectorRetrieved**: Relevant chunks retrieved from Qdrant
4. **ContextFiltered**: Chunks filtered by relevance and token budget
5. **ContextAssembled**: Final context prepared for LLM
6. **ResponseReady**: QueryResponse created and returned

### Session Management Flow
1. **SessionCreated**: New session initialized with empty history
2. **MessageAdded**: User message added to session history
3. **ContextRetrieved**: Relevant context retrieved for response
4. **ResponseAdded**: Assistant response added to session history
5. **SessionUpdated**: Session state maintained for next interaction

## Relationships

- One `SessionState` contains many `Message` objects
- One `QueryRequest` produces one `QueryResponse`
- One `QueryResponse` contains many `RetrievedChunk` objects
- Many `RetrievedChunk` objects contribute to one assembled `context`