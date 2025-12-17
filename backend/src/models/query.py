from pydantic import BaseModel, Field, validator
from typing import List, Dict, Optional
from datetime import datetime
import uuid


class RetrievedChunk(BaseModel):
    """
    Individual content chunk retrieved from vector database
    """
    content: str = Field(..., description="The text content of the chunk")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score (0.0 to 1.0)")
    metadata: Dict = Field(..., description="Associated metadata (module, language, etc.)")
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    source_document: str = Field(..., description="Reference to source document")

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Relevance score must be between 0.0 and 1.0')
        return v


class QueryRequest(BaseModel):
    """
    Represents a user query request with optional metadata
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The natural language query from the user")
    module: Optional[str] = Field(None, pattern=r'^[a-zA-Z0-9_-]+$', description="Optional module filter for targeted search")
    language: Optional[str] = Field(None, pattern=r'^[a-z]{2}$', description="Language preference for results")
    session_id: Optional[str] = Field(None, pattern=r'^[a-zA-Z0-9-]+$', description="Chat session identifier for context")
    metadata: Optional[Dict] = Field(None, description="Additional query metadata")

    @validator('query')
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query must be non-empty')
        if len(v) > 1000:
            raise ValueError('Query must not exceed 1000 characters')
        return v


class QueryResponse(BaseModel):
    """
    Response structure containing processed context for LLM consumption
    """
    context: str = Field(..., min_length=1, description="Assembled context ready for LLM")
    retrieved_chunks: List[RetrievedChunk] = Field(..., description="Detailed retrieved chunks")
    session_id: str = Field(..., description="Session identifier")
    processing_time: float = Field(..., gt=0, description="Time taken for processing in seconds")
    query_id: str = Field(..., description="Unique identifier for this query")

    @validator('context')
    def validate_context(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Context must not be empty')
        return v

    @validator('retrieved_chunks')
    def validate_retrieved_chunks(cls, v):
        if not v or len(v) == 0:
            raise ValueError('Retrieved chunks must contain at least one item')
        return v

    @validator('processing_time')
    def validate_processing_time(cls, v):
        if v <= 0:
            raise ValueError('Processing time must be positive')
        return v