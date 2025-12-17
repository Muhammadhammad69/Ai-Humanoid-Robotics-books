from pydantic import BaseModel, Field, validator
from typing import List, Dict, Optional
from datetime import datetime
import uuid


class Message(BaseModel):
    """
    Individual message in a conversation
    """
    role: str = Field(..., pattern=r'^(user|assistant)$', description="Either 'user' or 'assistant'")
    content: str = Field(..., min_length=1, description="Message content")
    timestamp: datetime = Field(..., description="When the message was created")
    message_id: str = Field(..., description="Unique message identifier")

    @validator('role')
    def validate_role(cls, v):
        if v not in ['user', 'assistant']:
            raise ValueError("Role must be either 'user' or 'assistant'")
        return v

    @validator('content')
    def validate_content(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Content must not be empty')
        return v


class SessionState(BaseModel):
    """
    Maintains chat session context and history
    """
    session_id: str = Field(..., description="Unique session identifier")
    history: List[Message] = Field(..., max_items=50, description="Conversation history")
    created_at: datetime = Field(..., description="Session creation timestamp")
    last_accessed: datetime = Field(..., description="Last access timestamp")
    metadata: Optional[Dict] = Field(None, description="Additional session metadata")

    @validator('session_id')
    def validate_session_id(cls, v):
        # Validate that it's a valid string (in a real implementation, we might validate UUID format)
        if not v or len(v.strip()) == 0:
            raise ValueError('Session ID must not be empty')
        return v

    @validator('history')
    def validate_history(cls, v):
        if len(v) > 50:
            raise ValueError('History must not exceed 50 messages')
        return v

    @validator('created_at', 'last_accessed')
    def validate_timestamps(cls, v):
        if v is None:
            raise ValueError('Timestamp must be a valid datetime')
        return v