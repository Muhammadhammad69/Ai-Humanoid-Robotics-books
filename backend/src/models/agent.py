from pydantic import BaseModel, Field, validator
from typing import Optional, List, Dict, Any
from datetime import datetime


class AgentConfig(BaseModel):
    """
    Configuration settings for the agent service
    """
    model_name: str = Field(default="gemini-2.5-flash", description="Name of the LLM model to use")
    temperature: float = Field(default=0.7, ge=0.0, le=1.0, description="Temperature setting for response creativity")
    max_tokens: int = Field(default=1000, gt=0, description="Maximum tokens in the response")
    timeout: int = Field(default=30, gt=0, description="Timeout in seconds for agent processing")

    @validator('model_name')
    def validate_model_name(cls, v):
        allowed_models = ["gemini-2.5-flash","gemini-2.5-flash-lite", "gemini-pro", "gpt-4o", "gpt-4-turbo"]
        if v not in allowed_models:
            raise ValueError(f'Model name must be one of: {allowed_models}')
        return v


class AgentRequest(BaseModel):
    """
    Represents a request to the agent service with context from the RAG pipeline
    """
    query: str = Field(..., description="The original user query", min_length=1)
    context: str = Field(..., description="Retrieved context from Steps 1-5 of the RAG pipeline")
    module_info: Optional[str] = Field(None, description="Module or topic information for context")
    session_id: Optional[str] = Field(None, description="Session identifier for conversation continuity")
    agent_config: Optional[AgentConfig] = Field(None, description="Configuration parameters for the agent")

    class Config:
        schema_extra = {
            "example": {
                "query": "What are the key concepts in chapter 3?",
                "context": "Chapter 3 covers advanced physics concepts including...",
                "module_info": "Advanced Physics",
                "session_id": "session-12345"
            }
        }


class AgentResponse(BaseModel):
    """
    Represents the response from the agent service
    """
    agent_answer: str = Field(..., description="The AI-generated answer from the agent", min_length=1)
    context_used: str = Field(..., description="The context that was provided to the agent")
    processing_time: float = Field(..., gt=0, description="Time taken for agent processing in seconds")
    model_used: str = Field(..., description="The model that generated the response")
    confidence_score: Optional[float] = Field(None, ge=0.0, le=1.0, description="Confidence score of the response")

    class Config:
        schema_extra = {
            "example": {
                "agent_answer": "The key concepts in chapter 3 include quantum mechanics, wave-particle duality, and the uncertainty principle.",
                "context_used": "Chapter 3 covers advanced physics concepts...",
                "processing_time": 1.25,
                "model_used": "gemini-2.5-flash",
                "confidence_score": 0.85
            }
        }