# Data Model: Agent Integration for RAG Pipeline

## Overview
This document defines the data models required for the agent integration in the RAG pipeline, extending the existing query processing models.

## Entity: AgentRequest
**Purpose**: Represents a request to the agent service with context from the RAG pipeline

**Fields**:
- `query` (str): The original user query
- `context` (str): Retrieved context from Steps 1-5 of the RAG pipeline
- `module_info` (str, optional): Module or topic information for context
- `session_id` (str, optional): Session identifier for conversation continuity
- `agent_config` (dict, optional): Configuration parameters for the agent

**Relationships**: Extends from existing QueryRequest model

## Entity: AgentResponse
**Purpose**: Represents the response from the agent service

**Fields**:
- `agent_answer` (str): The AI-generated answer from the agent
- `context_used` (str): The context that was provided to the agent
- `processing_time` (float): Time taken for agent processing
- `model_used` (str): The model that generated the response
- `confidence_score` (float, optional): Confidence score of the response

**Relationships**: Extends from existing QueryResponse model

## Entity: AgentConfig
**Purpose**: Configuration settings for the agent service

**Fields**:
- `model_name` (str): Name of the LLM model to use (e.g., "gemini-2.5-flash")
- `temperature` (float): Temperature setting for response creativity
- `max_tokens` (int): Maximum tokens in the response
- `timeout` (int): Timeout in seconds for agent processing

## Entity: AgentSession
**Purpose**: Manages agent session state for conversation continuity

**Fields**:
- `session_id` (str): Unique identifier for the session
- `history` (list): Conversation history items
- `created_at` (datetime): Session creation timestamp
- `last_accessed` (datetime): Last access timestamp

## Validation Rules

1. **AgentRequest Validation**:
   - Query must not be empty
   - Context must not exceed maximum token limit
   - Session ID must follow UUID format if provided

2. **AgentResponse Validation**:
   - Agent answer must not be empty
   - Processing time must be positive
   - Confidence score must be between 0 and 1 if provided

3. **AgentConfig Validation**:
   - Model name must be in allowed list
   - Temperature must be between 0 and 1
   - Max tokens must be positive
   - Timeout must be positive

## State Transitions

1. **AgentRequest Processing**:
   - PENDING → PROCESSING → COMPLETED/FAILED
   - Each state transition includes timestamp and status details

2. **AgentSession Lifecycle**:
   - CREATED → ACTIVE → INACTIVE/EXPIRED
   - Sessions automatically expire after configurable time period