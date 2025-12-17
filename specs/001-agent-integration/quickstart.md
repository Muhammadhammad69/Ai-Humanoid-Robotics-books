# Quickstart: Agent Integration for RAG Pipeline

## Overview
This guide provides quick setup instructions for the agent integration feature that adds Steps 6-7 to the RAG pipeline: agent-based answer generation and response delivery.

## Prerequisites
- Python 3.11+
- Backend service running with Steps 1-5 of RAG pipeline
- Valid GEMINI_API_KEY in environment variables
- Access to Gemini API via OpenAI-compatible endpoint

## Environment Setup
1. Ensure `.env` file contains:
   ```
   GEMINI_API_KEY="your-gemini-api-key"
   GEMINI_BASE_URL="https://generativelanguage.googleapis.com/v1beta/openai/"
   GEMINI_MODEL="gemini-2.5-flash"
   ```

2. Verify dependencies are installed:
   ```bash
   cd backend
   uv pip install openai-agents openai
   ```

## Key Components

### 1. Agent Service (`src/services/agent_service.py`)
- Handles agent creation and execution
- Processes context from RAG pipeline Steps 1-5
- Generates AI responses using configured LLM

### 2. Query Processing Integration (`src/services/query_processor.py`)
- Updated to call agent service after Step 5
- Formats context appropriately for agent consumption
- Incorporates agent response into final output

### 3. API Routes (`src/api/routes/query.py`)
- Modified to include agent response in query endpoint
- Maintains backward compatibility with existing response format

## Usage Example

### Standard Query with Agent Response
```python
# The existing query endpoint now includes agent response
response = await query_processor.process_query_with_agent(query_request)
# Returns both context (from Steps 1-5) and agent_answer (from Steps 6-7)
```

### Direct Agent Service Usage
```python
from src.services.agent_service import AgentService

agent_service = AgentService()
agent_response = await agent_service.generate_answer(
    query="What is the main concept of chapter 3?",
    context="Retrieved context from RAG pipeline...",
    module_info="Module 2: Advanced Concepts"
)
```

## Error Handling
- API failures are caught and logged
- Fallback responses provided when agent unavailable
- Proper HTTP status codes returned to frontend

## Testing
Run the following to verify the integration:
```bash
cd backend
python -m pytest tests/agent_tests.py
```

## Next Steps
1. Implement the agent service components as defined in tasks.md
2. Integrate with existing query processing pipeline
3. Test end-to-end functionality
4. Monitor performance metrics