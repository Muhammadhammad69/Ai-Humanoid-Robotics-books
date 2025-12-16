# FastAPI Backend for ChatKit

This example shows how to set up a FastAPI backend to work with ChatKit frontend.

## Basic Session Creation Endpoint

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from openai import OpenAI
import os
import logging

app = FastAPI()
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChatSessionRequest(BaseModel):
    user_id: str = None
    thread_id: str = None

@app.post("/api/chatkit/session")
async def create_chatkit_session(request: ChatSessionRequest = None):
    """
    Create a new ChatKit session and return the client secret.
    This endpoint is called by the frontend to initialize a ChatKit session.
    """
    try:
        # Create a new session using OpenAI's ChatKit
        session_data = {
            # Add any session-specific configuration here
        }

        if request and request.thread_id:
            # If a thread ID is provided, continue that thread
            session_data["thread_id"] = request.thread_id

        session = client.chatkit.sessions.create(**session_data)

        logger.info(f"Created ChatKit session: {session.id}")

        return {"client_secret": session.client_secret}
    except Exception as e:
        logger.error(f"Error creating ChatKit session: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to create chat session")

@app.post("/api/chatkit/refresh")
async def refresh_chatkit_session(token: str):
    """
    Refresh an existing ChatKit session token.
    """
    try:
        # Implement token refresh logic here
        # This might involve validating the existing token and issuing a new one
        refreshed_session = client.chatkit.sessions.refresh(
            client_secret=token
        )

        return {"client_secret": refreshed_session.client_secret}
    except Exception as e:
        logger.error(f"Error refreshing ChatKit session: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to refresh session")

@app.post("/api/analytics/message")
async def track_message_event(data: dict):
    """
    Track message events for analytics.
    """
    try:
        logger.info(f"Message event: {data}")
        # Implement analytics tracking logic here
        # This could involve storing in a database or sending to an analytics service
        return {"status": "tracked"}
    except Exception as e:
        logger.error(f"Error tracking message event: {str(e)}")
        # Don't fail the chat if analytics fails
        return {"status": "failed"}

@app.post("/api/analytics/thread-change")
async def track_thread_change(data: dict):
    """
    Track thread change events for analytics.
    """
    try:
        logger.info(f"Thread change: {data}")
        # Implement analytics tracking logic here
        return {"status": "tracked"}
    except Exception as e:
        logger.error(f"Error tracking thread change: {str(e)}")
        # Don't fail the chat if analytics fails
        return {"status": "failed"}

@app.post("/api/errors")
async def log_error(data: dict):
    """
    Log errors from the frontend for debugging.
    """
    try:
        logger.error(f"Frontend error: {data}")
        # Implement error logging logic here
        # This could involve storing in a database or sending to an error tracking service
        return {"status": "logged"}
    except Exception as e:
        logger.error(f"Error logging frontend error: {str(e)}")
        return {"status": "failed"}

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the backend is running.
    """
    return {"status": "healthy"}
```

## Environment Setup

Create a `.env` file:

```
OPENAI_API_KEY=your_openai_api_key_here
```

## Running the Backend

```bash
pip install fastapi uvicorn openai python-dotenv
uvicorn main:app --reload
```

## Frontend Integration

The frontend will call these endpoints:

1. `/api/chatkit/session` - To get a client secret for initializing ChatKit
2. `/api/analytics/message` - To track message events
3. `/api/analytics/thread-change` - To track thread changes
4. `/api/errors` - To log frontend errors

## Security Considerations

1. Always validate and sanitize input data
2. Use proper authentication for user-specific operations
3. Implement rate limiting to prevent abuse
4. Store API keys securely using environment variables
5. Use HTTPS in production
6. Validate the domain in OpenAI's dashboard for custom endpoints

## Error Handling

The backend includes proper error handling for:
- API key issues
- Network failures
- Invalid requests
- Session creation failures

Each endpoint returns appropriate HTTP status codes and error messages that the frontend can handle gracefully.