# ChatKit Integration Guide

This guide covers how to integrate FastAPI with ChatKit services for chat orchestration.

## ChatKit Python Backend Setup

### Installation
```bash
pip install openai  # For ChatKit backend
```

### Basic Session Creation Endpoint
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from openai import OpenAI
import os

app = FastAPI()
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

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
        session_data = {}

        if request and request.thread_id:
            # If a thread ID is provided, continue that thread
            session_data["thread_id"] = request.thread_id

        session = client.chatkit.sessions.create(**session_data)

        return {"client_secret": session.client_secret}
    except Exception as e:
        raise HTTPException(status_code=500, detail="Failed to create chat session")
```

### Session Refresh Endpoint
```python
@app.post("/api/chatkit/refresh")
async def refresh_chatkit_session(token: str):
    """
    Refresh an existing ChatKit session token.
    """
    try:
        refreshed_session = client.chatkit.sessions.refresh(
            client_secret=token
        )

        return {"client_secret": refreshed_session.client_secret}
    except Exception as e:
        raise HTTPException(status_code=500, detail="Failed to refresh session")
```

## Chat Message Processing

### Chat Message Model
```python
from pydantic import BaseModel, Field
from typing import Optional, List
import datetime

class ChatMessage(BaseModel):
    content: str = Field(..., min_length=1, max_length=2000)
    role: str = Field(..., pattern="^(user|assistant|system)$")
    timestamp: Optional[str] = None
    metadata: Optional[dict] = None

class ChatRequest(BaseModel):
    messages: List[ChatMessage]
    user_id: str
    thread_id: Optional[str] = None
    model: str = "gpt-4-turbo"
```

### Chat Processing Endpoint
```python
from fastapi import BackgroundTasks
import logging

logger = logging.getLogger(__name__)

@app.post("/api/chat")
async def process_chat(request: ChatRequest, background_tasks: BackgroundTasks):
    """
    Process a chat request and return the AI response.
    """
    try:
        # Prepare messages for OpenAI API
        openai_messages = [
            {"role": msg.role, "content": msg.content}
            for msg in request.messages
        ]

        # Call OpenAI API
        response = client.chat.completions.create(
            model=request.model,
            messages=openai_messages
        )

        ai_message = response.choices[0].message.content

        # Schedule background tasks
        background_tasks.add_task(
            log_chat_interaction,
            request.user_id,
            request.messages[-1].content,
            ai_message
        )

        return {
            "response": ai_message,
            "thread_id": response.id if hasattr(response, 'id') else request.thread_id,
            "timestamp": datetime.datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Chat processing error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process chat message")

def log_chat_interaction(user_id: str, user_message: str, ai_response: str):
    """
    Background task to log chat interactions for analytics.
    """
    # Implementation for logging to database or analytics service
    print(f"Logging chat: User {user_id} - {user_message} -> {ai_response}")
```

## WebSocket Integration for Real-time Updates

### Chat WebSocket Endpoint
```python
from fastapi import WebSocket, WebSocketDisconnect
from typing import Dict, List
import json

class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections[client_id] = websocket

    def disconnect(self, client_id: str):
        if client_id in self.active_connections:
            del self.active_connections[client_id]

    async def send_personal_message(self, message: str, client_id: str):
        websocket = self.active_connections.get(client_id)
        if websocket:
            await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections.values():
            await connection.send_text(message)

manager = ConnectionManager()

@app.websocket("/ws/chat/{client_id}")
async def websocket_chat_endpoint(websocket: WebSocket, client_id: str):
    await manager.connect(websocket, client_id)
    try:
        while True:
            data = await websocket.receive_text()
            message_data = json.loads(data)

            # Process the message (could integrate with ChatKit here)
            response = {"type": "echo", "content": f"Server received: {message_data}"}
            await manager.send_personal_message(json.dumps(response), client_id)
    except WebSocketDisconnect:
        manager.disconnect(client_id)
        await manager.broadcast(f"Client {client_id} left the chat")
```

## Authentication for Chat Services

### JWT Token Authentication
```python
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer
import jwt
from datetime import datetime, timedelta

security = HTTPBearer()

def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, os.environ["SECRET_KEY"], algorithm="HS256")
    return encoded_jwt

async def verify_token(credentials: HTTPBearer = Depends(security)):
    try:
        token = credentials.credentials
        payload = jwt.decode(token, os.environ["SECRET_KEY"], algorithms=["HS256"])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )
        return user_id
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )

@app.post("/token")
async def login_for_access_token(username: str, password: str):
    # Verify credentials (implement your own logic)
    user = authenticate_user(username, password)  # Your authentication logic
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password"
        )
    access_token_expires = timedelta(minutes=30)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@app.post("/api/chat/protected")
async def protected_chat_endpoint(
    request: ChatRequest,
    user_id: str = Depends(verify_token)
):
    # Process chat with authenticated user
    request.user_id = user_id
    return await process_chat(request)
```

## Error Handling and Monitoring

### Custom Exception Handlers
```python
from fastapi import Request
from fastapi.responses import JSONResponse

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    return JSONResponse(
        status_code=500,
        content={
            "message": "Internal server error",
            "detail": str(exc) if os.environ.get("DEBUG") else "An error occurred"
        }
    )

@app.exception_handler(WebSocketDisconnect)
async def websocket_disconnect_handler(websocket: WebSocket, exc: WebSocketDisconnect):
    print(f"WebSocket disconnected: {exc.code}")
```

## Health Checks and Monitoring

### Health Check Endpoint
```python
@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the service is running.
    """
    return {
        "status": "healthy",
        "timestamp": datetime.datetime.now().isoformat(),
        "service": "chat-orchestration-api"
    }

@app.get("/metrics")
async def get_metrics():
    """
    Metrics endpoint for monitoring.
    """
    return {
        "active_connections": len(manager.active_connections),
        "total_messages_processed": 0,  # Implement your counter
        "uptime": "implement uptime tracking"
    }
```

## Environment Configuration

### Required Environment Variables
```bash
# OpenAI API key for ChatKit
OPENAI_API_KEY=your_openai_api_key_here

# JWT secret for authentication
SECRET_KEY=your_jwt_secret_key_here

# Database connection (if using)
DATABASE_URL=postgresql://user:password@localhost/dbname

# Redis for session management (optional)
REDIS_URL=redis://localhost:6379

# Debug mode
DEBUG=false
```

### Settings Management with Pydantic
```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    openai_api_key: str
    secret_key: str
    database_url: str = "sqlite:///./chat.db"
    debug: bool = False
    redis_url: str = "redis://localhost:6379"

    class Config:
        env_file = ".env"

settings = Settings()
```

## Deployment Considerations

### ASGI Server Configuration (Uvicorn)
```python
# main.py
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        workers=4,  # Adjust based on your needs
        reload=True,  # Set to False in production
        log_level="info"
    )
```

### Docker Configuration
```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

This integration guide provides the foundation for building a robust chat orchestration service using FastAPI and ChatKit.