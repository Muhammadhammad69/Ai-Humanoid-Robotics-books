---
name: fastapi-chat-orchestration
description: A skill for building FastAPI-based chat orchestration services, handling installation, setup, endpoint creation, request/response handling, async support, background tasks, WebSocket integration, dependency injection, authentication, and integration with chatkit-python backend and chatkit-js frontend.
---

# FastAPI Chat Orchestration Skill

## Purpose

This skill provides comprehensive guidance for building FastAPI-based chat orchestration services. It covers installation, setup, endpoint creation, request/response handling, async support, background tasks, WebSocket integration, dependency injection, authentication, and integration with chatkit-python backend and chatkit-js frontend.

## When to Use This Skill

Use this skill when you need to:
- Set up a FastAPI backend for chat applications
- Create chat orchestration endpoints with proper async handling
- Implement WebSocket connections for real-time chat
- Handle authentication and security for chat services
- Integrate with ChatKit frontend and backend services
- Implement background tasks for message processing
- Build streaming responses for chat interactions

## Dependencies

- `fastapi` - The main web framework
- `uvicorn` - ASGI server for running the application
- `pydantic` - Data validation and settings management
- `websockets` - For WebSocket support (if using)
- `python-multipart` - For form data handling
- `chatkit-python` - For backend integration (optional)
- `sqlalchemy` - For database integration (optional)

## Inputs

- API configuration (dependencies, middleware, security)
- Request models (Pydantic models for validation)
- Authentication schemes (OAuth2, JWT, Basic Auth)
- Database connection settings
- WebSocket connection parameters

## Outputs

- Fully functional FastAPI application
- Orchestration endpoints for chat services
- WebSocket endpoints for real-time communication
- Authentication and authorization systems
- Background task processing
- Streaming responses for chat messages

## Internal Workflow

### 1. Installation and Setup
- Install FastAPI with standard dependencies
- Set up virtual environment and requirements
- Configure development server (Uvicorn)

### 2. Endpoint Creation and Routing
- Define API routes with proper HTTP methods
- Create request/response models using Pydantic
- Implement path and query parameter validation
- Add request/response examples and documentation

### 3. Request/Response Handling
- Create Pydantic models for request validation
- Implement response models for consistent output
- Handle different content types and encodings
- Add custom response classes when needed

### 4. Async Support and Concurrency
- Use async/await for I/O-bound operations
- Implement concurrent request handling
- Use async generators for streaming responses
- Leverage FastAPI's built-in async capabilities

### 5. Background Tasks
- Implement background task processing with BackgroundTasks
- Handle non-blocking operations (email sending, logging)
- Manage resource cleanup in background tasks
- Chain multiple background operations

### 6. WebSocket/Streaming Support
- Create WebSocket endpoints for real-time communication
- Handle connection lifecycle and error management
- Implement message broadcasting to multiple clients
- Add connection authentication and authorization

### 7. Dependency Injection
- Create reusable dependency functions
- Implement database connection management
- Handle authentication dependencies
- Chain dependencies for complex validation

### 8. Authentication and Security
- Implement OAuth2 with password flow
- Add JWT token generation and validation
- Implement HTTP Basic Authentication
- Add security headers and CORS configuration

### 9. Integration with Chat Services
- Connect to ChatKit backend services
- Handle session management and tokens
- Implement message routing and orchestration
- Add error handling for external service calls

## Example Endpoints

### Basic Chat Endpoint
```python
from typing import Union
from fastapi import FastAPI, Query
from pydantic import BaseModel

app = FastAPI()

class ChatRequest(BaseModel):
    message: str
    user_id: str
    session_id: Union[str, None] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    timestamp: str

@app.post("/chat/", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    # Process the chat request
    response = {
        "response": f"Echo: {request.message}",
        "session_id": request.session_id or "new_session",
        "timestamp": "2023-01-01T00:00:00Z"
    }
    return response
```

### WebSocket Chat Endpoint
```python
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import Dict, List

app = FastAPI()

class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            await manager.broadcast(f"Client {client_id}: {data}")
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        await manager.broadcast(f"Client {client_id} left the chat")
```

### Background Task Example
```python
from fastapi import BackgroundTasks, FastAPI
import asyncio

app = FastAPI()

def send_notification(email: str, message: str = ""):
    with open("log.txt", "a") as log:
        log.write(f"Notification for {email}: {message}\\n")

@app.post("/send-notification/{email}")
async def send_notification_endpoint(
    email: str,
    background_tasks: BackgroundTasks
):
    background_tasks.add_task(send_notification, email, "Chat message received")
    return {"message": "Notification scheduled"}
```

### Authentication Example
```python
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel

app = FastAPI()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

class User(BaseModel):
    username: str
    email: Union[str, None] = None

async def get_current_user(token: str = Depends(oauth2_scheme)):
    # Validate token and return user
    user = fake_decode_token(token)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user

@app.get("/users/me", response_model=User)
async def read_users_me(current_user: User = Depends(get_current_user)):
    return current_user
```

### Streaming Response Example
```python
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import asyncio

app = FastAPI()

async def generate_chat_stream():
    messages = ["Hello", "How", "can", "I", "help", "you", "today?"]
    for message in messages:
        yield f"data: {message}\\n\\n"
        await asyncio.sleep(0.5)

@app.get("/stream-chat")
async def stream_chat():
    return StreamingResponse(generate_chat_stream(), media_type="text/event-stream")
```

### Dependency Injection Example
```python
from fastapi import Depends, FastAPI

app = FastAPI()

# Database dependency
async def get_db():
    db = {"connection": "active"}
    try:
        yield db
    finally:
        db["connection"] = "closed"

# Common parameters dependency
async def common_parameters(
    q: Union[str, None] = None,
    skip: int = 0,
    limit: int = 100
):
    return {"q": q, "skip": skip, "limit": limit}

@app.get("/items/")
async def read_items(
    commons: dict = Depends(common_parameters),
    db: dict = Depends(get_db)
):
    return {"params": commons, "database": db}
```

## Error Handling

- Implement proper HTTP status codes for different error types
- Use FastAPI's automatic request validation with Pydantic
- Handle WebSocket disconnections gracefully
- Implement custom exception handlers
- Add logging for debugging and monitoring
- Provide meaningful error messages to clients
- Implement retry mechanisms for external service calls

## Notes and Best Practices

- Always use async/await for I/O-bound operations
- Implement proper request validation with Pydantic models
- Use dependency injection for reusable components
- Handle authentication with FastAPI's security features
- Use BackgroundTasks for non-blocking operations
- Implement proper error handling and logging
- Use environment variables for configuration
- Add CORS middleware for web client integration
- Implement rate limiting for production deployments
- Use database connection pooling for efficiency
- Implement proper cleanup in background tasks
- Add health check endpoints for monitoring
- Use middleware for cross-cutting concerns (logging, auth, etc.)
- Implement proper session management for chat applications
- Add security headers and implement security best practices