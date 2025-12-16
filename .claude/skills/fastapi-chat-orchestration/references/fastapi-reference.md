# FastAPI Reference Guide

## Core Components

### FastAPI Application
The main application instance that handles routing and configuration.

```python
from fastapi import FastAPI

app = FastAPI(
    title="Chat Orchestration API",
    description="API for managing chat sessions and messages",
    version="1.0.0"
)
```

### Path Operations
Define endpoints with HTTP methods and path parameters.

```python
@app.get("/")
@app.post("/items")
@app.put("/items/{item_id}")
@app.delete("/items/{item_id}")
```

## Request and Response Models

### Pydantic Models
Define request and response schemas with validation.

```python
from pydantic import BaseModel, Field
from typing import Optional

class ChatMessage(BaseModel):
    content: str = Field(..., min_length=1, max_length=1000)
    sender_id: str
    timestamp: Optional[str] = None
    metadata: Optional[dict] = None

class ChatResponse(BaseModel):
    message_id: str
    content: str
    timestamp: str
    status: str = "success"
```

### Request Parameters
Handle different types of request parameters.

```python
from fastapi import Query, Path, Header, Cookie, Form, File

@app.get("/items/")
async def read_items(
    q: str = Query(None, max_length=50),
    skip: int = Query(0, ge=0),
    limit: int = Query(10, le=100)
):
    pass

@app.get("/items/{item_id}")
async def read_item(
    item_id: int = Path(..., title="The ID of the item to get", ge=1)
):
    pass

@app.post("/login")
async def login(
    username: str = Form(...),
    password: str = Form(...)
):
    pass
```

## Dependency Injection

### Simple Dependencies
Create reusable functions that can be injected into endpoints.

```python
from fastapi import Depends

async def common_parameters(q: str = None, skip: int = 0, limit: int = 100):
    return {"q": q, "skip": skip, "limit": limit}

@app.get("/items/")
async def read_items(commons: dict = Depends(common_parameters)):
    return commons
```

### Dependencies with Yield
Create dependencies with setup and cleanup logic.

```python
from contextlib import asynccontextmanager

@asynccontextmanager
async def get_db():
    db = {"connection": "active"}
    try:
        yield db
    finally:
        db["connection"] = "closed"
        # Perform cleanup here

@app.get("/items/")
async def read_items(db: dict = Depends(get_db)):
    return db
```

## Security and Authentication

### OAuth2 Password Flow
Implement OAuth2 with username/password authentication.

```python
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel

app = FastAPI()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

class User(BaseModel):
    username: str
    email: str

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

### HTTP Basic Authentication
Simple username/password authentication.

```python
from fastapi import Depends, FastAPI
from fastapi.security import HTTPBasic, HTTPBasicCredentials

app = FastAPI()
security = HTTPBasic()

@app.get("/users/me")
async def read_current_user(credentials: HTTPBasicCredentials = Depends(security)):
    return {"username": credentials.username, "password": credentials.password}
```

## Background Tasks

### Adding Background Tasks
Execute operations after sending the response.

```python
from fastapi import BackgroundTasks, FastAPI

app = FastAPI()

def send_email_task(email: str, message: str):
    # Send email logic here
    print(f"Sending email to {email}: {message}")

@app.post("/send-notification/{email}")
async def send_notification(
    email: str,
    background_tasks: BackgroundTasks
):
    background_tasks.add_task(send_email_task, email, "Welcome!")
    return {"message": "Notification scheduled"}
```

## WebSocket Support

### Basic WebSocket Endpoint
Create real-time communication endpoints.

```python
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse

app = FastAPI()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            await websocket.send_text(f"Message text was: {data}")
    except Exception as e:
        print(f"WebSocket connection closed or error: {e}")
```

## Streaming Responses

### Server-Sent Events (SSE)
Stream data to clients continuously.

```python
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import asyncio

app = FastAPI()

async def generate_data():
    for i in range(10):
        yield f"data: Item {i}\\n\\n"
        await asyncio.sleep(0.5)

@app.get("/stream")
async def stream_data():
    return StreamingResponse(generate_data(), media_type="text/event-stream")
```

## Middleware

### CORS Middleware
Enable cross-origin resource sharing.

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Custom Middleware
Create custom middleware for cross-cutting concerns.

```python
from fastapi import FastAPI, Request
from fastapi.responses import Response

@app.middleware("http")
async def add_process_time_header(request: Request, call_next):
    import time
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = str(process_time)
    return response
```

## Exception Handling

### Custom Exception Handlers
Handle specific exceptions with custom responses.

```python
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse

app = FastAPI()

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={"message": exc.detail}
    )
```

## Testing

### Testing with TestClient
Test FastAPI applications using TestClient.

```python
from fastapi.testclient import TestClient

def test_read_main():
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"Hello": "World"}
```