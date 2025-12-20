# API Contract: Chatbot Backend Integration

## Overview
This document defines the contract between the frontend ChatKit UI and the backend API at http://localhost:8000/query.

## Request Format
- **Endpoint**: `POST http://localhost:8000/query`
- **Content-Type**: `application/json`
- **Request Body**:
```json
{
  "query": "string"
}
```

## Response Format
- **Success Response** (200 OK):
```json
{
  "response": "string"
}
```

## Error Responses
- **Client Error** (4xx):
```json
{
  "error": "string",
  "details": "string"
}
```

- **Server Error** (5xx):
```json
{
  "error": "string",
  "details": "string"
}
```

## Frontend Integration Points

### 1. Message Submission
- When user submits a message in the chat interface
- Frontend sends the user's query to the backend endpoint
- Frontend waits for response and displays it in the chat

### 2. Error Handling
- If API call fails, frontend displays user-friendly error message
- Frontend provides retry mechanism for failed messages

### 3. Loading States
- Frontend shows loading indicator while waiting for response
- Frontend handles streaming responses if supported by the backend