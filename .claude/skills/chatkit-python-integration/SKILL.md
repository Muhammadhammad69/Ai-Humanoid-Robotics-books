---
name: chatkit-python-integration
description: A skill for integrating OpenAI ChatKit with Python applications, providing tools for setting up ChatKit servers, handling message lifecycles, managing conversations, processing attachments, and implementing client-server communication patterns with FastAPI.
---

# ChatKit Python Integration

## Overview

This skill provides comprehensive tools and guidance for integrating OpenAI ChatKit with Python applications. It enables developers to create robust chat applications with streaming responses, attachment handling, interactive widgets, and seamless client-server communication using FastAPI.

## Core Capabilities

### 1. Server Setup and Configuration
- FastAPI integration for ChatKit endpoints
- Thread and attachment storage configuration
- Streaming response handling
- Health check endpoints

### 2. Message Lifecycle Management
- Thread item conversion between ChatKit and Agent SDK formats
- Attachment processing with base64 encoding
- Hidden context management
- Streaming widget updates

### 3. Conversation Management
- Thread creation and persistence
- Session state management
- Context passing between client and server
- Multi-user conversation handling

### 4. Interactive Widget Handling
- Form creation and processing
- Action handling from client widgets
- Progress updates for long-running operations
- Client-side action callbacks

### 5. Error Handling and Retries
- Automatic retry mechanisms with exponential backoff
- Comprehensive error logging
- Stream error handling
- Attachment processing error management

## Quick Start

To implement a basic ChatKit server:

1. Use the server template in `assets/server_template.py` as your starting point
2. Customize the `MyChatKitServer` class with your specific business logic
3. Configure your data stores (PostgresStore, BlobStorageStore)
4. Set up attachment handling with the ThreadItemConverter
5. Test with the provided error handling mechanisms

## Implementation Patterns

### FastAPI Integration
Use the `server_setup.py` script to quickly configure a ChatKit endpoint with proper streaming response handling.

### Attachment Processing
Implement custom `ThreadItemConverter` classes using the patterns in `thread_converter.py` to handle different attachment types.

### Action Handling
Use `client_action_handler.py` as a reference for implementing server-side action processing that integrates with client-side widgets.

## Best Practices

1. Always implement proper error handling with the retry mechanisms provided
2. Use strongly-typed actions for better maintainability
3. Implement proper context management for multi-user scenarios
4. Use streaming responses for better user experience
5. Handle attachments securely with proper validation

## Resources

This skill includes the following resources:

### scripts/
- `server_setup.py`: FastAPI integration template
- `thread_converter.py`: Attachment and message conversion utilities
- `error_handling.py`: Retry mechanisms and error handling patterns
- `client_action_handler.py`: Client-server action handling implementation

### references/
- `api_reference.md`: Complete API documentation for ChatKit components

### assets/
- `server_template.py`: Complete server implementation template with all required methods
