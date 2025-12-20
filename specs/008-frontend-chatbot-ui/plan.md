# Implementation Plan: Frontend Chatbot UI

**Branch**: `008-frontend-chatbot-ui` | **Date**: 2025-12-20 | **Spec**: [specs/008-frontend-chatbot-ui/spec.md](specs/008-frontend-chatbot-ui/spec.md)
**Input**: Feature specification from `/specs/008-frontend-chatbot-ui/spec.md`

## Summary

Implementation of a frontend chatbot UI using ChatKit JS React bindings for integration within a Docusaurus website. The solution provides both floating widget and sidebar panel display modes, with automatic greeting message on first session, API communication to a custom backend endpoint, and proper error handling and loading states.

## Technical Context

**Language/Version**: TypeScript/JavaScript with React 18+
**Primary Dependencies**: @openai/chatkit-react, React, Docusaurus
**Storage**: localStorage for session state management
**Testing**: Jest, React Testing Library
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web application frontend component
**Performance Goals**: <200ms response time for UI interactions, <500ms for API calls
**Constraints**: <200ms p95 for UI responsiveness, must work with Docusaurus theme, accessible UI
**Scale/Scope**: Single page application, 1000+ concurrent users, 50+ message conversations

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Accuracy**: All technical claims verified against ChatKit JS documentation
- ✅ **Clarity**: Targeted to developers familiar with React and Docusaurus
- ✅ **Reproducibility**: All code snippets executable, specs traceable to Spec-Kit Plus artifacts
- ✅ **Rigor**: Using official OpenAI ChatKit JS library with citations to GitHub repo

## Project Structure

### Documentation (this feature)

```text
specs/008-frontend-chatbot-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api-contract.md  # API contract definition
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── ChatWidget/
│   │   ├── ChatWidget.jsx          # Main chat container component
│   │   ├── ChatHeader.jsx          # Chat header component
│   │   ├── ChatMessageList.jsx     # Message list component
│   │   ├── ChatMessage.jsx         # Individual message component
│   │   ├── ChatInput.jsx           # Message input component
│   │   └── ChatWidget.module.css   # Component-specific styles
├── services/
│   ├── chatbot-api.js              # API service for backend communication
│   └── chat-session.js             # Session state management
├── hooks/
│   └── useChatSession.js           # Custom hook for chat session management
└── contexts/
    └── ChatContext.js              # Context for chat state management
```

**Structure Decision**: Web application frontend component structure selected as the feature is a frontend-only implementation for Docusaurus integration. Components are organized in a modular way to support the specified requirements.

## Architecture Overview

### High-Level Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Docusaurus    │────│   ChatWidget     │────│  Backend API    │
│   Application   │    │   (React)        │    │   (localhost:   │
└─────────────────┘    ├──────────────────┤    │   8000/query)   │
                       │ • ChatHeader     │    └─────────────────┘
                       │ • ChatMessageList│
                       │ • ChatMessage    │
                       │ • ChatInput      │
                       └──────────────────┘
```

### Component Structure & Responsibilities

1. **ChatWidget**: Main container component that manages the overall chat interface state, display mode (floating/sidebar), and open/close functionality
2. **ChatHeader**: Handles the header UI with title, close button, and any additional controls
3. **ChatMessageList**: Manages the display of message history with auto-scrolling to latest message
4. **ChatMessage**: Renders individual messages with visual distinction between user and assistant
5. **ChatInput**: Handles user input with text field, send button, and Enter key submission

### Data Flow & Integration

1. User submits query via ChatInput component
2. Custom API service sends HTTP POST request to http://localhost:8000/query with payload {"query": "string"}
3. Backend processes the query and returns response
4. Response is displayed in ChatMessageList as assistant message
5. Session state is managed using localStorage to track greeting message display

### UI States & Behaviors

1. **Initial State**: Chat closed, floating widget button visible
2. **Greeting State**: When first opened in session, show greeting message "Hello! Ask me anything about this book."
3. **Active Chat State**: Message input enabled, conversation history displayed
4. **Loading State**: Input disabled, loading indicator shown while waiting for response
5. **Error State**: Error message displayed with retry option
6. **Closed State**: Chat interface hidden, only toggle button visible

## Technology Stack

- **Frontend Framework**: React 18+
- **Chat Library**: @openai/chatkit-react
- **Styling**: CSS Modules with Docusaurus theme compatibility
- **State Management**: React hooks and context API
- **API Communication**: Fetch API with async/await
- **Build Tool**: Webpack (via Docusaurus)
- **Testing**: Jest and React Testing Library

## Implementation Approach

1. **Phase 1**: Set up basic ChatKit integration with React components
2. **Phase 2**: Implement custom backend API integration (bypassing OpenAI's default API)
3. **Phase 3**: Add greeting message functionality with session tracking
4. **Phase 4**: Implement floating widget and sidebar panel display modes
5. **Phase 5**: Add loading and error states with proper UI feedback
6. **Phase 6**: Style components to match Docusaurus theme
7. **Phase 7**: Add responsive design and accessibility features

## Key Integration Points

1. **Custom API Service**: Implement a custom service to handle communication with http://localhost:8000/query instead of OpenAI's standard API
2. **Session Management**: Use localStorage to track if greeting has been shown in current session
3. **Theme Compatibility**: Ensure styling matches Docusaurus design system
4. **Streaming Support**: Implement progressive response rendering if backend supports streaming

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom API integration | Backend uses custom endpoint instead of OpenAI API | Direct ChatKit integration insufficient for custom endpoint requirement |
| Session state management | Greeting message should appear only once per session | Simple boolean state insufficient for cross-tab session tracking |
