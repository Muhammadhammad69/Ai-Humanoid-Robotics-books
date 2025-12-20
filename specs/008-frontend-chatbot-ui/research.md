# Research: Frontend Chatbot UI with ChatKit JS

## Decision: Use ChatKit JS for the chat interface
- **Rationale**: ChatKit JS is an official OpenAI library designed specifically for building AI-powered chat experiences with streaming support, making it ideal for our requirements
- **Alternatives considered**:
  - Custom React chat components from scratch: More development time and complexity
  - Third-party chat libraries: Less integration with OpenAI ecosystem
  - ChatUI Kit React: Less official and potentially less maintained

## Decision: Backend API Integration
- **Rationale**: We need to integrate ChatKit JS with our custom backend API at http://localhost:8000/query instead of using OpenAI's standard API
- **Approach**: Since ChatKit JS doesn't directly support custom endpoints, we'll need to implement a custom integration that intercepts user messages and sends them to our backend

## Decision: Component Structure
- **Rationale**: Based on the spec requirements, we need to create modular components for the chat interface
- **Components**: ChatWidget (main container), ChatHeader, ChatMessageList, ChatMessage, ChatInput

## Decision: Session Management
- **Rationale**: To show the greeting message only once per session, we need to implement session tracking
- **Approach**: Use localStorage to track if the greeting has been shown in the current session

## Decision: Styling Approach
- **Rationale**: Need to match Docusaurus theme while maintaining ChatKit's functionality
- **Approach**: Use CSS modules for custom styling and ChatKit's theming options for basic customization

## Decision: Floating Widget vs Sidebar Panel
- **Rationale**: The spec requires both display modes
- **Approach**: Create a toggleable component that can be positioned as either a floating widget or sidebar panel based on props

## Decision: Error Handling
- **Rationale**: Need to handle API failures gracefully
- **Approach**: Implement error boundaries and display user-friendly error messages with retry functionality

## Decision: Loading/Streaming States
- **Rationale**: Need to show loading indicators and handle streaming responses
- **Approach**: Use ChatKit's built-in streaming capabilities where possible, with custom loading states for API calls

## Key Findings:

1. **ChatKit JS Integration**: The library provides `useChatKit` hook and `ChatKit` component for React integration
2. **Custom API Integration**: ChatKit JS is designed to work with OpenAI's API by default, but we need to customize it to work with our backend endpoint at http://localhost:8000/query
3. **Theming**: ChatKit supports extensive theming options that can help match the Docusaurus theme
4. **Start Screen**: ChatKit has a built-in start screen with greeting messages and prompt suggestions that fits our requirements
5. **Streaming**: ChatKit supports streaming responses which aligns with our requirements
6. **Session Management**: We can use localStorage to track if the greeting has been shown in the current session
7. **Responsive Design**: ChatKit components are responsive by default