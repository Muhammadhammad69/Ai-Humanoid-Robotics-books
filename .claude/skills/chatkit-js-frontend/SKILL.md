---
name: chatkit-js-frontend
description: A skill for building frontend chat interfaces with OpenAI ChatKit, handling installation, setup, client initialization, chat UI, message handling, event handling, state management, async streaming, error handling, and FastAPI backend integration.
---

# ChatKit JS Frontend Skill

## Purpose

This skill provides comprehensive guidance for building frontend chat interfaces using OpenAI's ChatKit library. It covers installation, setup, client initialization, chat UI implementation, message handling, event handling, state management, async streaming, error handling, and integration with FastAPI backends.

## When to Use This Skill

Use this skill when you need to:
- Set up a ChatKit frontend for an AI-powered chat application
- Integrate ChatKit with React applications
- Handle chat session management and authentication
- Implement custom UI theming and customization
- Handle events, error management, and widget interactions
- Connect ChatKit frontend with FastAPI backend services

## Dependencies

- `@openai/chatkit-react` - React bindings for ChatKit
- React 18+ (for React implementation)
- OpenAI API key for backend authentication
- FastAPI (for backend integration)

## Inputs

- API configuration (client secret, domain key, custom endpoints)
- UI customization options (theme, composer settings, start screen)
- Event handlers (onError, onReady, onResponseStart, etc.)
- Widget action handlers for interactive components

## Outputs

- Fully functional ChatKit component integrated into React application
- Configured authentication and session management
- Customized UI with theme and branding
- Event handling and error management systems
- Integration with backend services

## Internal Workflow

### 1. Installation and Setup
- Install `@openai/chatkit-react` package
- Set up basic React component structure
- Import required components and hooks

### 2. Client Initialization and Configuration
- Use `useChatKit` hook for state management
- Configure API settings with client secret or custom endpoint
- Set up session management and token refresh

### 3. Chat UI Implementation
- Render `ChatKit` component with control prop
- Configure composer, start screen, and theme options
- Handle responsive design and accessibility

### 4. Message Handling and State Management
- Implement message sending and receiving
- Handle thread switching and persistence
- Manage loading states and UI feedback

### 5. Event Handling
- Implement error handling with `onError` callback
- Handle response lifecycle events
- Track analytics and user interactions

### 6. Async Streaming and Real-time Updates
- Configure real-time message streaming
- Handle connection states and reconnection
- Implement optimistic UI updates

### 7. Error Handling and Reconnection Strategies
- Set up error boundaries and recovery
- Implement retry mechanisms
- Handle network failures gracefully

### 8. FastAPI Backend Integration
- Configure API endpoints for session creation
- Implement token generation and refresh
- Handle authentication and authorization

## Example Usage

### Basic React Implementation
```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react';

export function MyChat() {
  const { control } = useChatKit({
    api: {
      async getClientSecret(existing) {
        if (existing) {
          // implement session refresh
        }

        const res = await fetch('/api/chatkit/session', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
        });
        const { client_secret } = await res.json();
        return client_secret;
      },
    },
  });

  return <ChatKit control={control} className="h-[600px] w-[320px]" />;
}
```

### Customized Chat with Start Screen
```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react';

function ChatWithStartScreen() {
  const { control } = useChatKit({
    api: {
      getClientSecret: async () => {
        const res = await fetch('/api/chatkit/session', { method: 'POST' });
        return (await res.json()).client_secret;
      },
    },
    startScreen: {
      greeting: 'Welcome to our AI Assistant! How can I help you today?',
      prompts: [
        {
          label: 'Explain a concept',
          prompt: 'Explain quantum computing in simple terms',
          icon: 'lightbulb',
        },
        {
          label: 'Write code',
          prompt: 'Write a React component that fetches data from an API',
          icon: 'square-code',
        },
      ],
    },
  });

  return <ChatKit control={control} className="h-[600px] w-[400px]" />;
}
```

### Custom API Backend Configuration
```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react';

function ChatWithCustomBackend() {
  const { control } = useChatKit({
    api: {
      url: '/api/chat',
      domainKey: 'your-domain-key-from-openai',
      fetch: async (input, init) => {
        // Add custom headers or authentication
        const headers = {
          ...init?.headers,
          'Authorization': `Bearer ${localStorage.getItem('authToken')}`,
          'X-Custom-Header': 'custom-value',
        };

        return fetch(input, {
          ...init,
          headers,
          credentials: 'include',
        });
      },
      uploadStrategy: {
        type: 'two_phase',
      },
    },
    composer: {
      attachments: {
        enabled: true,
      },
    },
  });

  return <ChatKit control={control} className="h-[600px] w-[400px]" />;
}
```

### Event Handling Implementation
```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { useEffect, useState } from 'react';

function ChatWithEventHandling() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const { control } = useChatKit({
    api: {
      getClientSecret: async () => {
        const res = await fetch('/api/chatkit/session', { method: 'POST' });
        return (await res.json()).client_secret;
      },
    },
    onReady: () => {
      console.log('ChatKit is ready');
    },
    onError: ({ error }) => {
      console.error('ChatKit error:', error);
      setError(error.message);

      // Send to error tracking service
      fetch('/api/errors', {
        method: 'POST',
        body: JSON.stringify({
          error: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString(),
        }),
        headers: { 'Content-Type': 'application/json' },
      });
    },
    onResponseStart: () => {
      setIsLoading(true);
      setError(null);
    },
    onResponseEnd: () => {
      setIsLoading(false);
    },
    onThreadChange: ({ threadId }) => {
      console.log('Thread changed to:', threadId);

      // Track in analytics
      fetch('/api/analytics/thread-change', {
        method: 'POST',
        body: JSON.stringify({ threadId }),
        headers: { 'Content-Type': 'application/json' },
      });
    },
    onLog: ({ name, data }) => {
      console.log('ChatKit log:', name, data);

      // Send to analytics
      if (name === 'message.send') {
        fetch('/api/analytics/message', {
          method: 'POST',
          body: JSON.stringify(data),
          headers: { 'Content-Type': 'application/json' },
        });
      }
    }
  });

  return (
    <div>
      {isLoading && <div className="loading-indicator">AI is thinking...</div>}
      {error && <div className="error-banner">{error}</div>}
      <ChatKit control={control} className="h-[600px] w-[400px]" />
    </div>
  );
}
```

### Widget Action Handling
```tsx
import { ChatKit, useChatKit, type Widgets } from '@openai/chatkit-react';

function ChatWithWidgets() {
  const { control } = useChatKit({
    api: {
      getClientSecret: async () => {
        const res = await fetch('/api/chatkit/session', { method: 'POST' });
        return (await res.json()).client_secret;
      },
    },
    widgets: {
      onAction: async (action, widgetItem) => {
        console.log('Widget action triggered:', action);

        switch (action.type) {
          case 'approve_request':
            // Handle locally
            await fetch('/api/requests/approve', {
              method: 'POST',
              body: JSON.stringify({
                requestId: action.payload?.requestId,
              }),
              headers: { 'Content-Type': 'application/json' },
            });

            // Notify server to update widget
            await control.ref.current?.sendCustomAction(
              {
                type: 'request_approved',
                payload: action.payload,
              },
              widgetItem.id,
            );
            break;

          case 'select_option':
            // Send selection back to server
            await control.ref.current?.sendCustomAction(action, widgetItem.id);
            break;

          case 'open_details':
            // Client-only action
            window.open(
              `/details/${action.payload?.id}`,
              '_blank',
            );
            break;
        }
      },
    },
  });

  return <ChatKit control={control} className="h-[700px] w-[500px]" />;
}
```

### Theme Customization
```tsx
const options = {
  theme: {
    colorScheme: "dark",
    color: {
      accent: {
        primary: "#2D8CFF",
        level: 2
      }
    },
    radius: "round",
    density: "compact",
    typography: { fontFamily: "'Inter', sans-serif" },
  },
  composer: {
    placeholder: "Ask anything about your data…",
  },
  startScreen: {
    greeting: "Welcome to FeedbackBot!",
  },
};
```

## Error Handling

- Implement error boundaries around ChatKit components
- Use `onError` callback for ChatKit-specific errors
- Handle network failures with retry mechanisms
- Provide user-friendly error messages
- Log errors for debugging and monitoring
- Implement graceful degradation when ChatKit is unavailable

## Notes and Best Practices

- Always implement proper session management and token refresh
- Use `getClientSecret` for secure token handling instead of exposing API keys
- Configure domain allowlist in OpenAI dashboard when using custom domains
- Implement proper cleanup for event listeners and subscriptions
- Consider accessibility requirements for chat interfaces
- Optimize bundle size by only importing needed components
- Implement proper loading states for better UX
- Use environment variables for API keys and configuration
- Implement analytics tracking for usage and engagement metrics
- Test thoroughly across different browsers and devices
- Consider implementing offline support where appropriate
- Follow security best practices for handling user data and authentication