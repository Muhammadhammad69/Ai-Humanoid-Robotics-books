# ChatKit API Reference

## Core Components

### ChatKit Component
The main UI component that renders the chat interface.

Props:
- `control`: The control object from useChatKit hook
- `className`: CSS classes for styling
- `style`: Inline styles

### useChatKit Hook
The main hook for managing ChatKit state and configuration.

Configuration options:
- `api`: API configuration object
- `theme`: Theme customization options
- `composer`: Composer settings
- `startScreen`: Start screen configuration
- `widgets`: Widget action handlers
- `threadItemActions`: Thread item action settings
- Event handlers: `onReady`, `onError`, `onResponseStart`, etc.

## API Configuration

### getClientSecret Method
Used to retrieve or refresh the client secret for ChatKit sessions.

```ts
{
  api: {
    async getClientSecret(existing) {
      if (existing) {
        // Refresh expired token
      }
      // Create new session
      const res = await fetch('/api/chatkit/session', { method: 'POST' });
      const { client_secret } = await res.json();
      return client_secret;
    }
  }
}
```

### Custom API Endpoint
Used for custom backend integration.

```ts
{
  api: {
    url: '/api/chat',
    domainKey: 'your-domain-key',
    fetch: async (input, init) => {
      // Custom fetch implementation with headers/auth
    }
  }
}
```

## Theming Options

### Theme Configuration
- `colorScheme`: 'light' | 'dark'
- `color`: Color customization object
- `radius`: 'none' | 'smooth' | 'round'
- `density`: 'compact' | 'normal' | 'spacious'
- `typography`: Font family settings

## Event Handlers

### Available Events
- `onReady`: When ChatKit is initialized
- `onError`: When an error occurs
- `onResponseStart`: When AI response starts
- `onResponseEnd`: When AI response ends
- `onThreadChange`: When thread changes
- `onThreadLoadStart`: When thread starts loading
- `onThreadLoadEnd`: When thread finishes loading
- `onLog`: For analytics and logging

## Widget Actions

### onAction Handler
Handles interactive widget actions in messages.

```ts
{
  widgets: {
    onAction: async (action, widgetItem) => {
      // Handle different action types
    }
  }
}
```

## Composer Options

### Attachments
Enable file attachments in the composer:
```ts
{
  composer: {
    attachments: {
      enabled: true
    }
  }
}
```

## Thread Item Actions

Enable feedback, retry, and share actions on messages:
```ts
{
  threadItemActions: {
    feedback: true,
    retry: true,
    share: true
  }
}
```