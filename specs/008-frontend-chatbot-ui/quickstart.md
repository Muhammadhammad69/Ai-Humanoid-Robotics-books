# Quickstart: Frontend Chatbot UI

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3+ project set up
- Backend API running at http://localhost:8000/query

## Installation

1. Install the required dependencies:
```bash
npm install @openai/chatkit-react
```

2. If using TypeScript, add the types to your tsconfig.json:
```json
{
  "compilerOptions": {
    "types": ["@openai/chatkit-types"]
  }
}
```

## Basic Setup

1. Create the main ChatWidget component:
```jsx
// src/components/ChatWidget.jsx
import React, { useState, useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import './ChatWidget.css';

const ChatWidget = ({ displayMode = 'floating' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [hasSeenGreeting, setHasSeenGreeting] = useState(false);

  // Check if greeting has been shown in this session
  useEffect(() => {
    const greetingShown = localStorage.getItem('chatbot-greeting-shown');
    setHasSeenGreeting(greetingShown === 'true');
  }, []);

  const { control } = useChatKit({
    // Configuration will go here
  });

  const toggleChat = () => {
    if (!hasSeenGreeting && !isOpen) {
      // Show greeting message
      localStorage.setItem('chatbot-greeting-shown', 'true');
      setHasSeenGreeting(true);
    }
    setIsOpen(!isOpen);
  };

  return (
    <div className={`chat-container ${displayMode} ${isOpen ? 'open' : 'closed'}`}>
      <button className="chat-toggle" onClick={toggleChat}>
        {isOpen ? '×' : '💬'}
      </button>
      {isOpen && (
        <div className="chat-content">
          <ChatKit
            control={control}
            className="chat-interface"
          />
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
```

2. Integrate the ChatWidget into your Docusaurus layout:
```jsx
// In your Docusaurus layout or as a plugin
import ChatWidget from './components/ChatWidget';

// Add to your layout
<ChatWidget displayMode="floating" />
```

## Backend API Integration

To connect with your backend API at http://localhost:8000/query, you'll need to implement a custom service that intercepts messages and routes them to your endpoint instead of OpenAI's API.

## Environment Configuration

Set up your API endpoint in a config file:
```js
// config/chatbot.js
export const CHATBOT_CONFIG = {
  BACKEND_ENDPOINT: process.env.BACKEND_ENDPOINT || 'http://localhost:8000/query',
  DISPLAY_MODE: 'floating', // or 'sidebar'
  GREETING_MESSAGE: 'Hello! Ask me anything about this book.'
};
```

## Theming

Customize the chat interface to match your Docusaurus theme by adjusting the CSS variables in your ChatWidget.css file.