#!/bin/bash
# Script to set up a basic React project with ChatKit integration

set -e

PROJECT_NAME=${1:-"chatkit-app"}
REACT_VERSION=${2:-"latest"}

echo "Setting up ChatKit React project: $PROJECT_NAME"

# Create React app
npx create-react-app $PROJECT_NAME --template typescript
cd $PROJECT_NAME

# Install ChatKit dependencies
npm install @openai/chatkit-react

# Install additional dependencies for the example
npm install openai

# Create a Chat component
cat > src/ChatComponent.tsx << 'EOF'
import React from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';

function ChatComponent() {
  const { control } = useChatKit({
    api: {
      async getClientSecret(existing) {
        if (existing) {
          // Refresh expired token
          const res = await fetch('/api/chatkit/refresh', {
            method: 'POST',
            body: JSON.stringify({ token: existing }),
            headers: { 'Content-Type': 'application/json' },
          });
          const { client_secret } = await res.json();
          return client_secret;
        }

        // Create new session
        const res = await fetch('/api/chatkit/session', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
        });
        const { client_secret } = await res.json();
        return client_secret;
      },
    },
    theme: 'light',
    startScreen: {
      greeting: 'Welcome! How can I help you today?',
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

  return <ChatKit control={control} className="h-[600px] w-[400px] border border-gray-300 rounded-lg" />;
}

export default ChatComponent;
EOF

# Update App.tsx to include the Chat component
cat > src/App.tsx << 'EOF'
import React from 'react';
import ChatComponent from './ChatComponent';
import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>ChatKit React App</h1>
        <ChatComponent />
      </header>
    </div>
  );
}

export default App;
EOF

# Add basic styling to App.css
cat >> src/App.css << 'EOF'

.App {
  text-align: center;
  padding: 20px;
}

.App-header {
  padding: 20px;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start;
  font-size: calc(10px + 2vmin);
  color: white;
  background-color: #282c34;
}

/* Add scrollbar styling for ChatKit */
::-webkit-scrollbar {
  width: 8px;
}

::-webkit-scrollbar-track {
  background: #f1f1f1;
}

::-webkit-scrollbar-thumb {
  background: #888;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb:hover {
  background: #555;
}
EOF

# Add a basic backend proxy for development
cat > src/setupProxy.js << 'EOF'
// This file is only used in development mode
// It proxies API requests to the backend server
const { createProxyMiddleware } = require('http-proxy-middleware');

module.exports = function(app) {
  app.use(
    '/api',
    createProxyMiddleware({
      target: 'http://localhost:8000', // FastAPI backend
      changeOrigin: true,
    })
  );
};
EOF

# Install proxy dependency
npm install --save-dev http-proxy-middleware

echo "ChatKit React project setup complete!"
echo "To run the app:"
echo "1. Start your FastAPI backend server on port 8000"
echo "2. Run 'npm start' in the $PROJECT_NAME directory"