# ChatWidget Component

A Docusaurus-compatible chatbot UI component that integrates with a backend API to provide AI-powered assistance to users.

## Features

- Floating widget or sidebar panel display modes
- Session-based greeting message (shown only once per session)
- Real-time messaging with loading states
- Error handling with retry functionality
- Responsive design for mobile and desktop
- Accessibility compliant
- Docusaurus-themed styling

## Installation

The component requires the following dependencies:

```bash
npm install @openai/chatkit-react
```

## Usage

### Basic Usage

```jsx
import ChatWidget from './components/ChatWidget/ChatWidget';

function App() {
  return (
    <div>
      <ChatWidget displayMode="floating" />  // or "sidebar"
    </div>
  );
}
```

### Integration with Docusaurus

To integrate with Docusaurus, you can add the ChatWidget to your layout:

```jsx
// In your Docusaurus layout file (e.g., src/theme/Layout/index.js)
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function Layout({children}) {
  return (
    <>
      <div>
        {/* Your existing layout */}
        {children}
        <ChatWidget displayMode="floating" />
      </div>
    </>
  );
}
```

## Props

- `displayMode`: Either "floating" (default) or "sidebar" - controls the display mode of the chat widget

## Backend API Configuration

The component communicates with a backend API at `http://localhost:8000/query` by default. To change this, you can set the `BACKEND_ENDPOINT` environment variable:

```bash
BACKEND_ENDPOINT=https://your-api.com/query
```

The API should accept POST requests with the following format:
```json
{
  "query": "user's question"
}
```

And return responses in the format:
```json
{
  "response": "AI's answer"
}
```

## Functionality

- **Session Management**: Uses localStorage to track if the greeting message has been shown in the current session
- **Message History**: Preserves message history across page reloads using localStorage
- **Error Handling**: Displays user-friendly error messages and provides retry functionality
- **Loading States**: Shows loading indicators when waiting for responses
- **Responsive Design**: Adapts to different screen sizes

## Styling

The component uses CSS modules and follows Docusaurus design principles. The main styling file is `ChatWidget.module.css` which can be customized to match your specific Docusaurus theme.