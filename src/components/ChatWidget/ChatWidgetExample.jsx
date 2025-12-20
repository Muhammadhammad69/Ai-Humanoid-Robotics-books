import React from 'react';
import ChatWidget from './ChatWidget';

// Example integration component for Docusaurus
const ChatWidgetExample = () => {
  return (
    <div>
      <h1>Documentation Page</h1>
      <p>This is an example page to demonstrate the ChatWidget integration.</p>
      <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.</p>

      {/* The ChatWidget can be placed anywhere in your Docusaurus layout */}
      <ChatWidget displayMode="floating" />
    </div>
  );
};

export default ChatWidgetExample;