import React, { useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import styles from './ChatWidget.module.css';

const ChatMessageList = ({ messages = [], onRetryMessage }) => {
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  return (
    <div className={styles.messageList}>
      {messages.map((message, index) => (
        <ChatMessage
          key={message.id || index}
          message={message}
          onRetry={onRetryMessage}
        />
      ))}
      <div ref={messagesEndRef} />
    </div>
  );
};

export default ChatMessageList;