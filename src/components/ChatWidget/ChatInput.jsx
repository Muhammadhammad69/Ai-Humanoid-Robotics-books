import React, { useState, useRef } from 'react';
import styles from './ChatWidget.module.css';

const ChatInput = ({ onSendMessage, disabled = false }) => {
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef(null);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !disabled) {
      onSendMessage && onSendMessage(inputValue);
      setInputValue('');
      // Auto-resize textarea after clearing
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const handleInput = (e) => {
    setInputValue(e.target.value);

    // Auto-resize textarea
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = Math.min(textareaRef.current.scrollHeight, 120) + 'px';
    }
  };

  return (
    <form className={styles.chatInputForm} onSubmit={handleSubmit}>
      <textarea
        ref={textareaRef}
        className={styles.chatInput}
        value={inputValue}
        onInput={handleInput}
        onKeyDown={handleKeyDown}
        placeholder="Type your message here..."
        disabled={disabled}
        rows={1}
      />
      <button
        type="submit"
        className={styles.sendButton}
        disabled={disabled || !inputValue.trim()}
      >
        Send
      </button>
    </form>
  );
};

export default ChatInput;