import React, { useState, useEffect, useCallback, useRef } from 'react';
import ChatHeader from './ChatHeader';
import ChatMessageList from './ChatMessageList';
import ChatInput from './ChatInput';
import styles from './ChatWidget.module.css';
import chatSessionService from '../../services/chat-session';
import chatbotApi from '../../services/chatbot-api';

// Add client-side check to prevent SSR issues
const useIsClient = () => {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    if (typeof window !== 'undefined') {
      setIsClient(true);
    }
  }, []);

  return isClient;
};

// Fix 1: Message ordering - ensure messages are appended in chronological order (oldest first, newest last)
// Fix 2: Loading state - prevent loading state from persisting across page refreshes
// Fix 3: Response field - handle both possible response field names from backend

const ChatWidget = ({ displayMode = 'floating' }) => {
  const isClient = useIsClient();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [hasSeenGreeting, setHasSeenGreeting] = useState(false);

  // Check if greeting has been shown in this session
  useEffect(() => {
    if (!isClient) return; // Only run on client side

    const greetingShown = chatSessionService.hasSeenGreeting();
    setHasSeenGreeting(greetingShown);

    // Load existing messages - maintain chronological order
    // Fix: Reset any pending/loading states to prevent phantom loading after page refresh
    const loadedMessages = chatSessionService.getMessages().map(msg => {
      // If a message was in 'pending' state from a previous session, it means the request
      // was interrupted or didn't complete. Reset to 'sent' if it was a user message that was sent
      // or 'error' if it was a failed message. Assistant messages in 'pending' state should not exist
      // in history since they're created after response.
      if (msg.status === 'pending') {
        // For user messages that were pending, they should be reset to 'sent' if they were actually sent
        // or 'error' if they failed. Since we can't know the exact state, we'll reset to 'sent'
        // but if it was an error message, we'll keep the error status
        if (msg.error) {
          return { ...msg, status: 'error' }; // Keep error status if there was an error
        } else {
          return { ...msg, status: 'sent' }; // Reset pending to sent to prevent loading after refresh
        }
      }
      return msg;
    });
    setMessages(loadedMessages); // Keep chronological order: oldest first, newest last
  }, [isClient]);

  // Show greeting message on first open of session
  useEffect(() => {
    if (!isClient) return; // Only run on client side

    if (isOpen && !hasSeenGreeting) {
      const greetingMessage = {
        id: `greeting-${Date.now()}`,
        role: 'assistant',
        content: 'Hello! Ask me anything about this book.',
        timestamp: new Date().toLocaleTimeString(),
        status: 'sent'
      };

      // Add greeting message at the end (append)
      setMessages(prev => [...prev, greetingMessage]);
      chatSessionService.markGreetingAsSeen();
      setHasSeenGreeting(true);
    }
  }, [isOpen, hasSeenGreeting, isClient]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = useCallback(async (query) => {
    if (!isClient || !query.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessageId = `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    const userMessage = {
      id: userMessageId,
      role: 'user',
      content: query,
      timestamp: new Date().toLocaleTimeString(),
      status: 'pending'
    };

    // Add user message at the end (append to show in chronological order)
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Send to backend API
      const response = await chatbotApi.sendQuery(query);

      // Update user message status to sent
      setMessages(prev =>
        prev.map(msg =>
          msg.id === userMessageId ? { ...msg, status: 'sent' } : msg
        )
      );

      // Add assistant response
      const assistantMessage = {
        id: `resp-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        role: 'assistant',
        content: response.response || response.agent_answer || 'I received your message', // Try both possible response fields
        timestamp: new Date().toLocaleTimeString(),
        status: 'sent'
      };

      // Add assistant message at the end (append to maintain chronological order)
      setMessages(prev => [...prev, assistantMessage]);

      // Save messages to storage - store in chronological order
      chatSessionService.addMessage(userMessage);
      chatSessionService.addMessage(assistantMessage);
    } catch (error) {
      // Update user message to show error
      setMessages(prev =>
        prev.map(msg =>
          msg.id === userMessageId
            ? {
                ...msg,
                status: 'error',
                error: error.message || 'Failed to send message'
              }
            : msg
        )
      );

      // Add error to storage
      chatSessionService.updateMessage(userMessageId, {
        status: 'error',
        error: error.message || 'Failed to send message'
      });
    } finally {
      setIsLoading(false);
    }
  }, [isLoading, isClient]);

  // Function to handle retry of failed messages
  const handleRetryMessage = useCallback(async (messageId) => {
    if (!isClient) return;

    const messageToRetry = messages.find(msg => msg.id === messageId);
    if (!messageToRetry || messageToRetry.role !== 'user') return;

    // Update message status to pending
    setMessages(prev =>
      prev.map(msg =>
        msg.id === messageId ? { ...msg, status: 'pending', error: null } : msg
      )
    );

    setIsLoading(true);

    try {
      // Send to backend API
      const response = await chatbotApi.sendQuery(messageToRetry.content);

      // Update user message status to sent
      setMessages(prev =>
        prev.map(msg =>
          msg.id === messageId ? { ...msg, status: 'sent', error: null } : msg
        )
      );

      // Add assistant response
      const assistantMessage = {
        id: `resp-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        role: 'assistant',
        content: response.response || response.agent_answer || 'I received your message', // Try both possible response fields
        timestamp: new Date().toLocaleTimeString(),
        status: 'sent'
      };

      // Add assistant message at the end (append to maintain chronological order)
      setMessages(prev => [...prev, assistantMessage]);

      // Save messages to storage
      chatSessionService.updateMessage(messageId, { status: 'sent', error: null });
      chatSessionService.addMessage(assistantMessage);
    } catch (error) {
      // Update user message to show error
      setMessages(prev =>
        prev.map(msg =>
          msg.id === messageId
            ? {
                ...msg,
                status: 'error',
                error: error.message || 'Failed to send message'
              }
            : msg
        )
      );

      // Add error to storage
      chatSessionService.updateMessage(messageId, {
        status: 'error',
        error: error.message || 'Failed to send message'
      });
    } finally {
      setIsLoading(false);
    }
  }, [messages, isClient]);

  // Don't render anything on the server side
  if (!isClient) {
    return null;
  }

  return (
    <div className={`${styles.chatContainer} ${styles[displayMode]} ${isOpen ? styles.open : styles.closed}`}>
      {!isOpen && 
      <button className={styles.chatToggle} onClick={toggleChat}>
        💬
      </button>
      } 
      
      {isOpen && (
        <div className={styles.chatContent}>
          <ChatHeader onClose={() => setIsOpen(false)} />
          <ChatMessageList messages={messages} onRetryMessage={handleRetryMessage} />
          <ChatInput
            onSendMessage={handleSendMessage}
            disabled={isLoading}
          />
        </div>
      )}
    </div>
  );
};

export default ChatWidget;