import React, { createContext, useContext, useReducer } from 'react';

const ChatContext = createContext();

const initialState = {
  messages: [],
  isLoading: false,
  error: null,
  displayMode: 'floating', // 'floating' or 'sidebar'
  isChatOpen: false,
  hasSeenGreeting: false,
};

const chatReducer = (state, action) => {
  switch (action.type) {
    case 'SET_MESSAGES':
      return { ...state, messages: action.payload };
    case 'ADD_MESSAGE':
      return { ...state, messages: [...state.messages, action.payload] };
    case 'UPDATE_MESSAGE':
      return {
        ...state,
        messages: state.messages.map(msg =>
          msg.id === action.payload.id ? { ...msg, ...action.payload.updates } : msg
        ),
      };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    case 'CLEAR_ERROR':
      return { ...state, error: null };
    case 'SET_DISPLAY_MODE':
      return { ...state, displayMode: action.payload };
    case 'TOGGLE_CHAT':
      return { ...state, isChatOpen: !state.isChatOpen };
    case 'SET_CHAT_OPEN':
      return { ...state, isChatOpen: action.payload };
    case 'SET_HAS_SEEN_GREETING':
      return { ...state, hasSeenGreeting: action.payload };
    case 'RESET_CHAT':
      return initialState;
    default:
      return state;
  }
};

export const ChatProvider = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  const value = {
    state,
    dispatch,
    addMessage: (message) => dispatch({ type: 'ADD_MESSAGE', payload: message }),
    updateMessage: (id, updates) => dispatch({ type: 'UPDATE_MESSAGE', payload: { id, updates } }),
    setLoading: (isLoading) => dispatch({ type: 'SET_LOADING', payload: isLoading }),
    setError: (error) => dispatch({ type: 'SET_ERROR', payload: error }),
    clearError: () => dispatch({ type: 'CLEAR_ERROR' }),
    setDisplayMode: (mode) => dispatch({ type: 'SET_DISPLAY_MODE', payload: mode }),
    toggleChat: () => dispatch({ type: 'TOGGLE_CHAT' }),
    setChatOpen: (isOpen) => dispatch({ type: 'SET_CHAT_OPEN', payload: isOpen }),
    setHasSeenGreeting: (hasSeen) => dispatch({ type: 'SET_HAS_SEEN_GREETING', payload: hasSeen }),
    resetChat: () => dispatch({ type: 'RESET_CHAT' }),
  };

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
};

export const useChatContext = () => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
};