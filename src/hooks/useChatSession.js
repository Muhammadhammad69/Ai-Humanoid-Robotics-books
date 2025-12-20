import { useState, useEffect } from 'react';

const useChatSession = () => {
  const [sessionId, setSessionId] = useState(null);
  const [hasSeenGreeting, setHasSeenGreeting] = useState(false);

  // Initialize session
  useEffect(() => {
    // Create or retrieve session ID
    let currentSessionId = localStorage.getItem('chatbot-session-id');
    if (!currentSessionId) {
      currentSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chatbot-session-id', currentSessionId);
    }
    setSessionId(currentSessionId);

    // Check if greeting has been shown in this session
    const greetingShown = localStorage.getItem('chatbot-greeting-shown') === 'true';
    setHasSeenGreeting(greetingShown);
  }, []);

  const markGreetingAsSeen = () => {
    localStorage.setItem('chatbot-greeting-shown', 'true');
    setHasSeenGreeting(true);
  };

  const resetSession = () => {
    const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem('chatbot-session-id', newSessionId);
    localStorage.removeItem('chatbot-greeting-shown');
    setSessionId(newSessionId);
    setHasSeenGreeting(false);
  };

  return {
    sessionId,
    hasSeenGreeting,
    markGreetingAsSeen,
    resetSession,
  };
};

export default useChatSession;