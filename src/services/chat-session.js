// Service for managing chat session state and localStorage operations

class ChatSessionService {
  constructor() {
    this.sessionIdKey = 'chatbot-session-id';
    this.greetingShownKey = 'chatbot-greeting-shown';
    this.messagesKey = 'chatbot-messages';
  }

  // Get or create a session ID
  getSessionId() {
    let sessionId = localStorage.getItem(this.sessionIdKey);
    if (!sessionId) {
      sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem(this.sessionIdKey, sessionId);
    }
    return sessionId;
  }

  // Check if greeting has been shown in current session
  hasSeenGreeting() {
    return localStorage.getItem(this.greetingShownKey) === 'true';
  }

  // Mark greeting as shown for this session
  markGreetingAsSeen() {
    localStorage.setItem(this.greetingShownKey, 'true');
  }

  // Get messages from storage
  getMessages() {
    const messagesJson = localStorage.getItem(this.messagesKey);
    return messagesJson ? JSON.parse(messagesJson) : [];
  }

  // Save messages to storage
  saveMessages(messages) {
    localStorage.setItem(this.messagesKey, JSON.stringify(messages));
  }

  // Add a message to storage
  addMessage(message) {
    const messages = this.getMessages();
    messages.push(message);
    this.saveMessages(messages);
  }

  // Update a message in storage
  updateMessage(messageId, updates) {
    const messages = this.getMessages();
    const messageIndex = messages.findIndex(msg => msg.id === messageId);
    if (messageIndex !== -1) {
      messages[messageIndex] = { ...messages[messageIndex], ...updates };
      this.saveMessages(messages);
    }
  }

  // Clear session data
  clearSession() {
    localStorage.removeItem(this.sessionIdKey);
    localStorage.removeItem(this.greetingShownKey);
    localStorage.removeItem(this.messagesKey);
  }

  // Reset greeting status (for testing purposes)
  resetGreetingStatus() {
    localStorage.removeItem(this.greetingShownKey);
  }
}

export default new ChatSessionService();