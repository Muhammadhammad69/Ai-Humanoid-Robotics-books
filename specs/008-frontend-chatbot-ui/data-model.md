# Data Model: Frontend Chatbot UI

## Entities

### ChatSession
- **Description**: Represents a single interaction session with the chatbot
- **Fields**:
  - id: string (unique identifier for the session)
  - hasSeenGreeting: boolean (tracks if greeting message has been shown)
  - createdAt: Date (when the session started)
  - lastActive: Date (when the session was last used)
  - messages: Message[] (list of messages in the session)

### Message
- **Description**: Represents a single message in the conversation
- **Fields**:
  - id: string (unique identifier for the message)
  - role: "user" | "assistant" (determines if it's from user or assistant)
  - content: string (the actual message content)
  - timestamp: Date (when the message was sent/received)
  - status: "pending" | "sent" | "error" (status of the message)
  - error?: string (error message if status is "error")

### ChatConfig
- **Description**: Configuration options for the chat interface
- **Fields**:
  - displayMode: "sidebar" | "floating" (how the chat is displayed)
  - greetingMessage: string (the greeting message to show)
  - backendEndpoint: string (the API endpoint to call)
  - theme: ChatTheme (theming options)
  - isExpanded: boolean (whether the chat interface is open)

### ChatTheme
- **Description**: Theming options for the chat interface
- **Fields**:
  - primaryColor: string (primary color for the interface)
  - secondaryColor: string (secondary color for the interface)
  - fontFamily: string (font family to use)
  - borderRadius: string (border radius for elements)
  - colorScheme: "light" | "dark" (color scheme to use)

## State Transitions

### ChatSession State Transitions
- New Session → Greeting Shown: When user first opens chat in session
- Greeting Shown → Active Chat: When user sends first message
- Active Chat → Session Closed: When user closes chat interface
- Session Closed → Active Chat: When user reopens chat in same session

### Message State Transitions
- Pending → Sent: When message is successfully sent to backend
- Pending → Error: When there's an error sending the message
- Error → Pending: When user retries a failed message
- Sent → Error: When there's an error receiving the response

## Relationships
- ChatSession contains 0..n Message entities
- Message belongs to exactly 1 ChatSession